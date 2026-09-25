#include "ccplex.h"
#include "bpmp.h"

#include <cstdio>
#include <cstring>
#include <vector>
#include <memory>

#include <unicorn/unicorn.h>

#include "../emu_state.h"
#include "memory_map.h"
#include "mmio.h"
#include "pcie.h"

// ==========================================================================
// Registers (bdk soc/clock.h, soc/t210.h, soc/pmc.h; Tegra X1 TRM ch.5/16)
// ==========================================================================

#define CAR_CCLK_BURST_POLICY  0x020
#define  CCLK_RUN_SRC(v)       (((v) >> 4) & 0xF)
#define  CCLK_SRC_PLLX_OUT0_LJ 8
#define  CCLK_SRC_PLLX_OUT0    14
#define CAR_PLLX_BASE          0x0E0
#define  PLL_BASE_ENABLE       (1u << 30)
#define CAR_CLK_OUT_ENB_V      0x360
#define CAR_CLK_ENB_V_SET      0x440
#define CAR_CLK_ENB_V_CLR      0x444
#define  CLK_V_CPUG            0
#define  CLK_V_MSELECT         3
#define CAR_RST_CPUG_CMPLX_SET 0x450
#define CAR_RST_CPUG_CMPLX_CLR 0x454
// RST_CPUG_CMPLX (TRM 5.2.152, reset 0x2000feef). CPU0 runs once nCPUPORESET
// (CPURESET0, bit 0), nCORERESET (CORERESET0, bit 16), nL2RESET (bit 24) and
// the nonCPU region's reset (NONCPURESET, bit 29) are all released.
// ccplex_boot_cpu0() also clears PRESETDBG (bit 30), but that only resets
// the CoreSight debug logic, which a core runs without; the DBGRESET/
// WDRESET/DERESET bits in 15:4 are reserved on this SoC.
#define  RST_CPUG_CMPLX_RESET  0x2000FEEFu
#define  RST_CPU0_MASK         ((1u << 0) | (1u << 16) | (1u << 24))
#define  RST_NONCPU            (1u << 29)

// SB block, SYSREG + 0x200.
#define SB_CSR                 0x00
#define SB_AA64_RESET_LOW      0x30
#define SB_AA64_RESET_HIGH     0x34
#define  SB_AA64_RST_AARCH64_MODE_EN (1u << 0)

// PMC partitions carrying CPU0.
#define POWER_RAIL_CRAIL       0
#define POWER_RAIL_CE0         14
#define POWER_RAIL_C0NC        15

// ==========================================================================
// Timing
// ==========================================================================
//
// PLLX runs the A57s at ~1 GHz. Retiring one instruction per nanosecond is
// the right order of magnitude, and it is ten times the rate the BPMP clock
// model uses, which is the other thing that has to hold: CPU0 is the fast
// core.
//
// A bus access is not free. From the A57, a register read goes out through
// MSELECT and the APC bridge onto a 100-odd MHz APB, and costs a few hundred
// nanoseconds. Charging it keeps a udelay() loop - which is nothing but
// TIMERUS reads - at a realistic few million accesses per emulated second
// instead of a billion instructions of busy-wait.
static constexpr uint64_t NS_PER_INSN = 1;
static constexpr uint64_t NS_PER_BUS_ACCESS = 250;

// ==========================================================================
// Reset entry
// ==========================================================================
//
// A real A57 comes out of reset at EL3 in AArch64 and fetches from the
// SB_AA64_RESET vector. Unicorn creates its ARM64 core at EL1 instead (a
// backward-compatibility choice in its cpu init), and it cannot be moved:
// writing PSTATE updates the mode bits but not the translator's cached
// hflags, so the first `msr scr_el3` still decodes as EL1 and is UNDEFINED.
// Nor can an exception carry it up, because Unicorn never delivers guest
// exceptions - SMC and friends end the run.
//
// ERET can. Its helper takes the current EL from PSTATE - which the register
// write did update - reads SPSR_EL3/ELR, and rebuilds hflags for the target
// EL. So the core starts on a one-instruction trampoline in a page only CPU0
// can see, with PSTATE, SPSR_EL3 = EL3h/DAIF masked and ELR = the vector.
// The translator fetches the new PC's address from ELR at the EL it thinks
// it is at (EL1), so ELR_EL1 carries it too. One ERET later the core is at
// EL3h with interrupts masked and the MMU off: architectural reset state.
//
// The page sits at 1 TiB: inside the A57's 44-bit physical space, far
// outside the Tegra's, so it can never alias anything a payload can reach.
static constexpr uint64_t RESET_TRAMPOLINE = 0x10000000000ULL;
static constexpr uint32_t A64_ERET = 0xD69F03E0;
static constexpr uint32_t A64_WFE = 0xD503205F;
static constexpr uint32_t A64_WFI = 0xD503207F;
static constexpr uint64_t PSTATE_EL3H_DAIF = 0x3CD;

namespace {

enum class Cpu0 { Off, Running, Hung };

struct Model {
  uint32_t rst_cpug = RST_CPUG_CMPLX_RESET;
  uint32_t clk_enb_v = 0;
  uint32_t cclk_burst = 0;
  uint32_t pllx_base = 0;
  uint32_t sb_csr = 0;
  uint32_t sb_rst_lo = 0;
  uint32_t sb_rst_hi = 0;

  Cpu0 cpu0 = Cpu0::Off;
  uc_engine *uc = nullptr;
  uint64_t pc = 0;
  uint64_t ns = 0;         // CPU0's clock, emulated nanoseconds
  uint64_t target_ns = 0;  // end of the slice being run
  uint64_t pending = 0;    // instructions in the block now executing
  bool wfe_next = false;   // the block now executing ends in WFE/WFI
  bool in_run = false;
  bool kill_pending = false;
  bool reset_pending = false;
  const char *kill_reason = nullptr;

  uint64_t retired = 0;
  uint64_t bus_accesses = 0;
  uint32_t boots = 0;
  const char *last_refusal = nullptr;
};

Model m;
EmuState *g_state = nullptr;

// Pages CPU0 has executed code from, with the bytes it translated; see
// drop_stale_code().
struct CodePage {
  uint64_t base;
  uint8_t bytes[4096];
};
std::vector<std::unique_ptr<CodePage>> code_pages;
uint64_t last_code_page = ~0ULL;
uc_hook h_block, h_unmapped, h_intr;

bool held() { return (m.rst_cpug & (RST_CPU0_MASK | RST_NONCPU)) != 0; }

uint64_t entry_vector() {
  return ((uint64_t)m.sb_rst_hi << 32) | (m.sb_rst_lo & ~1u);
}

bool entry_mapped(uint64_t a) {
  return (a >= DRAM_BASE && a + 4 <= DRAM_BASE + DRAM_WINDOW_SIZE) ||
         (a >= IRAM_BASE && a + 4 <= IRAM_BASE + IRAM_SIZE);
}

const uint8_t *host_ptr(uint64_t a, uint32_t len) {
  // Valid for the identity map CPU0 runs with before (MMU off) and after
  // (the stub's 1 GiB identity blocks) enabling the MMU.
  if (a >= DRAM_BASE && a + len <= DRAM_BASE + DRAM_WINDOW_SIZE)
    return g_state->dram_low_ptr + (a - DRAM_BASE);
  if (a >= IRAM_BASE && a + len <= IRAM_BASE + IRAM_SIZE)
    return g_state->iram_ptr + (a - IRAM_BASE);
  return nullptr;
}

// Everything the core needs to actually execute once released. Returns the
// first thing missing, or nullptr.
const char *boot_blocked(EmuState *state) {
  if (!cpu_rail_on(state))
    return state->pmic_otp.load() == 0x53
               ? "CPU rail off (MAX77812 EN_CTRL.EN_M4 clear)"
               : "CPU rail off (MAX77621 VOUT_EN / MAX77620 GPIO5 EN pin)";
  if (!pmc_partition_on(POWER_RAIL_CRAIL)) return "CRAIL partition powergated";
  if (!pmc_partition_on(POWER_RAIL_C0NC))  return "C0NC partition powergated";
  if (!pmc_partition_on(POWER_RAIL_CE0))   return "CE0 partition powergated";
  if (!(m.clk_enb_v & (1u << CLK_V_CPUG))) return "CPUG clock disabled (CLK_ENB_V bit 0)";
  uint32_t src = CCLK_RUN_SRC(m.cclk_burst);
  if ((src == CCLK_SRC_PLLX_OUT0_LJ || src == CCLK_SRC_PLLX_OUT0) &&
      !(m.pllx_base & PLL_BASE_ENABLE))
    return "CCLK runs from PLLX but PLLX is disabled";
  if (!(m.clk_enb_v & (1u << CLK_V_MSELECT))) return "MSELECT clock disabled";
  if (!pcie_mselect_up())                     return "MSELECT still in reset";
  if (!(m.sb_rst_lo & SB_AA64_RST_AARCH64_MODE_EN))
    return "SB_AA64_RESET_LOW has no AArch64 vector (AArch32 CPU boot is not modelled)";
  if (!entry_mapped(entry_vector())) return "reset vector outside IRAM/DRAM";
  return nullptr;
}

void destroy_engine() {
  code_pages.clear();
  last_code_page = ~0ULL;
  if (!m.uc)
    return;
  mmio_unmap_bus(m.uc);
  uc_close(m.uc);
  m.uc = nullptr;
}

void stop_cpu0(const char *why) {
  if (m.cpu0 == Cpu0::Off)
    return;
  if (m.in_run) {
    // The core is the one executing: it just wrote the register that stops
    // it. Finish the current access, then tear down once the run returns.
    m.kill_pending = true;
    m.kill_reason = why;
    uc_emu_stop(m.uc);
    return;
  }
  printf("[ccplex] CPU0 stopped (%s) after %llu instructions, %llu bus "
         "accesses\n", why, (unsigned long long)m.retired,
         (unsigned long long)m.bus_accesses);
  fflush(stdout);
  destroy_engine();
  m.cpu0 = Cpu0::Off;
}

// ---- CPU0 engine hooks ---------------------------------------------------

// ---- Code the BPMP rewrites under CPU0 ----------------------------------
//
// Stores the BPMP makes into shared DRAM/IRAM go through its own engine, so
// this engine's translation cache never hears of them: a stub loaded for a
// new job at the address of the last one would run the old one's code. A
// full flush before every slice cures that but costs tens of milliseconds a
// time (it walks the page descriptors of the whole 44-bit space), so instead
// the pages CPU0 has executed from are remembered with a copy of their
// bytes, and before each slice any that changed have just their own
// translations dropped.
void note_code_page(uint64_t address) {
  uint64_t base = address & ~0xFFFULL;
  if (base == last_code_page)
    return;
  last_code_page = base;
  for (auto &p : code_pages)
    if (p->base == base)
      return;
  const uint8_t *src = host_ptr(base, 4096);
  if (!src || code_pages.size() >= 256)
    return;
  auto p = std::make_unique<CodePage>();
  p->base = base;
  memcpy(p->bytes, src, 4096);
  code_pages.push_back(std::move(p));
}

void drop_stale_code() {
  for (auto &p : code_pages) {
    const uint8_t *now = host_ptr(p->base, 4096);
    if (!now || memcmp(now, p->bytes, 4096) == 0)
      continue;
    uc_ctl_remove_cache(m.uc, p->base, p->base + 4096);
    memcpy(p->bytes, now, 4096);
  }
}

void commit_pending() {
  m.ns += m.pending * NS_PER_INSN;
  m.retired += m.pending;
  m.pending = 0;
}

// Once per translated block, before it runs. Accounts the previous block
// (which has now fully executed), ends the slice when CPU0's clock reaches
// its target, and parks the core after a WFE/WFI. Stopping here means the
// block is NOT executed - Unicorn leaves PC on its first instruction - so
// the next slice picks it up again.
void hook_block(uc_engine *uc, uint64_t address, uint32_t size, void *) {
  commit_pending();
  if (m.wfe_next) {
    // The block that just ran ended in WFE/WFI: the core sleeps until
    // something wakes it. Nothing in this model sends it an event, so it
    // sleeps to the end of the slice and resumes - a spurious wake-up, which
    // the architecture allows, and which a `for(;;) wfe;` park loop turns
    // straight back into sleep. Without this that loop would spin through
    // hundreds of millions of instructions per emulated second.
    m.wfe_next = false;
    if (m.ns < m.target_ns)
      m.ns = m.target_ns;
  }
  g_state->emu_usec = m.ns / 1000;
  if (m.ns >= m.target_ns || m.kill_pending) {
    uc_emu_stop(uc);
    return;
  }
  note_code_page(address);
  // Every A64 instruction is four bytes.
  m.pending = size >= 4 ? size / 4 : 1;
  if (const uint8_t *p = host_ptr(address + size - 4, 4)) {
    uint32_t last;
    memcpy(&last, p, 4);
    m.wfe_next = (last == A64_WFE || last == A64_WFI);
  }
}

bool hook_unmapped(uc_engine *uc, uc_mem_type type, uint64_t address, int size,
                   int64_t value, void *) {
  (void)size;
  (void)value;
  uint64_t pc = 0;
  uc_reg_read(uc, UC_ARM64_REG_PC, &pc);
  const char *kind = type == UC_MEM_FETCH_UNMAPPED   ? "fetch"
                     : type == UC_MEM_WRITE_UNMAPPED ? "write"
                                                     : "read";
  // Nothing decodes this address, so on silicon the transaction never
  // completes and the core waits on it for ever. Return false: the run ends
  // and the core is marked wedged, which is what the BPMP's watchdog on the
  // mailbox heartbeat is there to notice.
  printf("[ccplex] CPU0 %s of unmapped 0x%llX at PC=0x%llX: nothing answers, "
         "CPU0 wedged\n", kind, (unsigned long long)address,
         (unsigned long long)pc);
  fflush(stdout);
  return false;
}

void hook_intr(uc_engine *uc, uint32_t intno, void *) {
  // Unicorn does not deliver guest exceptions (see the reset-entry note),
  // so a fault cannot reach the stub's own vectors. Say what happened and
  // stop the core rather than resume past the faulting instruction.
  uint64_t pc = 0;
  uc_reg_read(uc, UC_ARM64_REG_PC, &pc);
  static const char *names[] = {"?", "UDEF", "SWI", "PREFETCH_ABORT",
                                "DATA_ABORT", "IRQ", "FIQ", "BKPT", "EXIT",
                                "KERNEL_TRAP", "HVC", "HYP_TRAP", "SMC"};
  printf("[ccplex] CPU0 took exception %u (%s) near PC=0x%llX; exception "
         "entry is not modelled, CPU0 halted\n", intno,
         intno < sizeof(names) / sizeof(names[0]) ? names[intno] : "?",
         (unsigned long long)pc);
  fflush(stdout);
  m.kill_pending = false;
  m.cpu0 = Cpu0::Hung;
  uc_emu_stop(uc);
}

bool start_cpu0(EmuState *state) {
  uint64_t entry = entry_vector();
  uc_engine *uc = nullptr;
  uc_err err = uc_open(UC_ARCH_ARM64, UC_MODE_ARM, &uc);
  if (err != UC_ERR_OK) {
    printf("[ccplex] cannot create the CPU0 engine: %s\n", uc_strerror(err));
    return false;
  }
  uc_ctl_set_cpu_model(uc, UC_CPU_ARM64_A57);

  // IRAM, DRAM and TZRAM are the same host memory the BPMP engine maps, so
  // a stub the BPMP copies into DRAM and the mailbox both cores poll are one
  // set of bytes. Stores from the other engine bypass this one's self-
  // modifying-code tracking, so ccplex_run() drops its translations before
  // every slice.
  err = uc_mem_map_ptr(uc, IRAM_BASE, IRAM_SIZE, UC_PROT_ALL, state->iram_ptr);
  if (err == UC_ERR_OK)
    err = uc_mem_map_ptr(uc, DRAM_BASE, DRAM_WINDOW_SIZE, UC_PROT_ALL,
                         state->dram_low_ptr);
  if (err == UC_ERR_OK)
    err = uc_mem_map(uc, RESET_TRAMPOLINE, 0x1000, UC_PROT_READ | UC_PROT_EXEC);
  if (err != UC_ERR_OK) {
    printf("[ccplex] cannot map CPU0 memory: %s\n", uc_strerror(err));
    uc_close(uc);
    return false;
  }
  mmio_map_bus(uc, state);
  uc_mem_write(uc, RESET_TRAMPOLINE, &A64_ERET, sizeof(A64_ERET));

  uint64_t v = PSTATE_EL3H_DAIF;
  uc_reg_write(uc, UC_ARM64_REG_PSTATE, &v);
  uc_arm64_cp_reg spsr_el3 = {4, 0, 3, 6, 0, PSTATE_EL3H_DAIF}; // crn crm op0 op1 op2
  uc_arm64_cp_reg elr_el3  = {4, 0, 3, 6, 1, entry};
  uc_arm64_cp_reg scr_el3  = {1, 1, 3, 6, 0, 0};  // reset: secure, nothing routed
  uc_reg_write(uc, UC_ARM64_REG_CP_REG, &spsr_el3);
  uc_reg_write(uc, UC_ARM64_REG_CP_REG, &elr_el3);
  uc_reg_write(uc, UC_ARM64_REG_CP_REG, &scr_el3);
  v = entry;
  uc_reg_write(uc, UC_ARM64_REG_ELR_EL1, &v);

  uc_hook_add(uc, &h_block, UC_HOOK_BLOCK, (void *)hook_block, nullptr, 1, 0);
  uc_hook_add(uc, &h_unmapped, UC_HOOK_MEM_UNMAPPED, (void *)hook_unmapped,
              nullptr, 1, 0);
  uc_hook_add(uc, &h_intr, UC_HOOK_INTR, (void *)hook_intr, nullptr, 1, 0);

  m.uc = uc;
  m.pc = RESET_TRAMPOLINE;
  m.ns = state->emu_usec * 1000;   // released at the BPMP's current time
  m.pending = 0;
  m.wfe_next = false;
  m.kill_pending = false;
  m.retired = 0;
  m.bus_accesses = 0;
  m.cpu0 = Cpu0::Running;
  m.boots++;
  // Released from a BPMP register write: end the BPMP's batch at the next
  // block so CPU0's first slice starts now, not up to a batch (~10 ms) later.
  if (g_bus_master == BUS_BPMP)
    bpmp_yield();
  printf("[ccplex] CPU0 released: AArch64 EL3, MMU off, entry 0x%08llX "
         "(boot #%u)\n", (unsigned long long)entry, m.boots);
  fflush(stdout);
  return true;
}

void try_release(EmuState *state) {
  const char *why = boot_blocked(state);
  if (why) {
    // On silicon the core would never run and nothing would say why. Name
    // it, once per distinct reason, so a payload that got the sequence wrong
    // learns what it got wrong.
    if (why != m.last_refusal)
      printf("[ccplex] CPU0 released from reset but cannot run: %s\n", why);
    m.last_refusal = why;
    fflush(stdout);
    return;
  }
  m.last_refusal = nullptr;
  start_cpu0(state);
}

// Out of reset but refused: on silicon the core simply starts the moment the
// last missing piece (clock, partition, rail, vector) arrives, so every such
// change looks again.
void maybe_release(EmuState *state) {
  if (m.cpu0 == Cpu0::Off && !held() && state)
    try_release(state);
}

} // namespace

// ==========================================================================

void ccplex_reset(EmuState *state) {
  g_state = state;
  if (m.in_run) {
    // A reset from inside CPU0's own slice: defer the teardown to the end
    // of the run.
    m.kill_pending = true;
    m.reset_pending = true;
    m.kill_reason = "SoC reset";
    uc_emu_stop(m.uc);
    return;
  }
  destroy_engine();
  uint32_t boots = m.boots;
  m = Model();
  m.boots = boots;
}

void ccplex_car_write(EmuState *state, uint32_t offset, uint32_t val) {
  switch (offset) {
  case CAR_CCLK_BURST_POLICY: m.cclk_burst = val; maybe_release(state); break;
  case CAR_PLLX_BASE:         m.pllx_base = val;  maybe_release(state); break;
  case CAR_CLK_OUT_ENB_V:     m.clk_enb_v = val;  maybe_release(state); break;
  case CAR_CLK_ENB_V_SET:     m.clk_enb_v |= val; maybe_release(state); break;
  case CAR_CLK_ENB_V_CLR:
    m.clk_enb_v &= ~val;
    if ((val & (1u << CLK_V_CPUG)) && m.cpu0 == Cpu0::Running) {
      // Clock stopped under a running core: it freezes where it is. Treat
      // it as wedged until the next reset, which is how bdk always follows
      // this write anyway.
      printf("[ccplex] CPUG clock gated under a running CPU0 - core frozen\n");
      m.cpu0 = Cpu0::Hung;
    }
    break;
  case CAR_RST_CPUG_CMPLX_SET:
    m.rst_cpug |= val;
    if (val & (RST_CPU0_MASK | RST_NONCPU))
      stop_cpu0("held in reset");
    break;
  case CAR_RST_CPUG_CMPLX_CLR: {
    bool was_held = held();
    m.rst_cpug &= ~val;
    if (was_held && !held() && m.cpu0 == Cpu0::Off)
      try_release(state);
    break;
  }
  default:
    break;
  }
}

bool ccplex_car_read(uint32_t offset, uint32_t *out) {
  switch (offset) {
  case CAR_CLK_OUT_ENB_V:
    *out = m.clk_enb_v;
    return true;
  case CAR_RST_CPUG_CMPLX_SET:
  case CAR_RST_CPUG_CMPLX_CLR:
    *out = m.rst_cpug;
    return true;
  default:
    return false;
  }
}

uint32_t ccplex_sb_read(uint32_t offset) {
  switch (offset) {
  case SB_CSR:             return m.sb_csr;
  case SB_AA64_RESET_LOW:  return m.sb_rst_lo;
  case SB_AA64_RESET_HIGH: return m.sb_rst_hi;
  default:                 return 0;
  }
}

void ccplex_sb_write(EmuState *state, uint32_t offset, uint32_t val) {
  switch (offset) {
  case SB_CSR:
    // NS_RST_VEC_WR_DIS (bit 1) is sticky until reset. It only locks out
    // non-secure writers, and the vector write below does not check it.
    m.sb_csr |= val;
    break;
  case SB_AA64_RESET_LOW:  m.sb_rst_lo = val; maybe_release(state); break;
  case SB_AA64_RESET_HIGH: m.sb_rst_hi = val & 0xFFF; maybe_release(state); break;
  default: break;
  }
}

void ccplex_partition_changed(EmuState *state, int part, bool on) {
  if (on) {
    maybe_release(state);
    return;
  }
  if (part == POWER_RAIL_CRAIL) stop_cpu0("CRAIL partition powergated");
  if (part == POWER_RAIL_C0NC)  stop_cpu0("C0NC partition powergated");
  if (part == POWER_RAIL_CE0)   stop_cpu0("CE0 partition powergated");
}

void ccplex_rail_changed(EmuState *state) {
  if (m.cpu0 != Cpu0::Off && !cpu_rail_on(state))
    stop_cpu0("CPU rail switched off");
  else
    maybe_release(state);
}

void ccplex_preconditions_changed(EmuState *state) { maybe_release(state); }

void ccplex_bus_tick(EmuState *state) {
  m.ns += NS_PER_BUS_ACCESS;
  m.bus_accesses++;
  state->emu_usec = m.ns / 1000;
}

uint64_t ccplex_now_us() { return m.ns / 1000; }

bool ccplex_cpu0_running() { return m.cpu0 == Cpu0::Running && m.uc; }

void ccplex_run(EmuState *state, uint64_t until_us) {
  if (!ccplex_cpu0_running())
    return;
  uint64_t target = until_us * 1000;
  if (m.ns >= target)
    return;

  // While CPU0 runs, it is the bus master and "now" is its clock: every
  // register model reads state->emu_usec, and must see CPU0's time - a
  // model timing CPU0's own sequence (PERST# after Trefclkstable, say) has
  // to measure it on the clock CPU0 waits on. Both are put back afterwards.
  // CPU0 trails the BPMP by up to a slice, so a model both cores touch sees
  // time step back at the switch; such models must not assume `now` only
  // grows.
  uint64_t bpmp_usec = state->emu_usec;
  BusMaster prev = g_bus_master;
  g_bus_master = BUS_CPU0;
  state->emu_usec = m.ns / 1000;
  m.target_ns = target;
  m.in_run = true;

  drop_stale_code();

  // `until` is an address an A64 PC can never hold (it is not 4-aligned).
  // 0 would make a branch to address 0 end every slice before it starts.
  uc_err err = uc_emu_start(m.uc, m.pc, ~0ULL, 0, 0);

  commit_pending();
  uc_reg_read(m.uc, UC_ARM64_REG_PC, &m.pc);
  m.in_run = false;
  g_bus_master = prev;
  state->emu_usec = bpmp_usec;

  if (m.kill_pending) {
    m.kill_pending = false;
    const char *why = m.kill_reason ? m.kill_reason : "stopped";
    m.cpu0 = Cpu0::Running;          // so stop_cpu0 does the teardown
    stop_cpu0(why);
    if (m.reset_pending)
      ccplex_reset(state);
    return;
  }
  if (err != UC_ERR_OK && m.cpu0 == Cpu0::Running) {
    printf("[ccplex] CPU0 wedged at PC=0x%llX: %s\n",
           (unsigned long long)m.pc, uc_strerror(err));
    fflush(stdout);
    m.cpu0 = Cpu0::Hung;
  }
}
