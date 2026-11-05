#include "ccplex.h"
#include "bpmp.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <memory>

#include <unicorn/unicorn.h>

#include "../emu_state.h"
#include "memory_map.h"
#include "mmio.h"
#include "pcie.h"
#include "gic.h"

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
// RST_CPU_CMPLX_SET (TRM 5.2.113) has RST_CPUG_CMPLX's fields and reset value:
// it is the active cluster's view, and the Switch runs the G cluster. Its
// CLR partner at 0x344 is not in the TRM; it pairs the way every SET does.
#define CAR_RST_CPU_CMPLX_SET  0x340
#define CAR_RST_CPU_CMPLX_CLR  0x344
// CLK_CPUG_CMPLX (TRM 5.2.122, reset 0) and its SET/CLR strobes (5.2.154/155):
// CPUGn_CLK_STP (bits 11:8) stops core n's clock. Like the peripheral banks,
// all three read back the register.
#define CAR_CLK_CPUG_CMPLX     0x378
#define CAR_CLK_CPUG_CMPLX_SET 0x460
#define CAR_CLK_CPUG_CMPLX_CLR 0x464
#define  CPU0_CLK_STP          (1u << 8)
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
  uint32_t clk_cpug = 0;
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

  // Generic timer (ARM ARM D11): the EL1 physical and virtual timers.
  uint64_t cntp_cval = 0, cntv_cval = 0, cntvoff = 0;
  uint32_t cntp_ctl = 0, cntv_ctl = 0, cntkctl = 0, cnthctl = 0;

  // An interrupt found pending and unmasked at a block boundary: the run
  // stops there and ccplex_run() takes it.
  bool irq_take = false;
  uint64_t irqs_taken = 0, syncs_taken = 0, smcs = 0;
  uint32_t xlat_gen = 1;     // bumped when TTBR/TCR/SCTLR change
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
uc_hook h_block, h_unmapped, h_intr, h_mrs, h_msr;

bool held() { return (m.rst_cpug & (RST_CPU0_MASK | RST_NONCPU)) != 0; }

// ---- Generic timer -------------------------------------------------------
//
// The A57s' system counter runs from the 19.2 MHz TSC on the Tegra X1 (the
// device tree's arch timer node says 19200000), so the count is CPU0's own
// clock in those units: deterministic, like everything else here. Every
// access to the timer registers is answered from this model; QEMU's own
// timers never see one.
static constexpr uint64_t CNT_HZ = 19200000;

uint64_t cnt_now() { return m.ns * 12 / 625; }          // ns * 19.2e6 / 1e9

// TMR10..13 (see ccplex.h): PTV bit 31 EN, bit 30 PER, 28:0 the count of
// microseconds in the n+1 scheme (TRM 8.7.1); PCR bit 30 clears the
// interrupt. Kept with CPU0 because only its clock times them.
struct CpuTmr { uint32_t ptv = 0; uint64_t armed_ns = 0; bool pending = false; };
CpuTmr cpu_tmr[4];

uint64_t tmr_deadline(const CpuTmr &t) {
  return t.armed_ns + ((uint64_t)(t.ptv & 0x1FFFFFFF) + 1) * 1000;
}

void cpu_tmrs_update() {
  for (int i = 0; i < 4; i++) {
    CpuTmr &t = cpu_tmr[i];
    if ((t.ptv & (1u << 31)) && m.ns >= tmr_deadline(t)) {
      t.pending = true;
      if (t.ptv & (1u << 30)) {
        uint64_t per = ((uint64_t)(t.ptv & 0x1FFFFFFF) + 1) * 1000;
        while (tmr_deadline(t) <= m.ns)
          t.armed_ns += per;
      } else {
        t.ptv &= ~(1u << 31);
      }
    }
    gic_set_spi(176 + i, t.pending);
  }
}
uint64_t ns_of_cnt(uint64_t c) {
  return c > UINT64_MAX / 625 ? UINT64_MAX : (c * 625 + 11) / 12;
}

void timers_update() {
  cpu_tmrs_update();
  uint64_t p = cnt_now(), v = p - m.cntvoff;
  bool pt = (m.cntp_ctl & 1) && p >= m.cntp_cval;
  bool vt = (m.cntv_ctl & 1) && v >= m.cntv_cval;
  gic_set_ppi(GIC_PPI_PTIMER, pt && !(m.cntp_ctl & 2));
  gic_set_ppi(GIC_PPI_VTIMER, vt && !(m.cntv_ctl & 2));
}

// When the next unmasked timer fires, on CPU0's clock; UINT64_MAX for never.
uint64_t next_timer_ns() {
  uint64_t t = UINT64_MAX;
  for (const CpuTmr &c : cpu_tmr)
    if (c.ptv & (1u << 31))
      t = std::min(t, tmr_deadline(c));
  if ((m.cntp_ctl & 3) == 1)
    t = std::min(t, ns_of_cnt(m.cntp_cval));
  if ((m.cntv_ctl & 3) == 1) {
    uint64_t c = m.cntv_cval + m.cntvoff;
    if (c >= m.cntv_cval)            // no wrap
      t = std::min(t, ns_of_cnt(c));
  }
  return t;
}

// ---- EL1&0 stage-1 translation ---------------------------------------------
//
// Only for the emulator's own peeks (is the last instruction of a block a
// WFI?), so 4 KiB granules only, which is what Linux and the stubs use. A
// small direct-mapped cache, emptied when the translation registers change.
uint64_t cp_read(uc_engine *uc, uint32_t op0, uint32_t op1, uint32_t crn,
                 uint32_t crm, uint32_t op2) {
  uc_arm64_cp_reg r = {crn, crm, op0, op1, op2, 0};
  uc_reg_read(uc, UC_ARM64_REG_CP_REG, &r);
  return r.val;
}

void cp_write(uc_engine *uc, uint32_t op0, uint32_t op1, uint32_t crn,
              uint32_t crm, uint32_t op2, uint64_t val) {
  uc_arm64_cp_reg r = {crn, crm, op0, op1, op2, val};
  uc_reg_write(uc, UC_ARM64_REG_CP_REG, &r);
}

const uint8_t *host_ptr(uint64_t a, uint32_t len);

bool walk_va(uc_engine *uc, uint64_t va, uint64_t *pa) {
  uint64_t sctlr = cp_read(uc, 3, 0, 1, 0, 0);
  if (!(sctlr & 1)) {
    *pa = va;
    return true;
  }
  uint64_t tcr = cp_read(uc, 3, 0, 2, 0, 2);
  bool hi = (va >> 55) & 1;
  uint64_t ttbr = hi ? cp_read(uc, 3, 0, 2, 0, 1) : cp_read(uc, 3, 0, 2, 0, 0);
  unsigned tsz = hi ? (tcr >> 16) & 0x3F : tcr & 0x3F;
  unsigned tg = hi ? (tcr >> 30) & 3 : (tcr >> 14) & 3;
  if ((hi && tg != 2) || (!hi && tg != 0))
    return false;                    // not a 4 KiB granule
  unsigned va_bits = 64 - tsz;
  if (va_bits < 25 || va_bits > 48)
    return false;
  int levels = (int)(va_bits - 12 + 8) / 9;
  uint64_t table = ttbr & 0x0000FFFFFFFFF000ULL;
  for (int lvl = 4 - levels; lvl <= 3; lvl++) {
    unsigned shift = 12 + 9 * (3 - lvl);
    uint64_t idx = (va >> shift) & 0x1FF;
    const uint8_t *p = host_ptr(table + idx * 8, 8);
    if (!p)
      return false;
    uint64_t d;
    memcpy(&d, p, 8);
    if (!(d & 1))
      return false;
    if (lvl == 3 || !(d & 2)) {
      if (lvl == 3 && !(d & 2))
        return false;
      uint64_t mask = (1ULL << shift) - 1;
      *pa = (d & 0x0000FFFFFFFFF000ULL & ~mask) | (va & mask);
      return true;
    }
    table = d & 0x0000FFFFFFFFF000ULL;
  }
  return false;
}

struct XlatEntry { uint64_t va_page; uint64_t pa_page; uint32_t gen; };
XlatEntry xlat_cache[1024];

bool va_to_pa(uc_engine *uc, uint64_t va, uint64_t *pa) {
  uint64_t page = va >> 12;
  XlatEntry &e = xlat_cache[page & 1023];
  if (e.gen == m.xlat_gen && e.va_page == page) {
    *pa = (e.pa_page << 12) | (va & 0xFFF);
    return true;
  }
  uint64_t p;
  if (!walk_va(uc, va & ~0xFFFULL, &p))
    return false;
  e = {page, p >> 12, m.xlat_gen};
  *pa = p | (va & 0xFFF);
  return true;
}

bool read_insn(uc_engine *uc, uint64_t va, uint32_t *insn) {
  uint64_t pa;
  if (!va_to_pa(uc, va, &pa))
    return false;
  const uint8_t *p = host_ptr(pa, 4);
  if (!p)
    return false;
  memcpy(insn, p, 4);
  return true;
}

// ---- Exception entry ---------------------------------------------------------
//
// Unicorn never delivers guest exceptions (see the reset-entry note), so the
// model takes them itself: SPSR/ELR/ESR, PSTATE, then the vector. Only to
// EL1 from EL1 on SP_EL1, which is every exception a kernel without
// userspace or hypervisor takes - and it is the case where the translator's
// cached EL stays right, because the EL does not change.
enum ExcKind { EXC_SYNC = 0, EXC_IRQ = 1, EXC_FIQ = 2, EXC_SERROR = 3 };

bool take_exception(uc_engine *uc, uint64_t ret_pc, ExcKind kind,
                    uint32_t esr, uint64_t far) {
  uint64_t ps = 0, vbar = 0;
  uc_reg_read(uc, UC_ARM64_REG_PSTATE, &ps);
  unsigned el = (ps >> 2) & 3;
  if (el != 1 || !(ps & 1)) {
    printf("[ccplex] CPU0 exception (kind %d, ESR 0x%08X) at PC=0x%llX from "
           "EL%u%c: only EL1h->EL1 entry is modelled, CPU0 halted\n", kind,
           esr, (unsigned long long)ret_pc, el, (ps & 1) ? 'h' : 't');
    fflush(stdout);
    return false;
  }
  uc_reg_read(uc, UC_ARM64_REG_VBAR_EL1, &vbar);
  cp_write(uc, 3, 0, 4, 0, 0, ps);            // SPSR_EL1
  uc_reg_write(uc, UC_ARM64_REG_ELR_EL1, &ret_pc);
  if (kind == EXC_SYNC) {
    uint64_t e = esr;
    uc_reg_write(uc, UC_ARM64_REG_ESR_EL1, &e);
    uc_reg_write(uc, UC_ARM64_REG_FAR_EL1, &far);
  }
  uint64_t nps = (ps & 0xF0000000ULL) | 0x3C0 | 0x5;   // EL1h, DAIF masked
  uc_reg_write(uc, UC_ARM64_REG_PSTATE, &nps);
  uint64_t vec = vbar + 0x200 + 0x80 * (uint64_t)kind;
  uc_reg_write(uc, UC_ARM64_REG_PC, &vec);
  return true;
}

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
  if (!cpu_rail_on(state)) {
    // A fault first: the registers may well say "on", and the reason has to
    // name what is actually wrong.
    if (const char *f = cpu_rail_fault(state)) {
      if (!strcmp(f, "vdd_cpu"))
        return "CPU rail has no output (fault: vdd_cpu, buck answers but does not deliver)";
      if (!strcmp(f, "max77812"))
        return "CPU rail has no output (fault: max77812 dead)";
      return "CPU rail has no output (fault: max77621_cpu dead)";
    }
    return state->pmic_otp.load() == 0x53
               ? "CPU rail off (MAX77812 EN_CTRL.EN_M4 clear)"
               : "CPU rail off (MAX77621 VOUT_EN / MAX77620 GPIO5 EN pin)";
  }
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
    // something wakes it - an interrupt, which here means a timer firing or
    // a device line the GIC already holds. With neither in sight it sleeps
    // to the end of the slice and resumes, a spurious wake-up the
    // architecture allows and a `for(;;) wfe;` park loop turns straight back
    // into sleep. Without this that loop would spin through hundreds of
    // millions of instructions per emulated second.
    m.wfe_next = false;
    if (!gic_irq_pending()) {
      uint64_t wake = std::min(m.target_ns, next_timer_ns());
      if (wake > m.ns)
        m.ns = wake;
    }
  }
  timers_update();
  g_state->emu_usec = m.ns / 1000;
  if (m.ns >= m.target_ns || m.kill_pending) {
    uc_emu_stop(uc);
    return;
  }
  if (gic_irq_pending()) {
    uint64_t ps = 0;
    uc_reg_read(uc, UC_ARM64_REG_PSTATE, &ps);
    if (!(ps & (1u << 7))) {          // PSTATE.I clear: take it before this block
      m.irq_take = true;
      uc_emu_stop(uc);
      return;
    }
  }
  note_code_page(address);
  // Every A64 instruction is four bytes.
  m.pending = size >= 4 ? size / 4 : 1;
  uint32_t last;
  if (read_insn(uc, address + size - 4, &last))
    m.wfe_next = (last == A64_WFE || last == A64_WFI);
}

// ---- System registers --------------------------------------------------------
//
// MRS/MSR of the generic timer are answered here (and skipped by Unicorn);
// writes to the translation registers also empty the emulator's VA cache.
bool is(const uc_arm64_cp_reg *r, uint32_t op0, uint32_t op1, uint32_t crn,
        uint32_t crm, uint32_t op2) {
  return r->op0 == op0 && r->op1 == op1 && r->crn == crn && r->crm == crm &&
         r->op2 == op2;
}

uint32_t hook_mrs(uc_engine *uc, uc_arm64_reg reg, const uc_arm64_cp_reg *r,
                  void *) {
  if (r->op0 != 3 || r->crn != 14)
    return 0;
  uint64_t p = cnt_now(), v = p - m.cntvoff, out;
  if (is(r, 3, 3, 14, 0, 0))      out = CNT_HZ;                    // CNTFRQ_EL0
  else if (is(r, 3, 3, 14, 0, 1)) out = p;                         // CNTPCT_EL0
  else if (is(r, 3, 3, 14, 0, 2)) out = v;                         // CNTVCT_EL0
  else if (is(r, 3, 3, 14, 2, 0)) out = (uint32_t)(m.cntp_cval - p);
  else if (is(r, 3, 3, 14, 2, 1)) out = m.cntp_ctl | (((m.cntp_ctl & 1) && p >= m.cntp_cval) ? 4 : 0);
  else if (is(r, 3, 3, 14, 2, 2)) out = m.cntp_cval;
  else if (is(r, 3, 3, 14, 3, 0)) out = (uint32_t)(m.cntv_cval - v);
  else if (is(r, 3, 3, 14, 3, 1)) out = m.cntv_ctl | (((m.cntv_ctl & 1) && v >= m.cntv_cval) ? 4 : 0);
  else if (is(r, 3, 3, 14, 3, 2)) out = m.cntv_cval;
  else if (is(r, 3, 0, 14, 1, 0)) out = m.cntkctl;                 // CNTKCTL_EL1
  else if (is(r, 3, 4, 14, 0, 3)) out = m.cntvoff;                 // CNTVOFF_EL2
  else if (is(r, 3, 4, 14, 1, 0)) out = m.cnthctl;                 // CNTHCTL_EL2
  else return 0;
  if (reg != UC_ARM64_REG_XZR)
    uc_reg_write(uc, reg, &out);
  return 1;
}

uint32_t hook_msr(uc_engine *uc, uc_arm64_reg reg, const uc_arm64_cp_reg *r,
                  void *) {
  (void)reg;
  (void)uc;
  if (r->op0 == 3 && r->op1 == 0 && ((r->crn == 2 && r->crm == 0 && r->op2 <= 2) ||
                                     (r->crn == 1 && r->crm == 0 && r->op2 == 0))) {
    m.xlat_gen++;                    // TTBR0/TTBR1/TCR/SCTLR_EL1
    return 0;
  }
  if (r->op0 != 3 || r->crn != 14)
    return 0;
  uint64_t p = cnt_now(), v = p - m.cntvoff, val = r->val;
  if (is(r, 3, 3, 14, 0, 0))      ;                                 // CNTFRQ: fixed
  else if (is(r, 3, 3, 14, 2, 0)) m.cntp_cval = p + (uint64_t)(int64_t)(int32_t)val;
  else if (is(r, 3, 3, 14, 2, 1)) m.cntp_ctl = val & 3;
  else if (is(r, 3, 3, 14, 2, 2)) m.cntp_cval = val;
  else if (is(r, 3, 3, 14, 3, 0)) m.cntv_cval = v + (uint64_t)(int64_t)(int32_t)val;
  else if (is(r, 3, 3, 14, 3, 1)) m.cntv_ctl = val & 3;
  else if (is(r, 3, 3, 14, 3, 2)) m.cntv_cval = val;
  else if (is(r, 3, 0, 14, 1, 0)) m.cntkctl = (uint32_t)val;
  else if (is(r, 3, 4, 14, 0, 3)) m.cntvoff = val;
  else if (is(r, 3, 4, 14, 1, 0)) m.cnthctl = (uint32_t)val;
  else return 0;
  timers_update();
  return 1;
}

// ---- PSCI (ARM DEN 0022), answered on SMC -------------------------------------
//
// On a Switch the secure monitor (ATF) answers these; here the emulator is
// the monitor. One core is modelled, so CPU_ON is refused.
void psci_call(uc_engine *uc) {
  uint64_t x0 = 0, x1 = 0, ret;
  uc_reg_read(uc, UC_ARM64_REG_X0, &x0);
  uc_reg_read(uc, UC_ARM64_REG_X1, &x1);
  uint32_t fid = (uint32_t)x0;
  m.smcs++;
  switch (fid) {
  case 0x84000000: ret = 0x00010000; break;                   // PSCI_VERSION 1.0
  case 0x84000006: ret = 2; break;                            // MIGRATE_INFO_TYPE: none
  case 0x8400000A: {                                          // PSCI_FEATURES
    uint32_t q = (uint32_t)x1;
    ret = (q == 0x84000000 || q == 0x84000001 || q == 0xC4000001 ||
           q == 0x84000002 || q == 0x84000003 || q == 0xC4000003 ||
           q == 0x84000004 || q == 0xC4000004 || q == 0x84000006 ||
           q == 0x84000008 || q == 0x84000009 || q == 0x8400000A) ? 0 : (uint64_t)-1;
    break;
  }
  case 0x84000001: case 0xC4000001: ret = 0; break;          // CPU_SUSPEND: a WFI
  case 0x84000003: case 0xC4000003: ret = (uint64_t)-3; break; // CPU_ON: DENIED
  case 0x84000004: case 0xC4000004: ret = (x1 & 0xFF) == 0 ? 0 : 1; break; // AFFINITY_INFO
  case 0x84000002:                                            // CPU_OFF
    printf("[ccplex] PSCI CPU_OFF from CPU0\n");
    m.kill_pending = true;
    m.kill_reason = "PSCI CPU_OFF";
    uc_emu_stop(uc);
    ret = 0;
    break;
  case 0x84000008:                                            // SYSTEM_OFF
  case 0x84000009:                                            // SYSTEM_RESET
    printf("[ccplex] PSCI %s from CPU0\n", fid == 0x84000008 ? "SYSTEM_OFF" : "SYSTEM_RESET");
    m.kill_pending = true;
    m.kill_reason = fid == 0x84000008 ? "PSCI SYSTEM_OFF" : "PSCI SYSTEM_RESET";
    uc_emu_stop(uc);
    ret = 0;
    break;
  default: {
    static uint32_t seen[32];
    static int nseen;
    bool known = false;
    for (int i = 0; i < nseen; i++)
      known |= seen[i] == fid;
    if (!known && nseen < 32) {
      seen[nseen++] = fid;
      printf("[ccplex] SMC 0x%08X (x1=0x%llX) not implemented, returns NOT_SUPPORTED\n",
             fid, (unsigned long long)x1);
      fflush(stdout);
    }
    ret = (uint64_t)-1;
    break;
  }
  }
  uc_reg_write(uc, UC_ARM64_REG_X0, &ret);
}

bool hook_unmapped(uc_engine *uc, uc_mem_type type, uint64_t address, int size,
                   int64_t value, void *) {
  (void)size;
  (void)value;
  uint64_t pc = 0;
  uc_reg_read(uc, UC_ARM64_REG_PC, &pc);
  // The peripheral band decodes everywhere on the APB/AHB: a register the
  // emulator does not model reads back what was written, as it does for the
  // BPMP. Undecoded space outside it (the PCIe apertures' holes, nothing at
  // all) still wedges the core.
  if (address >= 0x50000000 && address < 0x80000000 &&
      type != UC_MEM_FETCH_UNMAPPED) {
    static std::vector<uint64_t> told;
    uint64_t page = address & ~0xFFFULL;
    if (std::find(told.begin(), told.end(), page) == told.end() && told.size() < 512) {
      told.push_back(page);
      printf("[ccplex] CPU0 first access to unmodelled page 0x%08llX (PC=0x%llX)\n",
             (unsigned long long)page, (unsigned long long)pc);
      fflush(stdout);
    }
    return mmio_map_page(uc, g_state, page);
  }
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
  {
    uint64_t pa = 0;
    bool ok = walk_va(uc, address, &pa);
    printf("[ccplex]   SCTLR_EL1 %08llX TCR_EL1 %llX TTBR0 %llX TTBR1 %llX; walk %s 0x%llX\n",
           (unsigned long long)cp_read(uc, 3, 0, 1, 0, 0),
           (unsigned long long)cp_read(uc, 3, 0, 2, 0, 2),
           (unsigned long long)cp_read(uc, 3, 0, 2, 0, 0),
           (unsigned long long)cp_read(uc, 3, 0, 2, 0, 1), ok ? "->" : "failed",
           (unsigned long long)pa);
  }
  fflush(stdout);
  return false;
}

void hook_intr(uc_engine *uc, uint32_t intno, void *) {
  // Unicorn does not deliver guest exceptions (see the reset-entry note).
  // SMC is answered here (PSCI) and execution goes on after it; BRK and
  // undefined instructions are taken to EL1's own vectors, which is how a
  // kernel's WARN() and BUG() work. Anything else stops the core rather
  // than resume past the faulting instruction.
  uint64_t pc = 0;
  uc_reg_read(uc, UC_ARM64_REG_PC, &pc);
  uint32_t insn = 0;
  if (intno == 13 || intno == 12) {             // EXCP_SMC (QEMU 5: 13)
    if (read_insn(uc, pc - 4, &insn) && (insn & 0xFFE0001F) == 0xD4000003) {
      psci_call(uc);
      return;
    }
  }
  if (intno == 11 && read_insn(uc, pc - 4, &insn) && (insn & 0xFFE0001F) == 0xD4000002) {
    uint64_t r = (uint64_t)-1;                  // HVC with no hypervisor
    uc_reg_write(uc, UC_ARM64_REG_X0, &r);
    return;
  }
  if (intno == 7 && read_insn(uc, pc, &insn) && (insn & 0xFFE0001F) == 0xD4200000) {
    uint32_t esr = (0x3Cu << 26) | (1u << 25) | ((insn >> 5) & 0xFFFF);
    m.syncs_taken++;
    if (take_exception(uc, pc, EXC_SYNC, esr, 0))
      return;
  }
  if (intno == 1) {                             // EXCP_UDEF
    m.syncs_taken++;
    if (read_insn(uc, pc, &insn))
      printf("[ccplex] CPU0 undefined instruction 0x%08X at PC=0x%llX, taken to EL1\n",
             insn, (unsigned long long)pc);
    if (take_exception(uc, pc, EXC_SYNC, (1u << 25), 0))
      return;
  }
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
#if UC_API_MAJOR > 2 || (UC_API_MAJOR == 2 && UC_API_MINOR >= 1)
  // Translate through the guest's own page tables (the 2.1 default, set
  // explicitly): a kernel runs with its MMU on.
  uc_ctl_tlb_mode(uc, UC_TLB_CPU);
#endif

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
  uc_hook_add(uc, &h_mrs, UC_HOOK_INSN, (void *)hook_mrs, nullptr, 1, 0,
              UC_ARM64_INS_MRS);
  if (!getenv("RCM_EMU_NO_MSR_HOOK"))
    uc_hook_add(uc, &h_msr, UC_HOOK_INSN, (void *)hook_msr, nullptr, 1, 0,
                UC_ARM64_INS_MSR);
  m.cntp_cval = m.cntv_cval = m.cntvoff = 0;
  m.cntp_ctl = m.cntv_ctl = m.cntkctl = m.cnthctl = 0;
  m.irq_take = false;
  m.xlat_gen++;

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
  case CAR_RST_CPU_CMPLX_SET:
    m.rst_cpug |= val;
    if (val & (RST_CPU0_MASK | RST_NONCPU))
      stop_cpu0("held in reset");
    break;
  case CAR_CLK_CPUG_CMPLX:     m.clk_cpug = val;   break;
  case CAR_CLK_CPUG_CMPLX_SET: m.clk_cpug |= val;  break;
  case CAR_CLK_CPUG_CMPLX_CLR: m.clk_cpug &= ~val; break;
  case CAR_RST_CPUG_CMPLX_CLR:
  case CAR_RST_CPU_CMPLX_CLR: {
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
  case CAR_RST_CPU_CMPLX_SET:
  case CAR_RST_CPU_CMPLX_CLR:
    *out = m.rst_cpug;
    return true;
  case CAR_CLK_CPUG_CMPLX:
  case CAR_CLK_CPUG_CMPLX_SET:
  case CAR_CLK_CPUG_CMPLX_CLR:
    *out = m.clk_cpug;
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
    stop_cpu0(cpu_rail_fault(state) ? "CPU rail lost its output (fault injected)"
                                    : "CPU rail switched off");
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

void ccplex_tmr_write(uint32_t off, uint32_t val) {
  if (off < 0x90 || off >= 0xB0)
    return;
  CpuTmr &t = cpu_tmr[(off - 0x90) / 8];
  if ((off - 0x90) % 8 == 4) {            // PCR
    if (val & (1u << 30)) {
      t.pending = false;
      gic_set_spi(176 + (int)((off - 0x90) / 8), false);
    }
    return;
  }
  t.ptv = val;                            // PTV: (re)armed now
  t.armed_ns = m.ns;
}

bool ccplex_cpu0_running() { return m.cpu0 == Cpu0::Running && m.uc; }

void ccplex_run(EmuState *state, uint64_t until_us) {
  if (!ccplex_cpu0_running())
    return;
  uint64_t target = until_us * 1000;
  if (m.ns >= target)
    return;
  // CPU0's clock stopped: the core keeps its state, time passes, nothing
  // executes. Clearing CPU0_CLK_STP resumes it where it was.
  if (m.clk_cpug & CPU0_CLK_STP) {
    m.ns = target;
    return;
  }

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

  // The BPMP can only rewrite CPU0's code while it runs; parked, it cannot,
  // and comparing hundreds of kernel pages before every slice is the one
  // thing that would make a booted kernel crawl.
  if (!state->bpmp_halted)
    drop_stale_code();

  // `until` is an address an A64 PC can never hold (it is not 4-aligned).
  // 0 would make a branch to address 0 end every slice before it starts.
  uc_err err;
  for (;;) {
    err = uc_emu_start(m.uc, m.pc, ~0ULL, 0, 0);
    commit_pending();
    uc_reg_read(m.uc, UC_ARM64_REG_PC, &m.pc);
    if (err != UC_ERR_OK || m.kill_pending || m.cpu0 != Cpu0::Running || !m.irq_take)
      break;
    // An interrupt was pending and unmasked at the block that starts at PC:
    // take it there, then carry on with the slice from the vector.
    m.irq_take = false;
    m.irqs_taken++;
    if (!take_exception(m.uc, m.pc, EXC_IRQ, 0, 0)) {
      m.cpu0 = Cpu0::Hung;
      break;
    }
    uc_reg_read(m.uc, UC_ARM64_REG_PC, &m.pc);
    if (m.ns >= m.target_ns)
      break;
  }
  m.in_run = false;
  g_bus_master = prev;
  state->emu_usec = bpmp_usec;

  // RCM_EMU_CPU0_PROFILE=1: one line every second of real time with where
  // CPU0 is, how much it ran and how many interrupts it took - enough to
  // tell a hung kernel from a slow one.
  {
    static int prof = -1;
    static uint64_t last_wall = 0, last_ret = 0;
    if (prof < 0)
      prof = getenv("RCM_EMU_CPU0_PROFILE") ? 1 : 0;
    if (prof) {
      uint64_t wall = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now().time_since_epoch()).count();
      if (wall - last_wall >= 1000) {
        last_wall = wall;
        printf("[cpu0prof] t=%llu us pc=0x%llX retired=%llu (+%llu) irqs=%llu syncs=%llu smcs=%llu gic=%d\n",
               (unsigned long long)(m.ns / 1000), (unsigned long long)m.pc,
               (unsigned long long)m.retired, (unsigned long long)(m.retired - last_ret),
               (unsigned long long)m.irqs_taken, (unsigned long long)m.syncs_taken,
               (unsigned long long)m.smcs, gic_highest_pending());
        last_ret = m.retired;
        char g27[160];
        gic_describe(GIC_PPI_VTIMER, g27, sizeof(g27));
        printf("[cpu0prof]   vtimer ctl %u cval %llu cnt %llu; %s\n", m.cntv_ctl,
               (unsigned long long)m.cntv_cval, (unsigned long long)(cnt_now() - m.cntvoff), g27);
        fflush(stdout);
      }
    }
  }

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
