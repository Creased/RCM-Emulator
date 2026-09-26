#include "bpmp.h"

#include <cstdio>
#include <cstring>

#include "../emu_state.h"
#include "memory_map.h"

namespace {

// ---- TIMERUS pacing --------------------------------------------------------
//
// Real silicon retires a few hundred BPMP instructions per microsecond; this
// clock credits ten. That dilation is what makes busy-waits cheap to emulate,
// and it is harmless for the usual `while (now - start < us)` wait. It is not
// harmless for a wait that needs to SEE a particular counter value. bdk's own
// fan_get_speed() is one:
//
//     int timer = get_tmr_us() + 2000000;
//     while ((timer - get_tmr_us()) > 0)   // int - u32: unsigned, so "!= 0"
//
// GCC compiles that to `cmp; bne`: the loop ends only on the exact deadline.
// Its body is ~30 instructions - a tenth of a microsecond on hardware, three
// microseconds here - so the emulated counter stepped past the deadline and
// the loop spun until TIMERUS wrapped, about 72 emulated minutes later. That
// is where a hwtest sweep used to stall, in the fan spin-up test.
//
// So polls are paced the way hardware paces them: while TIMERUS reads come
// at most POLL_WINDOW instructions apart - under a microsecond at real speed -
// the clock advances by at most 1 us from one read to the next. Time stays
// monotonic for every observer (only increments are capped), a wait still
// ends at the emulated time it asked for, and a loop that polls less often
// than that runs on the plain instruction clock.
constexpr uint64_t POLL_WINDOW = 200;

struct Clock {
  uint64_t limit = 0;     // insn_count at which the current batch ends
  uint32_t pending = 0;   // instructions of the block now executing
  // TIMERUS pacing (see above).
  bool polling = false;
  uint64_t poll_insn = 0; // insn_count at the last TIMERUS read
  uint64_t poll_usec = 0; // value it returned
  // Diagnostics.
  uint64_t loop_addr = 0;
  uint64_t loop_insns = 0;
  bool stall_logged = false;
  bool nop_slide_reported = false;
};

Clock clk;
uc_hook h_block;

// Credit `n` retired instructions. emu_usec ticks once per ten, exactly as
// the per-instruction counter used to tick it, and anything a FLOW_CTLR
// timed halt added to emu_usec directly is preserved.
inline void commit(EmuState *s, uint32_t n) {
  uint64_t before = s->insn_count;
  s->insn_count += n;
  uint64_t delta = s->insn_count / 10 - before / 10;
  if (clk.polling && delta) {
    if (s->insn_count - clk.poll_insn <= POLL_WINDOW) {
      uint64_t cap = clk.poll_usec + 1;
      uint64_t room = cap > s->emu_usec ? cap - s->emu_usec : 0;
      if (delta > room)
        delta = room;
    } else {
      clk.polling = false;
    }
  }
  s->emu_usec += delta;
}

inline const uint8_t *code_ptr(const EmuState *s, uint64_t a, uint32_t len) {
  if (a >= IRAM_BASE && a + len <= IRAM_BASE + IRAM_SIZE)
    return s->iram_ptr + (a - IRAM_BASE);
  if (a >= DRAM_BASE && a + len <= DRAM_BASE + DRAM_WINDOW_SIZE)
    return s->dram_low_ptr + (a - DRAM_BASE);
  return nullptr;
}

// Instructions in a translated block, counted the way Unicorn's per-
// instruction hook saw them. ARM: all four bytes. Thumb: a halfword whose top
// five bits are 0b11101, 0b11110 or 0b11111 opens a 32-bit instruction - on
// this core (Unicorn's Cortex-A15) that includes the Thumb-1 BL/BLX prefix
// and suffix pair, which decodes as one instruction.
uint32_t decode_insns(uc_engine *uc, const EmuState *s, uint64_t addr,
                      uint32_t size) {
  uint32_t cpsr = 0;
  uc_reg_read(uc, UC_ARM_REG_CPSR, &cpsr);
  if (!(cpsr & (1u << 5)))
    return size >= 4 ? size / 4 : 1;

  static uint8_t scratch[8192];
  const uint8_t *p = code_ptr(s, addr, size);
  if (!p) {
    // Code outside IRAM/DRAM (the zero-filled iROM, low scratch): rare.
    uint32_t n = size < sizeof(scratch) ? size : (uint32_t)sizeof(scratch);
    if (uc_mem_read(uc, addr, scratch, n) != UC_ERR_OK)
      return size >= 2 ? size / 2 : 1;
    p = scratch;
    size = n;
  }
  uint32_t n = 0;
  for (uint32_t i = 0; i + 1 < size; n++) {
    uint16_t hw = (uint16_t)(p[i] | (p[i + 1] << 8));
    i += ((hw & 0xF800) >= 0xE800) ? 4 : 2;
  }
  return n ? n : 1;
}

// Decoding a hot block on every execution was most of this hook's cost, so
// counts are cached per block. A block is identified by address and size,
// and fingerprinted by its first and last words: code rewritten in place
// (a relocation, a chainloaded payload) almost always changes one of those,
// and when it does not, the translator's own retranslation still gives the
// same instruction boundaries unless the 16/32-bit mix inside changed too.
struct CountCacheEntry {
  uint32_t addr;
  uint32_t head;
  uint32_t tail;
  uint16_t size;
  uint16_t count;
};
constexpr size_t kCountCacheSize = 1u << 16;
CountCacheEntry count_cache[kCountCacheSize];

inline uint32_t load32(const uint8_t *p) {
  uint32_t v;
  memcpy(&v, p, 4);
  return v;
}

uint32_t count_insns(uc_engine *uc, const EmuState *s, uint64_t addr,
                     uint32_t size) {
  const uint8_t *p = (size >= 4 && size < 0x10000) ? code_ptr(s, addr, size)
                                                   : nullptr;
  if (!p)
    return decode_insns(uc, s, addr, size);
  uint32_t head = load32(p), tail = load32(p + size - 4);
  CountCacheEntry &e = count_cache[(addr >> 1) & (kCountCacheSize - 1)];
  if (e.addr == (uint32_t)addr && e.size == size && e.head == head &&
      e.tail == tail && e.count)
    return e.count;
  uint32_t n = decode_insns(uc, s, addr, size);
  e.addr = (uint32_t)addr;
  e.size = (uint16_t)size;
  e.head = head;
  e.tail = tail;
  e.count = (uint16_t)n;
  return n;
}

// A payload that jumps into zeroed memory executes `movs r0, r0` (Thumb
// 0x0000) or `andeq r0, r0, r0` (ARM) until it hits something. No compiler
// emits sixteen zero bytes at the start of a block, so that is the signal.
// The low 16 MB (a null function pointer lands there; the iROM has no
// contents) is zero-filled RAM too, read through the engine as it is not
// one of the host views.
bool nop_slide(uc_engine *uc, const EmuState *s, uint64_t addr) {
  static const uint8_t zero[16] = {0};
  if (addr < 0x01000000) {
    uint8_t buf[16];
    return uc_mem_read(uc, addr, buf, sizeof(buf)) == UC_ERR_OK &&
           memcmp(buf, zero, sizeof(zero)) == 0;
  }
  bool in_range = (addr >= 0x40030000 && addr < IRAM_BASE + IRAM_SIZE) ||
                  (addr >= DRAM_BASE && addr < DRAM_BASE + DRAM_WINDOW_SIZE);
  if (!in_range)
    return false;
  const uint8_t *p = code_ptr(s, addr, 16);
  if (!p)
    return false;
  return memcmp(p, zero, sizeof(zero)) == 0;
}

// Once per translated block, before it runs. Credits the previous block,
// which has now fully executed. When the batch budget is spent it stops the
// engine here, and Unicorn does not run this block at all - PC stays on its
// first instruction and the next batch starts there - so it is not counted.
void hook_block(uc_engine *uc, uint64_t address, uint32_t size, void *ud) {
  EmuState *s = (EmuState *)ud;
  commit(s, clk.pending);
  clk.pending = 0;
  if (s->insn_count >= clk.limit) {
    uc_emu_stop(uc);
    return;
  }

  if (nop_slide(uc, s, address)) {
    if (!clk.nop_slide_reported) {
      clk.nop_slide_reported = true;
      printf("\n[emu] NOP-slide detected at 0x%08llX (zeroed memory being "
             "executed) - emulation paused\n", (unsigned long long)address);
      fflush(stdout);
    }
    s->paused = true;
    uc_emu_stop(uc);
    return;
  }

  clk.pending = count_insns(uc, s, address, size);

  // A single-instruction self-loop (`b .`) spinning for a long time is a
  // payload that has given up; say so once when it finally leaves it.
  if (size <= 4 && address == clk.loop_addr) {
    clk.loop_insns += clk.pending;
  } else {
    if (clk.loop_insns > 100000 && !clk.stall_logged) {
      printf("[trace] STALL at PC=0x%08llX (looped %llu times)\n",
             (unsigned long long)clk.loop_addr,
             (unsigned long long)clk.loop_insns);
      clk.stall_logged = true;
    } else if (clk.loop_insns == 0) {
      clk.stall_logged = false;
    }
    clk.loop_addr = address;
    clk.loop_insns = 0;
  }
}

} // namespace

void bpmp_attach(uc_engine *uc, EmuState *state) {
  uc_hook_add(uc, &h_block, UC_HOOK_BLOCK, (void *)hook_block, state, 1, 0);
}

uc_err bpmp_run(uc_engine *uc, EmuState *state, uint64_t begin,
                uint64_t max_insns) {
  clk.limit = state->insn_count + (max_insns ? max_insns : 1);
  clk.pending = 0;
  // `until` must be an address the core can never reach. 0 is not one: a
  // null function pointer took the BPMP there, and from then on every start
  // returned at once without running anything - the main loop spun forever
  // waiting for instructions to retire. The PC is 32 bits wide.
  uc_err err = uc_emu_start(uc, begin, 1ULL << 32, 0, 0);
  // If something other than the budget ended the run (an MMIO handler
  // calling uc_emu_stop, a fault), the block in flight has still run to its
  // end - Unicorn only checks for a stop between blocks - so credit it.
  commit(state, clk.pending);
  clk.pending = 0;
  return err;
}

void bpmp_clock_reset() {
  clk = Clock();
  memset(count_cache, 0, sizeof(count_cache));
}

uint32_t bpmp_timerus_read(EmuState *state) {
  clk.polling = true;
  clk.poll_insn = state->insn_count;
  clk.poll_usec = state->emu_usec;
  return (uint32_t)state->emu_usec;
}

void bpmp_clock_jumped() {
  clk.polling = false;
}

void bpmp_yield() {
  clk.limit = 0;
}
