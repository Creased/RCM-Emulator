#ifndef T210_BPMP_H
#define T210_BPMP_H

#include <cstdint>
#include <unicorn/unicorn.h>

struct EmuState;

/*
 * BPMP execution and its deterministic clock.
 *
 * Emulated time is derived from retired instructions, never from the host:
 * emu_usec = skipped_us + insn_count / 10, where skipped_us is whatever a
 * FLOW_CTLR timed halt jumped over. That makes a run reproducible - the same
 * payload sees the same TIMERUS value at the same instruction every time -
 * which scripted input (--input-script, --auto-te-script) depends on.
 *
 * The instruction count used to come from a UC_HOOK_CODE callback on every
 * single instruction, which kept Unicorn from running any translated block
 * at native speed. It now comes from a block hook: the block's instructions
 * are counted once, from its bytes, and credited when the block has run. The
 * totals are exactly what per-instruction counting gave. What changed is
 * resolution inside a block: an MMIO read there sees the time at the start
 * of its block, not at its own instruction - a fraction of a microsecond at
 * 10 instructions per microsecond.
 */

// Install the clock's block hook on the BPMP engine.
void   bpmp_attach(uc_engine *uc, EmuState *state);

// Run from `begin` (bit 0 = Thumb) until at least `max_insns` more
// instructions have retired, or something stops the engine. Batches end on
// a block boundary.
uc_err bpmp_run(uc_engine *uc, EmuState *state, uint64_t begin,
                uint64_t max_insns);

// Forget any half-accounted block (soft reboot).
void   bpmp_clock_reset();

// TIMERUS_CNTR_1US as the BPMP reads it. See the pacing note in bpmp.cpp:
// polls issued close together never see the counter skip a value.
uint32_t bpmp_timerus_read(EmuState *state);

// Time jumped forward outside instruction accounting (a FLOW_CTLR timed
// halt): whatever poll loop was being paced is over.
void   bpmp_clock_jumped();

// End the current batch at the next block boundary - a clean stop, unlike
// uc_emu_stop() from an MMIO callback, which abandons the block in flight
// with the PC back at its start. Used when CPU0 is released, so its first
// slice starts now rather than a whole batch later.
void   bpmp_yield();

#endif // T210_BPMP_H
