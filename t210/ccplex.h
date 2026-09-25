#ifndef T210_CCPLEX_H
#define T210_CCPLEX_H

#include <cstdint>

struct EmuState;

/*
 * CCPLEX CPU0 - the first Cortex-A57 of the Tegra X1's main CPU cluster.
 *
 * An RCM payload runs on the BPMP, but it can boot CPU0 itself with bdk's
 * ccplex_boot_cpu0(): rail on, PLLX, CCLK, the CRAIL/C0NC/CE0 partitions,
 * RAM repair, the AArch64 reset vector in SB_AA64_RESET_LOW/HIGH, then
 * RST_CPUG_CMPLX_CLR. hwtest does exactly that for its PCIe probe, because
 * PCIe answers a CPU-complex master only.
 *
 * This model watches those registers and, when the core is released with
 * every precondition met, runs it: a second Unicorn engine in AArch64 mode,
 * entered at EL3 with the MMU off at the programmed vector, sharing IRAM,
 * DRAM, TZRAM and every peripheral model with the BPMP. A release that is
 * missing something does not start the core, and says which precondition
 * was missing - on silicon the core would simply never run.
 *
 * Time: CPU0 keeps its own clock, advanced by the instructions it retires
 * and by the latency of each bus access it makes, and the main loop runs it
 * in short slices up to the BPMP's current time. WFE/WFI park it until the
 * end of the slice. Everything is a pure function of the instruction stream,
 * so runs stay deterministic.
 */

// Power-on state: CPU0 in reset, no vector, clocks off.
void     ccplex_reset(EmuState *state);

// CAR (clock and reset controller) forwarding.
void     ccplex_car_write(EmuState *state, uint32_t offset, uint32_t val);
bool     ccplex_car_read(uint32_t offset, uint32_t *out);

// SB block (SYSREG + 0x200): SB_CSR and the AArch64 reset vector.
uint32_t ccplex_sb_read(uint32_t offset);
void     ccplex_sb_write(EmuState *state, uint32_t offset, uint32_t val);

// A PMC power partition toggled / the CPU rail regulator changed. Losing any
// of them under a running core stops it.
void     ccplex_partition_changed(EmuState *state, int part, bool on);
void     ccplex_rail_changed(EmuState *state);

// Called by the MMIO front-end for every access CPU0 makes: charges the bus
// latency to CPU0's clock before the access is dispatched.
void     ccplex_bus_tick(EmuState *state);

// True while CPU0 is out of reset and executing (not parked on a hung bus).
bool     ccplex_cpu0_running();

// Run CPU0 until its clock reaches `until_us` - the BPMP's current time.
void     ccplex_run(EmuState *state, uint64_t until_us);

#endif // T210_CCPLEX_H
