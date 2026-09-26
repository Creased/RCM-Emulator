#ifndef T210_MMIO_H
#define T210_MMIO_H

#include <cstdint>
#include <unicorn/unicorn.h>

/*
 * Tegra X1 MMIO Handler
 *
 * Routes memory-mapped IO reads/writes to peripheral handlers.
 * Unknown registers return 0 / ignore writes (permissive stub).
 */

// Forward declarations for peripheral modules.
struct EmuState;

// Which processor issued the access being dispatched. Two masters share one
// register model: the BPMP (the ARM7 every RCM payload runs on) and CPU0 of
// the CCPLEX (Cortex-A57), which a payload can boot through bdk's
// ccplex_boot_cpu0(). They do not see the same bus - PCIe and MSELECT answer
// a CPU-complex master only (Tegra X1 TRM ch.16/ch.19) - so the few models
// that care consult this. Both cores run on the emulator's one thread, one
// slice at a time, so a global is exact: whoever is running is the master.
enum BusMaster : uint8_t {
  BUS_BPMP = 0,
  BUS_CPU0 = 1,
};
extern BusMaster g_bus_master;

// Initialize all MMIO hooks on the Unicorn engine.
void mmio_init(uc_engine *uc, EmuState *state);

// Map the same peripheral windows into another engine (CPU0's), backed by the
// same register models. Everything in the 0x50000000..0x7FFFFFFF band plus
// the PCIe apertures low in the map.
void mmio_map_bus(uc_engine *uc, EmuState *state);
// Forget the windows mapped into `uc`, before that engine is closed.
void mmio_unmap_bus(uc_engine *uc);

// The shared dispatch both engines' MMIO callbacks funnel into. `size` is the
// access width in bytes (1, 2 or 4).
uint32_t mmio_bus_read(EmuState *state, uint64_t addr, unsigned size);
void     mmio_bus_write(EmuState *state, uint64_t addr, unsigned size,
                        uint64_t value);

// Reset the per-run peripheral state that a soft reboot must not carry over.
// `power_cycle`: the PMIC reset too (its register files return to their
// seeds); otherwise only the SoC reset (PMC MAIN_RST) and the PMIC keeps
// its rails.
void mmio_soft_reset(EmuState *state, bool power_cycle);

// PMC power-partition state (APBDEV_PMC_PWRGATE_STATUS bit `part`).
bool pmc_partition_on(int part);

// APBDEV_PMC_SECURE_SCRATCH`n` (n = 0..7), written by hardware: the SE
// deposits its context-save key there.
void pmc_secure_scratch_write(unsigned n, uint32_t value);

// True when the CCPLEX CPU rail is up: the MAX77621 on Erista (enabled, and
// its EN pin driven by MAX77620 GPIO5), the MAX77812 M4 phase on Mariko.
bool cpu_rail_on(EmuState *state);

// Individual peripheral handlers.
uint32_t timer_read(EmuState *state, uint64_t addr);
void     timer_write(EmuState *state, uint64_t addr, uint32_t val);

uint32_t gpio_read(EmuState *state, uint64_t addr);
void     gpio_write(EmuState *state, uint64_t addr, uint32_t val);

uint32_t i2c_read(EmuState *state, uint64_t addr);
void     i2c_write(EmuState *state, uint64_t addr, uint32_t val);

uint32_t display_read(EmuState *state, uint64_t addr);
void     display_write(EmuState *state, uint64_t addr, uint32_t val);

uint32_t pmc_read(EmuState *state, uint64_t addr);
void     pmc_write(EmuState *state, uint64_t addr, uint32_t val);

uint32_t clk_rst_read(EmuState *state, uint64_t addr);
void     clk_rst_write(EmuState *state, uint64_t addr, uint32_t val);

uint32_t fuse_read(EmuState *state, uint64_t addr);
void     fuse_write(EmuState *state, uint64_t addr, uint32_t val);

uint32_t misc_read(EmuState *state, uint64_t addr);
void     misc_write(uc_engine *uc, EmuState *state, uint64_t addr, int64_t value, int size);

#endif // T210_MMIO_H
