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

// Initialize all MMIO hooks on the Unicorn engine.
void mmio_init(uc_engine *uc, EmuState *state);

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
