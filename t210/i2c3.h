#ifndef T210_I2C3_H
#define T210_I2C3_H

#include <cstdint>

struct EmuState;

// I²C3 controller dispatcher (base 0x7000C500).
// Models enough of the Tegra I²C controller state machine (normal + packet
// modes) to satisfy Hekate's bdk/soc/i2c.c driver, with an STMFTS touchscreen
// (FTS4) stub at slave 0x49 sitting behind it.
uint32_t i2c3_read(EmuState *state, uint64_t addr);
void     i2c3_write(EmuState *state, uint64_t addr, uint32_t val);

#endif
