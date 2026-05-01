#pragma once
#include <cstdint>
#include <unicorn/unicorn.h>
#include "../emu_state.h"

void sdmmc_init_storage(EmuState *state);
uint32_t sdmmc1_read(EmuState *state, uint64_t addr);
void sdmmc1_write(uc_engine *uc, EmuState *state, uint64_t addr, uint32_t val);
uint32_t sdmmc4_read(EmuState *state, uint64_t addr);
void sdmmc4_write(uc_engine *uc, EmuState *state, uint64_t addr, uint32_t val);
