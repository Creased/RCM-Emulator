#include "mmio.h"
#include "../emu_state.h"
#include "i2c3.h"
#include "memory_map.h"
#include "se_engine.h"
#include "tegra_bl.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <map>
#include <unistd.h>
#include <vector>

#define BIT(n) (1U << (n))

static uint32_t pmc_scratch0 = 0;
static uint32_t pmc_scratch37 = 0;
static std::map<uint64_t, uint32_t> mmio_regs;

/*
 * Central MMIO dispatcher.
 *
 * Unicorn hooks for unmapped memory access route here.
 * We dispatch to the appropriate peripheral handler based on address range.
 */

// ==================== GPIO ====================
// Button state is stored in EmuState and read via GPIO registers.
// VOL_UP = GPIO_X6 (port X, pin 6), VOL_DOWN = GPIO_X7, POWER = GPIO_X0 (PMC)

uint32_t gpio_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - GPIO_BASE);

  // hekate reads buttons via btn_read() which accesses GPIO port X
  // Port X is in Bank 6. Offset for Port X starts at 0x530.
  // CNF=0x530, OE=0x534, OUT=0x538, IN=0x53C.
  if (offset == 0x53C) {
    uint32_t val = 0xFF; // All pins high (buttons not pressed, active low)
    if (state->btn_vol_up)
      val &= ~(1 << 6); // VOL_UP  = PX6
    if (state->btn_vol_down)
      val &= ~(1 << 7); // VOL_DOWN = PX7
    printf("[gpio] R: Port X IN = 0x%02X\n", val);
    return val;
  }

  if (offset == 0x61C) {
    printf("[gpio] R: Port Z IN = 0x00 (SD Inserted)\n");
    return 0; // Bit 1 = 0 (Inserted)
  }

  printf("[gpio] R: offset 0x%X = 0\n", offset);
  return 0;
}

void gpio_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - GPIO_BASE);
  printf("[gpio] W: offset 0x%X = 0x%08X\n", offset, val);
  (void)state;
}

// ==================== I2C ====================
// hekate uses I2C5 for MAX77620 (PMIC) and MAX17050 (fuel gauge)
// We stub the I2C transaction to return simulated values.

// I2C register offsets
#define I2C_CNFG 0x00
#define I2C_CMD_ADDR0 0x04
#define I2C_CMD_DATA1 0x0C
#define I2C_STATUS 0x1C

// MAX77620 addresses and registers
#define MAX77620_I2C_ADDR 0x3C
#define MAX77620_RTC_ADDR 0x68

// MAX17050 fuel gauge
#define MAX17050_ADDR 0x36
#define MAX17050_REP_SOC 0x06
#define MAX17050_VCELL 0x09

static uint8_t i2c_slave_addr = 0;
static uint8_t i2c_reg_addr = 0;

// ==================== Synaptics RMI4 touch controller (I2C addr 0x4C on I2C1) ====================
// Synaptics S7509A capacitive touch IC.  hekate polls two registers per cycle:
//   0x01 – F01 interrupt status  (bit 2 = F11 touch interrupt)
//   0x10 – F11 finger data       (state, X/Y/W/Z in successive registers)
//
// hekate reads DATA1 (bytes 0-3) and optionally DATA2 (bytes 4-7) per
// transaction via auto-increment.  The register pointer is set by writing
// the register address as a 1-byte payload to the device.
//
// Coordinate system: portrait 720×1280.
//   reg 0x10 = finger_state  (0x01 = one finger)
//   reg 0x11 = X[11:4]
//   reg 0x12 = Y[11:4]
//   reg 0x13 = X[3:0]<<4 | Y[3:0]
//   reg 0x14 = Wx<<4 | Wy
//   reg 0x15 = Z (pressure)

static uint8_t rmi4_regs[256] = {0};
static uint8_t rmi4_read_ptr  = 0;
static bool    rmi4_initialized = false;
static bool    rmi4_touch_updated = false; // prevent double-advance per poll

static void rmi4_init() {
    memset(rmi4_regs, 0, sizeof(rmi4_regs));
    rmi4_initialized = true;
}

static void rmi4_update_touch(EmuState *state) {
    int phase = state->touch_phase.load();
    uint16_t x = state->touch_x;
    uint16_t y = state->touch_y;

    if (phase == 1) {
        rmi4_regs[0x01] = 0x04;                        // F11 interrupt
        rmi4_regs[0x10] = 0x01;                        // finger 0 present
        rmi4_regs[0x11] = (x >> 4) & 0xFF;             // X[11:4]
        rmi4_regs[0x12] = (y >> 4) & 0xFF;             // Y[11:4]
        rmi4_regs[0x13] = ((x & 0xF) << 4) | (y & 0xF);
        rmi4_regs[0x14] = 0x44;                        // Wx=4, Wy=4
        rmi4_regs[0x15] = 0x40;                        // pressure
        state->touch_phase = 2;
        printf("[touch] RMI4: DOWN at portrait (%d,%d)\n", x, y);
    } else if (phase == 2) {
        rmi4_regs[0x01] = 0x04;                        // F11 interrupt (lift)
        rmi4_regs[0x10] = 0x00;                        // no finger
        memset(&rmi4_regs[0x11], 0, 5);
        state->touch_phase = 3;
        printf("[touch] RMI4: UP\n");
    } else {
        rmi4_regs[0x01] = 0x00;
        rmi4_regs[0x10] = 0x00;
    }
}

// Pack 4 consecutive rmi4_regs bytes into a little-endian uint32_t
static uint32_t rmi4_read4(uint8_t start) {
    uint32_t v = 0;
    for (int i = 0; i < 4; i++)
        v |= (uint32_t)rmi4_regs[(start + i) & 0xFF] << (i * 8);
    return v;
}

#define MAX77620_REG_ONOFFSTAT 0x15
#define MAX77620_ONOFFSTAT_EN0 BIT(2)

uint32_t i2c_read(EmuState *state, uint64_t addr) {
  uint32_t base = (addr >= I2C5_BASE) ? I2C5_BASE : I2C1_BASE;
  uint32_t offset = (uint32_t)(addr - base);

  switch (offset) {
  case 0x1C:          // I2C_STATUS
    return 0;         // Transaction complete, no error, not busy
  case 0x8C:          // I2C_CONFIG_LOAD
    return 0;         // MSTR_CONFIG_LOAD (bit 0) cleared = load complete
  case 0x68:          // I2C_INT_STATUS
    return (1 << 11); // BUS_CLEAR_DONE (bit 11)
  case 0x10: {        // I2C_CMD_DATA2 (bytes 4-7 of current chunk)
    if (i2c_slave_addr == 0x4C) {
      if (!rmi4_initialized) rmi4_init();
      return rmi4_read4(rmi4_read_ptr + 4);
    }
    return 0;
  }
  case 0x0C:          // I2C_CMD_DATA1
    // Synaptics RMI4 touch controller (slave 0x4C on I2C1)
    if (i2c_slave_addr == 0x4C) {
      if (!rmi4_initialized) rmi4_init();
      // When hekate reads interrupt status (reg 0x01), update touch state once per poll
      if (rmi4_read_ptr == 0x01 && !rmi4_touch_updated) {
        rmi4_update_touch(state);
        rmi4_touch_updated = true;
      }
      return rmi4_read4(rmi4_read_ptr);
    }
    // Return simulated data based on slave and register
    if (i2c_slave_addr == 0x36) { // MAX17050_ADDR
      switch (i2c_reg_addr) {
      case 0x06:
        return 0x5000; // ~80% battery MAX17050_REP_SOC
      case 0x09:
        return 3950; // 3950 mV MAX17050_VCELL
      default:
        return 0;
      }
    }
    if (i2c_slave_addr == 0x3C) { // MAX77620_ADDR
      if (i2c_reg_addr == 0x15) { // MAX77620_REG_ONOFFSTAT
        uint32_t val = 0;
        if (state->btn_power)
          val |= (1 << 2); // MAX77620_ONOFFSTAT_EN0
        return val;
      }
      return 0x00; // Default PMIC regs
    }
    return 0;
  default:
    return 0;
  }
}

void i2c_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t base = (addr >= I2C5_BASE) ? I2C5_BASE : I2C1_BASE;
  uint32_t offset = (uint32_t)(addr - base);

  switch (offset) {
  case I2C_CMD_ADDR0:
    i2c_slave_addr = (val >> 1) & 0x7F;
    break;
  case I2C_CMD_DATA1:
    i2c_reg_addr = val & 0xFF;
    // Synaptics RMI4 (slave 0x4C): track register pointer for reads
    if (i2c_slave_addr == 0x4C) {
      if (!rmi4_initialized) rmi4_init();
      rmi4_read_ptr = val & 0xFF;
      rmi4_touch_updated = false;
    }
    break;
  }
}

// ==================== Display ====================

uint32_t display_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - DISPLAY_A_BASE);
  if (offset == 0x800 * 4 || offset == 0x2000)
    return (uint32_t)state->fb_addr;
  return 0;
}

void display_write(EmuState *state, uint64_t addr, uint32_t val) {
  if (addr < 0x54200000 || addr >= 0x54240000)
    return;
  uint32_t offset = (uint32_t)(addr - 0x54200000);
  uint32_t index = offset / 4;

  // DC_CMD_DISPLAY_WINDOW_HEADER (offset 0x042*4 = 0x108).
  // Tracks which window's registers are currently selected.
  if (offset == 0x108)
    state->dc_window_sel = val;

  if (offset >= 0x1C00 && offset < 0x1E00) {
    uint32_t win_off = offset - 0x1C00;
    if (win_off == 0x00)
      state->pre_rot = val;
    if (win_off == 0x14) {
      state->pre_w = (val & 0x1FFF);
      state->pre_h = ((val >> 16) & 0x1FFF);
    }
    if (win_off == 0x18)
      state->pre_stride = val & 0xFFFF;
    if (win_off == 0x2C) {
      state->pre_sw = ((val & 0x7) == 2) ? 2 : 0;
      // Tegra DC surface kind: tile in low bits; bits 4–7 = log2(block height in
      // GOBs) for block-linear (values 1..5 → 2,4,8,16,32 GOBs).
      unsigned log2bh = (val >> 4) & 0xF;
      if (log2bh >= 1 && log2bh <= 5)
        state->pre_bh = 1u << log2bh;
      else
        state->pre_bh = 0;
    }
  } else if (offset >= 0x2000 && offset < 0x2100) {
    uint32_t buf_off = offset - 0x2000;
    if (buf_off == 0x00) {
      state->pre_addr = val;
      if (val == 0xF6200000) {
        printf("[display] WARNING: Guest wrote 0xF6200000 to pre_addr!\n");
        // Print the guest PC to see where this comes from
        uint32_t pc = 0;
        uc_reg_read(state->uc, UC_ARM_REG_PC, &pc);
        printf("[display] Guest PC: 0x%X\n", pc);
        uint8_t code[16] = {0};
        if (uc_mem_read(state->uc, pc - 8, code, 16) == UC_ERR_OK) {
          printf("[display] Code: ");
          for (int i=0; i<16; i++) printf("%02X ", code[i]);
          printf("\n");
        }
      }
    }
    if (buf_off == 0x2C) {
      state->pre_sw = ((val & 0x7) == 2) ? 2 : 0;
      unsigned log2bh = (val >> 4) & 0xF;
      if (log2bh >= 1 && log2bh <= 5)
        state->pre_bh = 1u << log2bh;
      else
        state->pre_bh = 0;
    }
  } else if (index == 0x85) { // SIZE legacy
    state->pre_w = (val & 0x1FFF);
    state->pre_h = ((val >> 16) & 0x1FFF);
  } else if (index == 0x41 || offset == 0x104) { // DC_CMD_STATE_CONTROL
    // Latch when activation request bits are set (WIN_A_ACT_REQ=bit1,
    // GENERAL_ACT_REQ=bit0)
    if (val & 0x3) {
      bool is_window_a = (state->dc_window_sel & 0x10) != 0; // Bit 4 = Window A
      bool is_window_d = (state->dc_window_sel & 0x80) != 0; // Bit 7 = Window D

      if (is_window_a && state->pre_addr) {
        // Save Window A parameters as primary display surface.
        state->winA_addr = state->pre_addr;
        state->winA_w = state->pre_w;
        state->winA_h = state->pre_h;
        state->winA_stride = state->pre_stride;
        state->winA_sw = state->pre_sw;
        state->winA_rot = state->pre_rot;
        state->winA_bh = state->pre_bh;
      }

      // Always latch to fb_* for display — but prefer Window A over Window D
      // since Window D is just a transparent overlay in Hekate.
      if (is_window_a || !is_window_d) {
        state->fb_addr = state->pre_addr;
        state->fb_width = state->pre_w;
        state->fb_height = state->pre_h;
        state->fb_stride = state->pre_stride;
        state->fb_swizzle = state->pre_sw;
        state->fb_rotation = state->pre_rot;
        if (state->pre_bh)
          state->fb_bh = state->pre_bh;
      }
      // If Window D is latched alone, don't override — keep Window A's surface.

      state->display_dirty = true;
      printf("[display] LATCH (ACT_REQ): 0x%llX (%dx%d), WinSel: 0x%X, Sw: %d, "
             "Str: %d\n",
             (unsigned long long)state->fb_addr, state->fb_width,
             state->fb_height, state->dc_window_sel, state->fb_swizzle,
             state->fb_stride);
    }
  }
}

// PINMUX read-back cache to satisfy polling
static uint32_t pinmux_reg[0x1000 / 4] = {0};

uint32_t misc_read(EmuState *state, uint64_t addr) {
  if (addr >= PINMUX_BASE && addr < PINMUX_BASE + PINMUX_SIZE) {
    return pinmux_reg[(addr - PINMUX_BASE) / 4];
  }
  // APB_MISC_GP_HIDREV - hardware revision (Erista)
  if (addr == APB_MISC_BASE + 0x804)
    return 0x01; // T210

  // UART
  if (addr >= 0x70006000 && addr < 0x70006500) {
    uint32_t offset = (addr - 0x70006000) % 0x40;
    if (offset == 0x14)
      return 0x60; // LSR: THRE | TMTY always ready
    return 0;
  }
  // DSI
  if (addr >= DSI_BASE && addr < DSI_BASE + DSI_SIZE) {
    return 0;
  }
  // MC
  if (addr >= MC_BASE && addr < MC_BASE + MC_SIZE) {
    uint32_t offset = (uint32_t)(addr - MC_BASE);
    if (offset == 0x65C)
      return 0x40000000; // MC_IRAM_BOM: set to bridge IRAM boundary
    return 0;
  }
  // PWM (Backlight)
  if (addr >= PWM_BASE && addr < PWM_BASE + 0x1000) {
    return 0;
  }
  // TSEC (Tegra Security Co-processor) — base 0x54500000 per Hekate bdk/soc/t210.h.
  // The TSEC microcontroller's firmware blob runs HDCP-based key derivation that
  // produces the per-console TSEC key. We don't emulate the Falcon CPU, so we
  // satisfy the polling protocol Lockpick / Hekate's tsec_query() uses:
  //   * TSEC_DMATRFCMD (0x1118) reads → return DMATRFCMD_IDLE so the DMA-wait exits.
  //   * TSEC_STATUS    (0x1044) reads → return 0xB0B0B0B0 (success magic).
  // The keyslot values that TSEC firmware would have produced are instead
  // pre-loaded into the SE keytable via --prod-keys (slots 12/13/14).
  if (addr >= TSEC_BASE && addr < TSEC_BASE + TSEC_SIZE) {
    uint32_t offset = (uint32_t)(addr - TSEC_BASE);
    if (offset == 0x1118) return (1u << 1);   // TSEC_DMATRFCMD_IDLE
    if (offset == 0x1044) return 0xB0B0B0B0;  // TSEC_STATUS = success magic
    return 0;
  }
  // KFUSE (Key Fuse / HDCP keys) — base 0x7000FC00 per Hekate bdk/soc/t210.h.
  // Hekate's kfuse_wait_ready() spins on KFUSE_STATE bit 16 (DONE) with no
  // timeout (bdk/soc/kfuse.c). Return DONE|CRCPASS so the wait exits and the
  // 144-word KFUSE_KEYS read produces zeros (fine for the boot path; HDCP
  // is unused in the emulator).
  if (addr >= 0x7000FC00 && addr < 0x7000FD00) {
    uint32_t offset = (uint32_t)(addr - 0x7000FC00);
    if (offset == 0x80) // KFUSE_STATE
      return (1u << 16) | (1u << 17); // DONE | CRCPASS
    if (offset == 0x8C) // KFUSE_KEYS (auto-incrementing zeros)
      return 0;
    return 0;
  }
  // SDMMC Controllers
  if ((addr >= SDMMC1_BASE && addr < SDMMC1_BASE + 0x1000) ||
      (addr >= SDMMC4_BASE && addr < SDMMC4_BASE + 0x1000)) {
    uint32_t base = (addr >= SDMMC4_BASE) ? SDMMC4_BASE : SDMMC1_BASE;
    uint32_t offset = addr - base;

    uint32_t *rsp =
        (base == SDMMC4_BASE) ? state->sdmmc4_rsp : state->sdmmc_rsp;
    uint32_t &norintsts = (base == SDMMC4_BASE) ? state->sdmmc4_norintsts
                                                : state->sdmmc_norintsts;
    uint16_t blksize =
        (base == SDMMC4_BASE) ? state->sdmmc4_blksize : state->sdmmc_blksize;
    uint16_t blkcnt =
        (base == SDMMC4_BASE) ? state->sdmmc4_blkcnt : state->sdmmc_blkcnt;
    uint16_t trnmod =
        (base == SDMMC4_BASE) ? state->sdmmc4_trnmod : state->sdmmc_trnmod;

    uint32_t result = 0;
    if (offset == 0x00)
      result = (base == SDMMC4_BASE) ? state->sdmmc4_sysad : state->sdmmc_sysad;
    else if (offset == 0x04)
      result = (blkcnt << 16) | blksize;
    else if (offset == 0x0C)
      result = trnmod;
    else if (offset == 0x58)
      result = (uint32_t)((base == SDMMC4_BASE) ? state->sdmmc4_adma_addr
                                                : state->sdmmc_adma_addr);
    else if (offset == 0x5C)
      result = (uint32_t)(((base == SDMMC4_BASE) ? state->sdmmc4_adma_addr
                                                 : state->sdmmc_adma_addr) >>
                          32);
    else if (offset == 0x24)
      result = 0x01F70000; // CARD_PRESENT | CD_STABLE | CD_LVL | DAT_LINE_LEVEL
                           // (Ready)
    else if (offset == 0x2C)
      result = 0x0003; // SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_INT_STABLE
    else if (offset == 0x40)
      result =
          0x376CD08C; // Full Tegra capabilities (64-bit, SDMA, ADMA2, etc.)
    else if (offset == 0x44)
      result = 0x10002F73; // CAP1
    else if (offset == 0x30)
      result = ((base == SDMMC4_BASE ? state->sdmmc4_errintsts
                                     : state->sdmmc_errintsts)
                << 16) |
               norintsts;
    else if (offset == 0x32)
      result = (base == SDMMC4_BASE) ? state->sdmmc4_errintsts
                                     : state->sdmmc_errintsts;
    else if (offset >= 0x10 && offset <= 0x1C)
      result = rsp[(offset - 0x10) / 4];
    else if (offset == 0x1EC)
      result = 0x00000001; // AUTOCAL_STS
    else if (offset == 0xFE)
      result = 0x0303; // SDHCI Version 4.0

    printf("[sdmmc%c] R: 0x%02lX = 0x%08X (PC=0x%llX)\n",
           (base == SDMMC4_BASE) ? '4' : '1', (unsigned long)offset,
           result, (unsigned long long)state->insn_count);
    return result;
  }
  return 0;
}

void misc_write(uc_engine *uc, EmuState *state, uint64_t addr, int64_t value,
                int size) {
  uint32_t val = (uint32_t)value;
  // SDMMC Controllers
  if ((addr >= SDMMC1_BASE && addr < SDMMC1_BASE + 0x1000) ||
      (addr >= SDMMC4_BASE && addr < SDMMC4_BASE + 0x1000)) {
    uint32_t base = (addr >= SDMMC4_BASE) ? SDMMC4_BASE : SDMMC1_BASE;
    uint32_t offset = addr - base;

    printf("[sdmmc%c] W: 0x%02X = 0x%08X (size %d)\n",
           (base == SDMMC4_BASE) ? '4' : '1', offset, val, size);

    uint32_t &arg =
        (base == SDMMC4_BASE) ? state->sdmmc4_arg : state->sdmmc_arg;
    uint32_t *rsp =
        (base == SDMMC4_BASE) ? state->sdmmc4_rsp : state->sdmmc_rsp;
    uint32_t &norintsts = (base == SDMMC4_BASE) ? state->sdmmc4_norintsts
                                                : state->sdmmc_norintsts;
    uint32_t &errintsts = (base == SDMMC4_BASE) ? state->sdmmc4_errintsts
                                                : state->sdmmc_errintsts;
    uint32_t &sysad =
        (base == SDMMC4_BASE) ? state->sdmmc4_sysad : state->sdmmc_sysad;
    uint8_t &hostctl =
        (base == SDMMC4_BASE) ? state->sdmmc4_hostctl : state->sdmmc_hostctl;
    uint16_t &blksize =
        (base == SDMMC4_BASE) ? state->sdmmc4_blksize : state->sdmmc_blksize;
    uint16_t &blkcnt =
        (base == SDMMC4_BASE) ? state->sdmmc4_blkcnt : state->sdmmc_blkcnt;
    uint16_t &trnmod =
        (base == SDMMC4_BASE) ? state->sdmmc4_trnmod : state->sdmmc_trnmod;
    uint64_t &adma_addr = (base == SDMMC4_BASE) ? state->sdmmc4_adma_addr
                                                : state->sdmmc_adma_addr;

    if (offset == 0x00)
      sysad = val;
    if (offset == 0x28)
      hostctl = val & 0x1E;
    if (offset == 0x2F) {
      // Software Reset. Clear immediately to signify completion.
      mmio_regs[base + 0x2C] &= ~(val << 24);
    }
    if (offset == 0x2C && size == 4) {
      // If reset bits were set in 4-byte write, clear them for subsequent reads
      mmio_regs[base + 0x2C] &= ~0x07000000;
    }
    if (offset == 0x04) {
      if (size == 4) {
        blksize = val & 0x0FFF;
        blkcnt = val >> 16;
      } else
        blksize = val & 0x0FFF;
    }

    if (offset == 0x06)
      blkcnt = val;
    if (offset == 0x08)
      arg = val;
    if (offset == 0x0C)
      trnmod = (size == 4) ? (val & 0xFFFF) : val;

    if (offset == 0x58) {
      if (size == 8)
        adma_addr = value;
      else
        adma_addr = (adma_addr & 0xFFFFFFFF00000000ULL) | val;
    }
    if (offset == 0x5C)
      adma_addr = (adma_addr & 0x00000000FFFFFFFFULL) | ((uint64_t)val << 32);
    if (offset == 0x30) {
      norintsts &= ~(val & 0xFFFF);
      errintsts &= ~(val >> 16);
    }
    if (offset == 0x32)
      errintsts &= ~val;

    if (offset == 0x0E || (offset == 0x0C && size == 4)) {
      uint32_t cmdreg = (offset == 0x0C) ? (val >> 16) : val;
      uint32_t cmd = (cmdreg >> 8) & 0x3F;
      bool is_read = (trnmod & 0x0010); // Bit 4 of TRNMOD is Read/Write

      if (base == SDMMC1_BASE) {
        printf("[sdmmc] W: offset 0x%X = 0x%08X (PC=0x%08llX)\n", offset, val,
               (unsigned long long)state->insn_count);
        if (offset == 0x0E || (offset == 0x0C && size == 4)) {
          printf("[sdmmc] CMD%d: arg=0x%08X, blkcnt=%d, trnmod=0x%04X\n", cmd,
                 arg, blkcnt, trnmod);
        }
        fflush(stdout);
      }

      uint32_t r1_base = 0x00000000;
      if (base == SDMMC1_BASE && state->last_cmd_was_55)
        r1_base |= 0x0100;
      if (base == SDMMC4_BASE && state->last_cmd4_was_55)
        r1_base |= 0x0100;

      // Set Command Complete for all commands
      norintsts |= 0x0001;

      switch (cmd) {
      case 0:
        break;
      case 8:
        rsp[0] = (base == SDMMC4_BASE) ? 0x00000900 : arg;
        // eMMC SEND_EXT_CSD: 512-byte data transfer to DMA dst. Without this,
        // sdmmc_storage_init_mmc times out waiting for transfer complete and
        // bails with storage->initialized = 0 — which silently breaks all
        // later sdmmc_storage_read calls (returns 0 without issuing CMD18).
        if (base == SDMMC4_BASE) {
          uint8_t ext_csd[512] = {0};
          // Set a few fields BDK actually parses (most others can be 0):
          //   EXT_CSD_REV = 192 (offset 192)
          //   EXT_CSD_CARD_TYPE = 196 (HS-52 + HS200 + HS400 supported)
          //   EXT_CSD_SEC_CNT = 212..215 (sector count, little-endian u32)
          ext_csd[192] = 7;  // eMMC v5.0
          ext_csd[196] = 0x57; // HS400_1.8V | HS200_1.8V | HS_52 | HS_DDR
          // 32GB worth of sectors (0x3A380000 = 977MB; for the user's actual
          // dump size we'd want the real value, but BDK only uses sec_cnt for
          // sanity, not for read addressing).
          uint32_t sec_cnt = 64 * 1024 * 1024; // 32GB / 512B = 64M sectors
          ext_csd[212] = sec_cnt & 0xFF;
          ext_csd[213] = (sec_cnt >> 8) & 0xFF;
          ext_csd[214] = (sec_cnt >> 16) & 0xFF;
          ext_csd[215] = (sec_cnt >> 24) & 0xFF;

          uint64_t dma_addr = 0;
          if (trnmod & 0x0001) {            // DMA enabled
            if ((hostctl & 0x18) == 0x10) { // ADMA2
              uint8_t desc[12];
              if (uc_mem_read(uc, adma_addr, desc, 12) == UC_ERR_OK) {
                uint32_t low = *(uint32_t *)(desc + 4);
                uint32_t high = *(uint32_t *)(desc + 8);
                dma_addr = ((uint64_t)high << 32) | low;
              }
            } else { // SDMA
              dma_addr = sysad;
            }
            if (dma_addr) uc_mem_write(uc, dma_addr, ext_csd, 512);
          } else {
            uc_mem_write(uc, sysad, ext_csd, 512);
          }
          norintsts |= 0x0002; // TRANSFER_COMPLETE
        }
        break;
      case 1:
        rsp[0] = 0xC0FF8000;
        break;
      case 2: // SEND_CID
        rsp[0] = 0x12345678;
        rsp[1] = 0xABCDEF01;
        rsp[2] = 0x23456789;
        rsp[3] = 0xBCDEF012;
        break;
      case 3: // SEND_RELATIVE_ADDR
        if (base == SDMMC1_BASE) {
          // SD: R6 response — RCA in upper 16 bits, status in lower 16.
          rsp[0] = 0x00010000 | (3 << 9); // RCA=1, State=stby(3)
        } else {
          // eMMC: R1 response — status only. RCA was set by host via the arg.
          // Bit 16 is R1_CID_CSD_OVERWRITE (error) — must NOT be set, or
          // Hekate's _sdmmc_storage_check_card_status() rejects the response
          // and sdmmc_storage_init_mmc bails before CMD9 (SEND_CSD).
          rsp[0] = (3 << 9); // State=stby(3) only
        }
        break;
      case 9: // SEND_CSD
        // R2 response. Hekate's sdmmc_get_rsp shifts each rspreg left by 8
        // (to account for the 7-bit CRC strip + start bit), so my rsp[3] here
        // ultimately becomes rsp[0] (the top dword of the 128-bit CSD) at
        // unstuff_bits time, but with bits remapped: original bit N of rspreg3
        // ends up at bit N+8 of unstuff_bits' logical CSD.
        // mmca_vsn is at CSD bits 122-125. After Hekate's shuffle, that's
        // bits 18-21 of my rspreg3. Set those to 0100 (=4) so storage->csd.
        // mmca_vsn >= CSD_SPEC_VER_4 and sdmmc_storage_init_mmc reaches
        // storage->initialized = 1 instead of the early-return at line 688.
        rsp[0] = 0x400E0032;
        rsp[1] = 0x5B590000;
        rsp[2] = 0x00007F80;
        rsp[3] = 0x16504000; // bits 18-21 = 0100 → mmca_vsn = 4
        break;
      case 13: { // SEND_STATUS / ACMD13 (SD_STATUS)
        bool is_acmd = (base == SDMMC1_BASE) ? state->last_cmd_was_55
                                             : state->last_cmd4_was_55;
        if (is_acmd) {
          uint8_t ss[64] = {0};
          ss[0] = 0x80; // 4-bit support (bit 511:510 = 10)
          uint64_t dma_addr = 0;
          if (trnmod & 0x0001) {
            if ((hostctl & 0x18) == 0x10) { // ADMA2
              uint8_t desc[12];
              if (uc_mem_read(uc, adma_addr, desc, 12) == UC_ERR_OK) {
                uint32_t low = *(uint32_t *)(desc + 4);
                uint32_t high = *(uint32_t *)(desc + 8);
                dma_addr = ((uint64_t)high << 32) | low;
              }
            } else { // SDMA
              dma_addr = sysad;
            }
            if (dma_addr)
              uc_mem_write(uc, dma_addr, ss, 64);
          } else {
            uc_mem_write(uc, sysad, ss, 64);
          }
          norintsts |= 0x0002;
        }
        rsp[0] = r1_base | (4 << 9); // TRAN state
        break;
      }
      case 7:
        rsp[0] = r1_base | (3 << 9);
        break; // STBY state (moving to tran)
      case 10:
        rsp[0] = r1_base | (4 << 9);
        break;
      case 12:
        rsp[0] = r1_base | (4 << 9);
        break;
      case 16:
        rsp[0] = r1_base | (4 << 9);
        break;
      case 23:
        rsp[0] = r1_base | (4 << 9);
        break;
      case 6: { // SWITCH_FUNC or ACMD6 (SET_BUS_WIDTH)
        bool is_acmd = (base == SDMMC1_BASE) ? state->last_cmd_was_55
                                             : state->last_cmd4_was_55;
        if (is_acmd) {
          // ACMD6: SET_BUS_WIDTH
          rsp[0] = r1_base | (4 << 9);
        } else if (base == SDMMC1_BASE) {
          // CMD6: SWITCH_FUNC (SD)
          uint8_t status[64] = {0};
          status[12] = 0x00;
          status[13] = 0x02; // HS Support
          status[16] = 0x01; // Group 1 switched to HS

          uint64_t dma_addr = 0;
          if (trnmod & 0x0001) {            // DMA Enabled
            if ((hostctl & 0x18) == 0x10) { // ADMA2
              uint8_t desc[12];
              if (uc_mem_read(uc, adma_addr, desc, 12) == UC_ERR_OK) {
                uint32_t low = *(uint32_t *)(desc + 4);
                uint32_t high = *(uint32_t *)(desc + 8);
                dma_addr = ((uint64_t)high << 32) | low;
              }
            } else { // SDMA
              dma_addr = sysad;
            }
            if (dma_addr)
              uc_mem_write(uc, dma_addr, status, 64);
          } else { // PIO
            uc_mem_write(uc, sysad, status, 64);
          }
          norintsts |= 0x0002; // Transfer Complete
          rsp[0] = r1_base | (4 << 9);
        } else if (base == SDMMC4_BASE) {
          // CMD6: SWITCH (eMMC)
          uint8_t index = (arg >> 16) & 0xFF;
          uint8_t val = (arg >> 8) & 0xFF;
          if (index == 179) { // PARTITION_CONFIG
            state->emmc_partition = val & 0x7;
            printf("[sdmmc] eMMC Partition Switch: %u\n",
                   state->emmc_partition);
          }
          rsp[0] = r1_base | (4 << 9);
        }
        break;
      }
      case 55: {
        rsp[0] = 0x00000920;
        if (base == SDMMC1_BASE)
          state->last_cmd_was_55 = true;
        else
          state->last_cmd4_was_55 = true;
        break;
      }
      case 41:
        rsp[0] = 0xC0FF8000;
        break;
      case 42:
        rsp[0] = r1_base | (4 << 9);
        break;   // ACMD42
      case 51: { // SEND_SCR (ACMD)
        bool is_acmd = (base == SDMMC1_BASE) ? state->last_cmd_was_55
                                             : state->last_cmd4_was_55;
        if (is_acmd) {
          uint8_t scr[8] = {0x02, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
          uint64_t dma_addr = 0;
          if (trnmod & 0x0001) {
            if ((hostctl & 0x18) == 0x10) { // ADMA2
              uint8_t desc[12];
              if (uc_mem_read(uc, adma_addr, desc, 12) == UC_ERR_OK) {
                uint32_t low = *(uint32_t *)(desc + 4);
                uint32_t high = *(uint32_t *)(desc + 8);
                dma_addr = ((uint64_t)high << 32) | low;
              }
            } else { // SDMA
              dma_addr = sysad;
            }
            if (dma_addr)
              uc_mem_write(uc, dma_addr, scr, 8);
          } else {
            uc_mem_write(uc, sysad, scr, 8);
          }
          norintsts |= 0x0002;
        } else {
          printf("[sdmmc] ERROR: Storage command %d on base 0x%llX but fd is "
                 "-1. (Missing --sd or --boot0?)\n",
                 cmd, (unsigned long long)base);
        }
        rsp[0] = r1_base | (4 << 9);
        break;
      }
      case 17:
      case 18:
      case 24:
      case 25: {
        int fd = -1;
        if (base == SDMMC1_BASE)
          fd = state->sd_fd;
        else {
          // eMMC Support
          if (state->emmc_partition == 1)
            fd = state->emmc_boot0_fd;
          else if (state->emmc_partition == 2)
            fd = state->emmc_boot1_fd;
          else
            fd = -2; // GPP (handled below)
        }

        if (fd != -1) {
          uint64_t sector = arg;
          uint64_t file_off = sector * 512;
          uint16_t bcnt =
              (blkcnt == 0 && (cmd == 18 || cmd == 25)) ? 1 : blkcnt;
          if (bcnt == 0)
            bcnt = 1;
          if (cmd == 17 || cmd == 18) {
            printf("[sdmmc] %s READ CMD%d: Sector = %llu, Count = %u (part=%u)\n",
                   (base == SDMMC1_BASE) ? "SD" : "eMMC", cmd,
                   (unsigned long long)sector, bcnt,
                   (base == SDMMC1_BASE) ? 0 : state->emmc_partition);
          }
          size_t xfer_len = bcnt * 512;

          auto do_io = [&](int current_fd, uint64_t current_off,
                           uint64_t dma_addr, size_t len) {
            if (current_fd == -2) { // GPP Spanning
              // Hekate-style rawnand splitting can use 2GB or 4GB chunks.
              // Stat the first part once and cache; assumes all but the last
              // part are the same size (true for both Hekate's standard
              // 4GB-FAT32-friendly splits and tools that use 2GB chunks).
              static size_t part_size = 0;
              if (!part_size && !state->emmc_gpp_fds.empty()) {
                struct stat st;
                if (fstat(state->emmc_gpp_fds[0], &st) == 0 && st.st_size > 0)
                  part_size = (size_t)st.st_size;
                else
                  part_size = 4ULL * 1024 * 1024 * 1024;
                printf("[sdmmc] GPP part size detected: %zu bytes\n", part_size);
              }
              if (!part_size) return; // No GPP files — nothing to read.
              int part_idx = (int)(current_off / part_size);
              uint64_t part_off = current_off % part_size;
              if (part_idx < (int)state->emmc_gpp_fds.size()) {
                int real_fd = state->emmc_gpp_fds[part_idx];
                std::vector<uint8_t> io_buf(len);
                if (is_read) {
                  ssize_t res = pread(real_fd, io_buf.data(), len, part_off);
                  if (res != (ssize_t)len)
                    printf("[sdmmc] eMMC GPP READ ERROR: res=%zd, "
                           "expected=%zu, off=0x%llX\n",
                           res, len, (unsigned long long)part_off);
                  uc_mem_write(uc, dma_addr, io_buf.data(), len);
                } else {
                  uc_mem_read(uc, dma_addr, io_buf.data(), len);
                  ssize_t res = pwrite(real_fd, io_buf.data(), len, part_off);
                  if (res != (ssize_t)len)
                    printf("[sdmmc] eMMC GPP WRITE ERROR: res=%zd, "
                           "expected=%zu, off=0x%llX\n",
                           res, len, (unsigned long long)part_off);
                }
              }
            } else if (current_fd >= 0) {
              std::vector<uint8_t> io_buf(len);
              if (is_read) {
                ssize_t res =
                    pread(current_fd, io_buf.data(), len, current_off);
                if (res != (ssize_t)len)
                  printf("[sdmmc] SD/BOOT READ ERROR: res=%zd, expected=%zu, "
                         "off=0x%llX\n",
                         res, len, (unsigned long long)current_off);
                uc_mem_write(uc, dma_addr, io_buf.data(), len);
              } else {
                uc_mem_read(uc, dma_addr, io_buf.data(), len);
                ssize_t res =
                    pwrite(current_fd, io_buf.data(), len, current_off);
                if (res != (ssize_t)len)
                  printf("[sdmmc] SD/BOOT WRITE ERROR: res=%zd, expected=%zu, "
                         "off=0x%llX\n",
                         res, len, (unsigned long long)current_off);
              }
            }
          };

          if (trnmod & 0x0001) {            // DMA Enabled (Bit 0 of TRNMOD)
            if ((hostctl & 0x18) == 0x10) { // ADMA2
              // Parse ADMA2 descriptors
              uint64_t desc_addr = adma_addr;
              uint8_t desc[12];
              while (true) {
                if (uc_mem_read(uc, desc_addr, desc, 12) != UC_ERR_OK)
                  break;
                uint16_t attr = desc[0];
                uint16_t len = *(uint16_t *)(desc + 2);
                uint32_t low = *(uint32_t *)(desc + 4);
                uint32_t high = *(uint32_t *)(desc + 8);
                uint64_t dma_addr = ((uint64_t)high << 32) | low;
                if ((attr & 0x3) == 0x2) { // Action: Transfer
                  do_io(fd, file_off, dma_addr, len);
                  file_off += len;
                }
                if (attr & 0x02)
                  break; // End bit is bit 1
                desc_addr += 12;
              }
            } else { // SDMA
              do_io(fd, file_off, adma_addr, xfer_len);
            }
          } else { // PIO
            do_io(fd, file_off, sysad, xfer_len);
          }
        }
        norintsts |= 0x0002; // Transfer Complete
        break;
      }
      }
      norintsts |= 0x0001; // Command Complete

      // Reset ACMD flag if not CMD55
      if (cmd != 55) {
        if (base == SDMMC1_BASE)
          state->last_cmd_was_55 = false;
        else
          state->last_cmd4_was_55 = false;
      }
    }
  }
}

// ==================== PMC ====================

uint32_t rtc_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - RTC_BASE);
  uint64_t ms = state->emu_usec / 1000;

  switch (offset) {
  case 0x0C:
    return (uint32_t)(ms / 1000); // APBDEV_RTC_SHADOW_SECONDS
  case 0x10:
    return (uint32_t)(ms % 1000); // APBDEV_RTC_MILLI_SECONDS
  default:
    return 0;
  }
}

void rtc_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  (void)addr;
  (void)val;
}

uint32_t pmc_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - PMC_BASE);

  switch (offset) {
  case 0x50:
    return pmc_scratch0; // APBDEV_PMC_SCRATCH0
  case 0x1A0:
    return 0; // PMC_PWR_DET - all rails OK
  default:
    return 0;
  }
}

// ==================== PMC ====================

void pmc_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - PMC_BASE);
  switch (offset) {
  case 0x50:
    pmc_scratch0 = val;
    break;
  case 0x120:
    pmc_scratch37 = val;
    break;
  }
}

// ==================== Flow Controller ====================

static void flow_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - 0x60007000);
  if (offset == 0x04) {
    // HALT_COP_EVENTS. Bits 31:29 select HALT_MODE; val 0x50000000 is
    // WAITEVENT | JTAG, written by bpmp_halt_cop() on power-off, reset and
    // fatal paths. The Hekate caller typically follows it with a tight
    // `while(true);`, so we see one halt write then the CPU spins forever.
    // Treat it as "the payload is done" and exit cleanly so the SDL window
    // closes (this is what "Power off" / "Reboot" expect).
    printf("[flow] BPMP HALT/WaitEvent (val=0x%08X), shutting down emulator\n",
           val);
    state->running = false;
    uc_emu_stop(state->uc);
  } else {
    printf("[flow] W: 0x%02X = 0x%08X\n", offset, val);
  }
}

// ==================== Clock/Reset ====================

uint32_t clk_rst_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - CLK_RST_BASE);
  // PLL_BASE registers (per Hekate bdk/soc/clock.h): each PLL has an _BASE
  // register where bit 30 = ENABLE and bit 27 = LOCK. After enabling a PLL
  // the boot code polls bit 27 until set. Real silicon locks within ~1ms; we
  // simulate "always locked + always enabled" so the polls return immediately.
  // Offsets: PLLC=0x80, PLLM=0x90, PLLP=0xA0, PLLA=0xB0, PLLU=0xC0, PLLD=0xD0,
  //          PLLX=0xE0, PLLE=0xE8, PLLD2=0x4B8, PLLREFE=0x4C4.
  switch (offset) {
  case 0x80: case 0x90: case 0xA0: case 0xB0:
  case 0xC0: case 0xD0: case 0xE0: case 0xE8:
  case 0x4B8: case 0x4C4:
    return (1u << 30) | (1u << 27); // ENABLE | LOCK
  }
  if (offset == 0x50)
    return (4 << 28); // 38.4 MHz (OSC_FREQ field in OSC_CTRL)
  if (offset == 0x10)
    return (1 << 14); // CLK_ENB_SDMMC1
  if (offset == 0x18)
    return (1 << 15); // CLK_ENB_SDMMC4
  if (offset == 0x04)
    return 0; // RST_DEVICES_L (none in reset)
  if (offset == 0x0C)
    return 0; // RST_DEVICES_H (none in reset)
  (void)state;
  return 0;
}

void clk_rst_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  (void)addr;
  (void)val;
}

// ==================== Fuse ====================

uint32_t fuse_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - FUSE_BASE);
  switch (offset) {
  case 0x100:
    return 0x83;
  case 0x110:
    return 0x07;
  case 0x1A0:
    return 0x06;
  case 0x148:
    return 0x83000001;
  case 0x118:
    return 1785;
  default:
    return 0;
  }
}

void fuse_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  (void)addr;
  (void)val;
}

// ==================== SE (Security Engine) ====================
// Real AES-128 implementation lives in t210/se_engine.cpp. The functions
// below are thin shims so the existing dispatcher routing keeps working.

static uint32_t se_read(EmuState *state, uint64_t addr) {
  return se_engine_read(state, addr);
}
static void se_write(EmuState *state, uint64_t addr, uint32_t val) {
  se_engine_write(state, addr, val);
}

// ==================== MMIO Hook Callbacks ====================

static uint64_t vic_src_addr = 0;
static uint64_t vic_dst_addr = 0;

static uint32_t vic_read(EmuState *state, uint64_t addr) {
  (void)state;
  (void)addr;
  return 0;
}

// VIC registers using Falcon PA translated offsets (1000 + untranslated >> 6)
// VIC registers using Falcon PA translated offsets (1000 + untranslated >> 6)
static uint64_t vic_config_addr = 0;
static uint32_t vic_config_size = 0;
static uint32_t vic_last_falcon_addr = 0;

struct vic_out_config_struct {
  uint32_t raw[2];
};

struct vic_out_sfc_config_struct {
  uint32_t raw[2];
};

static void vic_write_internal(EmuState *state, uint32_t translated_off,
                               uint32_t val);

static void vic_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - VIC_BASE);

  if (offset == 0x10AC) { // PVIC_FALCON_ADDR
    vic_last_falcon_addr = val;
  } else if (offset == 0x10B0) { // PVIC_FALCON_DATA
    uint32_t translated_off = (vic_last_falcon_addr >> 6) + 0x1000;
    vic_write_internal(state, translated_off, val);
  } else {
    vic_write_internal(state, offset, val);
  }
}

static void vic_write_internal(EmuState *state, uint32_t offset, uint32_t val) {

  if (offset == 0x1500) { // VIC_SC_PRAMBASE
    vic_config_addr = (uint64_t)val << 8;
  } else if (offset == 0x1504) { // VIC_SC_PRAMSIZE
    vic_config_size = val << 6;
  } else if (offset == 0x150C) { // VIC_SC_SFC0_BASE_LUMA
    vic_src_addr = (uint64_t)val << 8;
  } else if (offset == 0x1880) { // VIC_BL_TARGET_BASADR
    vic_dst_addr = (uint64_t)val << 8;
  } else if ((offset == 0x1400 || offset == 0x1440) &&
             val == 1) { // VIC_FC_COMPOSE
    if (vic_src_addr >= 0x40000000 && vic_dst_addr >= 0x40000000) {
      uint32_t sfc[2] = {0};
      uc_mem_read(state->uc, vic_config_addr + 0x18, sfc, 8);

      uint32_t dw = (sfc[1] & 0x3FFF), dh = ((sfc[1] >> 14) & 0x3FFF);
      bool fb_fb = false;
      if (dw < 16 || dh < 16 || dw > 2048 || dh > 2048) {
        dw = state->fb_width;
        dh = state->fb_height;
        fb_fb = true;
        if (dw < 16)
          dw = 720;
        if (dh < 16)
          dh = 1280;
      }

      printf("[vic] Compose: 0x%llX -> 0x%llX (%dx%d) %s\n",
             (unsigned long long)vic_src_addr, (unsigned long long)vic_dst_addr,
             dw, dh, fb_fb ? "(FALLBACK)" : "");

      // Hekate configures VIC with SlotBlkKind=PITCH, OutBlkKind=PITCH.
      // This is a straight pitch-to-pitch copy (possibly with rotation/flip
      // handled by the real VIC FCE microcode, but we just copy here).
      size_t sz = (size_t)dw * dh * 4;
      if (sz > 32 * 1024 * 1024)
        return;
      std::vector<uint8_t> buf(sz);
      if (uc_mem_read(state->uc, vic_src_addr, buf.data(), sz) == UC_ERR_OK) {
        uc_mem_write(state->uc, vic_dst_addr, buf.data(), sz);


        state->display_dirty = true;
      }
    }
  }
  printf("[vic] W: 0x%08X = 0x%08X\n", offset, val);
}

static uint32_t i2s_read(EmuState *state, uint64_t addr) {
  (void)state;
  (void)addr;
  return 0;
}

static void i2s_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  (void)addr;
  (void)val;
}

static uint32_t sysreg_read(EmuState *state, uint64_t addr) {
  (void)state;
  (void)addr;
  return 0;
}

static void sysreg_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - SYSREG_BASE);
  (void)state;
  printf("[sysreg] W: 0x%08X = 0x%08X\n", offset, val);
}

// ==================== BPMP Cache ====================

static uint32_t bpmp_cache_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - BPMP_CACHE_BASE);
  (void)state;

  switch (offset) {
  case 0x00:
    return 1; // BPMP_CACHE_CONFIG - return Cache Enabled
  case 0x48:
    return 1; // BPMP_CACHE_INT_RAW_EVENT - return MAINT_DONE
  default:
    return 0;
  }
}

static void bpmp_cache_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  (void)addr;
  (void)val;
}

static void hook_mmio_read(uc_engine *uc, uc_mem_type type, uint64_t address,
                           int size, int64_t value, void *user_data) {
  EmuState *state = (EmuState *)user_data;
  uint32_t result = 0;

  if (address >= TMR_BASE && address < TMR_BASE + TMR_SIZE) {
    uint32_t offset = (uint32_t)(address - TMR_BASE);
    if (offset == 0x10)
      result = (uint32_t)state->emu_usec; // TIMERUS_CNTR_1US
    else
      result = 0;
  } else {
    uint32_t pc;
    uc_reg_read(uc, UC_ARM_REG_PC, &pc);

    if (mmio_regs.count(address)) {
      result = mmio_regs[address];
    }

    if (address >= GPIO_BASE && address < GPIO_BASE + GPIO_SIZE) {
      result = gpio_read(state, address);
    } else if (address >= VIC_BASE && address < VIC_BASE + VIC_SIZE) {
      result = vic_read(state, address);
    } else if (address >= SYSREG_BASE && address < SYSREG_BASE + SYSREG_SIZE) {
      result = sysreg_read(state, address);
    } else if (address >= BPMP_CACHE_BASE &&
               address < BPMP_CACHE_BASE + BPMP_CACHE_SIZE) {
      result = bpmp_cache_read(state, address);
    } else if (address >= I2S_BASE && address < I2S_BASE + I2S_SIZE) {
      result = i2s_read(state, address);
    } else if (address >= RTC_BASE && address < RTC_BASE + RTC_SIZE) {
      result = rtc_read(state, address);
    } else if (address >= I2C3_BASE && address < I2C3_BASE + I2C3_SIZE) {
      // I²C3 / STMFTS touchscreen (slave 0x49). Must be tested before the I²C1
      // branch, which would otherwise swallow the I²C3 page.
      result = i2c3_read(state, address);
    } else if (address >= I2C1_BASE && address < I2C1_BASE + 0x100) {
      result = i2c_read(state, address);
    } else if (address >= I2C5_BASE && address < I2C5_BASE + I2C_SIZE) {
      result = i2c_read(state, address);
    } else if (address >= DISPLAY_A_BASE &&
               address < DISPLAY_A_BASE + DISPLAY_SIZE) {
      result = display_read(state, address);
    } else if (address >= PMC_BASE && address < PMC_BASE + PMC_SIZE) {
      result = pmc_read(state, address);
    } else if (address >= CLK_RST_BASE &&
               address < CLK_RST_BASE + CLK_RST_SIZE) {
      result = clk_rst_read(state, address);
    } else if (address >= FUSE_BASE && address < FUSE_BASE + FUSE_SIZE) {
      result = fuse_read(state, address);
    } else if (address >= SE_BASE && address < SE_BASE + SE_SIZE) {
      result = se_read(state, address);
    } else {
      result = misc_read(state, address);
    }
    // printf("[mmio] R: 0x%08llX = 0x%08X (PC=0x%08X)\n", (unsigned long
    // long)address, result, pc);
  }

  uc_mem_write(uc, address, &result, size);
}

static void hook_mmio_write(uc_engine *uc, uc_mem_type type, uint64_t address,
                            int size, int64_t value, void *user_data) {
  EmuState *state = (EmuState *)user_data;
  uint32_t val = (uint32_t)value;
  uint32_t pc;
  uc_reg_read(uc, UC_ARM_REG_PC, &pc);
  // printf("[mmio] W: 0x%08llX = 0x%08X (PC=0x%08X)\n", (unsigned long
  // long)address, val, pc); fflush(stdout);

  if (address >= IRAM_BASE && address < IRAM_BASE + 0x40000) {
    // Log IRAM writes if debugging
  } else {
    mmio_regs[address] = val; // Persistent storage

    if (address >= GPIO_BASE && address < GPIO_BASE + GPIO_SIZE) {
      gpio_write(state, address, val);
    } else if (address >= SDMMC1_BASE && address < SDMMC1_BASE + 0x200) {
      misc_write(uc, state, address, value, size);
    } else if (address >= SDMMC4_BASE && address < SDMMC4_BASE + 0x200) {
      misc_write(uc, state, address, value, size);
    } else if (address >= 0x70006000 && address < 0x70006500) {
      uint32_t offset = (address - 0x70006000) % 0x40;
      if (offset == 0) {
        // UART_THR. Hekate's UART carries Joy-Con HID protocol bytes (binary)
        // in addition to occasional debug text. We buffer printable ASCII into
        // a per-line buffer and flush on \n / \r / overflow so that a
        // gfx_putc-mirrored println shows up as a single [uart] line.
        static char uart_line[512];
        static size_t uart_len = 0;
        auto flush_uart_line = []() {
          if (uart_len > 0) {
            uart_line[uart_len] = 0;
            printf("[uart] %s\n", uart_line);
            fflush(stdout);
            uart_len = 0;
          }
        };
        uint8_t b = (uint8_t)val;
        if (b == '\n' || b == '\r') {
          flush_uart_line();
        } else if (b >= 0x20 && b < 0x7F) {
          if (uart_len + 1 >= sizeof(uart_line)) flush_uart_line();
          uart_line[uart_len++] = (char)b;
        }
        // Non-printables (Joy-Con HID etc.) are dropped silently.
      }
    } else if (address >= VIC_BASE && address < VIC_BASE + VIC_SIZE) {
      vic_write(state, address, val);
    } else if (address >= SYSREG_BASE && address < SYSREG_BASE + SYSREG_SIZE) {
      sysreg_write(state, address, val);
    } else if (address >= BPMP_CACHE_BASE &&
               address < BPMP_CACHE_BASE + BPMP_CACHE_SIZE) {
      bpmp_cache_write(state, address, val);
    } else if (address >= I2S_BASE && address < I2S_BASE + I2S_SIZE) {
      i2s_write(state, address, val);
    } else if (address >= RTC_BASE && address < RTC_BASE + RTC_SIZE) {
      rtc_write(state, address, val);
    } else if (address >= I2C3_BASE && address < I2C3_BASE + I2C3_SIZE) {
      i2c3_write(state, address, val);
    } else if (address >= I2C1_BASE && address < I2C1_BASE + 0x100) {
      i2c_write(state, address, val);
    } else if (address >= I2C5_BASE && address < I2C5_BASE + I2C_SIZE) {
      i2c_write(state, address, val);
    } else if (address >= DISPLAY_A_BASE &&
               address < DISPLAY_A_BASE + DISPLAY_SIZE) {
      display_write(state, address, val);
    } else if (address >= PMC_BASE && address < PMC_BASE + PMC_SIZE) {
      pmc_write(state, address, val);
    } else if (address >= 0x60007000 && address < 0x60007000 + 0x1000) {
      flow_write(state, address, val);
    } else if (address >= CLK_RST_BASE &&
               address < CLK_RST_BASE + CLK_RST_SIZE) {
      clk_rst_write(state, address, val);
    } else if (address >= FUSE_BASE && address < FUSE_BASE + FUSE_SIZE) {
      fuse_write(state, address, val);
    } else if (address >= SE_BASE && address < SE_BASE + SE_SIZE) {
      se_write(state, address, val);
    } else {
      // Log other MMIO writes but don't duplicate SDMMC/I2C/etc.
      if (!((address >= SDMMC1_BASE && address < SDMMC1_BASE + 0x1000) ||
            (address >= SDMMC4_BASE && address < SDMMC4_BASE + 0x1000) ||
            (address >= I2C1_BASE && address < I2C1_BASE + 0x1000) ||
            (address >= I2C5_BASE && address < I2C5_BASE + 0x1000) ||
            (address >= SE_BASE && address < SE_BASE + SE_SIZE) ||
            (address >= RTC_BASE && address < RTC_BASE + 0x1000))) {
        printf("[mmio] W: 0x%08llX = 0x%08X (PC=0x%08X)\n",
               (unsigned long long)address, val, pc);
        fflush(stdout);
      }
      misc_write(uc, state, address, val, size);
    }
  }
}

static bool hook_unmapped(uc_engine *uc, uc_mem_type type, uint64_t address,
                          int size, int64_t value, void *user_data) {
  uint32_t pc;
  uc_reg_read(uc, UC_ARM_REG_PC, &pc);
  if (type == UC_MEM_READ_UNMAPPED) {
    printf("[mmio] UNMAPPED R: 0x%08lX (PC=0x%08X)\n", (unsigned long)address,
           pc);
  } else {
    printf("[mmio] UNMAPPED W: 0x%08lX = 0x%08llX (PC=0x%08X)\n",
           (unsigned long)address, (unsigned long long)value, pc);
  }
  uint64_t page = address & ~0xFFFULL;
  uc_mem_map(uc, page, 0x1000, UC_PROT_ALL);
  if (type == UC_MEM_WRITE_UNMAPPED) {
    uc_mem_write(uc, address, &value, size);
  }
  return true;
}

void hook_code(uc_engine *uc, uint64_t address, uint32_t size,
               void *user_data) {
  EmuState *state = (EmuState *)user_data;
  static uint32_t last_pc = 0;
  static int loop_count = 0;
  static int logged_stall = 0;

  // Deterministic timer: 10 instructions per microsecond (approx 10MHz
  // speed-up)
  state->insn_count++;
  if ((state->insn_count % 10) == 0) {
    state->emu_usec++;
  }

  // Framebuffer-change-aware automation
  static uint32_t last_fb_sum = 0;
  static int fb_changes = 0;
  static int auto_stage = -1; // Disabled for manual debugging
  static uint64_t stage_start = 0;
  static uint64_t last_change_insn = 0;

  // Ignore changes for the first 2s (20M insns) to avoid boot transients
  if (auto_stage == 0 && state->insn_count > 20000000) {
    auto_stage = 1;
    printf("[emu] Initial boot period over. Waiting for screen change...\n");
  }

  if (auto_stage >= 1 && (state->insn_count % 1000000) == 0) {
    uint64_t current_fb = state->fb_addr ? state->fb_addr : 0xF5A00000;
    uint32_t pixels[2048];
    uint32_t sum = 0;

    // Sample Top rows for maximum sensitivity
    if (uc_mem_read(uc, current_fb, pixels, sizeof(pixels)) == UC_ERR_OK) {
      for (int i = 0; i < 2048; i++)
        sum += (pixels[i] & 0xFFFFFF);

      if (sum != last_fb_sum && sum != 0) {
        // Ensure there's a gap between 'changes' to let screens settle
        if (state->insn_count > last_change_insn + 5000000) { // 0.5s gap
          fb_changes++;
          last_fb_sum = sum;
          last_change_insn = state->insn_count;
          printf("[emu] FB Change #%d detected (sum=0x%08X). PC=0x%08llX\n",
                 fb_changes, sum, (unsigned long long)address);
          fflush(stdout);

          if (auto_stage == 1) { // Error Screen or Menu
            auto_stage = 2;
            stage_start = state->insn_count;
          } else if (auto_stage == 3) { // Subsequent menu transition
            auto_stage = 4;
            stage_start = state->insn_count;
          }
        }
      }
    }
  }

  // State Machine for Inputs (AutoRCM Verification Path)
  if (auto_stage == 2) { // Skip SD fail / Error screen
    uint64_t press = stage_start + 5000000;
    if (state->insn_count == press) {
      printf("[emu] POWER (Skip Screen)\n");
      state->btn_power = true;
    }
    if (state->insn_count == press + 1000000) {
      state->btn_power = false;
      auto_stage = 3;
    }
  } else if (auto_stage == 4) { // Main Menu reached Change #2
    uint64_t base = stage_start + 10000000;

    // 1. Move to "Tools..." (1 tap down from Launch)
    if (state->insn_count == base) {
      printf("[emu] VOL_DOWN (Select Tools)\n");
      state->btn_vol_down = true;
    }
    if (state->insn_count == base + 1000000) {
      state->btn_vol_down = false;
    }

    // 2. Enter Tools
    if (state->insn_count == base + 5000000) {
      printf("[emu] POWER (Enter Tools)\n");
      state->btn_power = true;
    }
    if (state->insn_count == base + 6000000) {
      state->btn_power = false;
    }

    // 3. Move to "AutoRCM" in Tools menu (2 taps down to skip Back and
    // Separator)
    if (state->insn_count == base + 10000000) {
      printf("[emu] VOL_DOWN (Select AutoRCM 1)\n");
      state->btn_vol_down = true;
    }
    if (state->insn_count == base + 11000000) {
      state->btn_vol_down = false;
    }
    if (state->insn_count == base + 15000000) {
      printf("[emu] VOL_DOWN (Select AutoRCM 2)\n");
      state->btn_vol_down = true;
    }
    if (state->insn_count == base + 16000000) {
      state->btn_vol_down = false;
    }

    // 4. Enter AutoRCM
    if (state->insn_count == base + 20000000) {
      printf("[emu] POWER (Enter AutoRCM)\n");
      state->btn_power = true;
    }
    if (state->insn_count == base + 21000000) {
      state->btn_power = false;
    }

    // 5. Toggle AutoRCM (Enable) / Clear potential red error
    if (state->insn_count == base + 30000000) {
      printf("[emu] POWER (Toggle/Skip Error 1)\n");
      state->btn_power = true;
    }
    if (state->insn_count == base + 31000000) {
      state->btn_power = false;
    }
    if (state->insn_count == base + 35000000) {
      printf("[emu] POWER (Toggle/Skip Error 2)\n");
      state->btn_power = true;
    }
    if (state->insn_count == base + 36000000) {
      state->btn_power = false;
    }

    // 6. Go back to Main Menu (Using VOL_UP (Back) repeatedly)
    if (state->insn_count == base + 50000000) {
      printf("[emu] VOL_UP (Back to Tools)\n");
      state->btn_vol_up = true;
    }
    if (state->insn_count == base + 51000000) {
      state->btn_vol_up = false;
    }

    if (state->insn_count == base + 60000000) {
      printf("[emu] VOL_UP (Back to Main Menu)\n");
      state->btn_vol_up = true;
    }
    if (state->insn_count == base + 61000000) {
      state->btn_vol_up = false;
    }

    // 7. Navigate to Power Off from Main Menu
    // Power Off is near the bottom. Let's do 10 taps of VOL_DOWN.
    uint64_t nav_base = base + 80000000;
    for (int i = 0; i < 10; i++) {
      uint64_t tap = nav_base + (i * 4000000);
      if (state->insn_count == tap) {
        printf("[emu] VOL_DOWN (Nav to Off %d)\n", i + 1);
        state->btn_vol_down = true;
      }
      if (state->insn_count == tap + 1000000) {
        state->btn_vol_down = false;
      }
    }

    // 8. Power Off
    uint64_t pwr_off = nav_base + 50000000;
    if (state->insn_count == pwr_off + 1000000) {
      state->btn_power = false;
      auto_stage = 5;
    }
  }

  if (address == last_pc) {
    loop_count++;
  } else {
    if (loop_count > 100000 && !logged_stall) {
      printf("[trace] STALL at PC=0x%08llX (looped %d times)\n",
             (unsigned long long)last_pc, loop_count);
      logged_stall = 1;
    } else if (loop_count == 0) {
      logged_stall = 0;
    }
    last_pc = (uint32_t)address;
    loop_count = 0;
  }
}

void mmio_init(uc_engine *uc, EmuState *state) {
  static uc_hook h_read, h_write, h_unmapped, h_code;

  const struct {
    uint64_t base;
    uint64_t size;
  } regions[] = {
      {CLK_RST_BASE, CLK_RST_SIZE},
      {TMR_BASE, TMR_SIZE},
      {GPIO_BASE, GPIO_SIZE},
      {PINMUX_BASE, PINMUX_SIZE},
      {UART_A_BASE, UART_SIZE},
      {I2C1_BASE, I2C_SIZE},
      {I2C5_BASE, I2C_SIZE},
      {RTC_BASE, RTC_SIZE},
      {PMC_BASE, PMC_SIZE},
      {FUSE_BASE, FUSE_SIZE},
      {0x60007000, 0x1000}, // FLOW_CTLR
      {EMC_BASE, EMC_SIZE},
      {MC_BASE, MC_SIZE},
      {DISPLAY_A_BASE, DISPLAY_SIZE},
      {DSI_BASE, DSI_SIZE},
      {MIPI_CAL_BASE, MIPI_CAL_SIZE},
      {SOR_BASE, SOR_SIZE},
      {SDMMC1_BASE, SDMMC_SIZE},
      {SDMMC4_BASE, SDMMC_SIZE},
      {APB_MISC_BASE, APB_MISC_SIZE},
      {HOST1X_BASE, HOST1X_SIZE},
      {VIC_BASE, VIC_SIZE},
      {SYSREG_BASE, SYSREG_SIZE},
      {I2S_BASE, I2S_SIZE},
      {SE_BASE, SE_SIZE},
      {TSEC_BASE, TSEC_SIZE},
      {SYSCTR0_BASE, SYSCTR0_SIZE},
      {BPMP_CACHE_BASE, BPMP_CACHE_SIZE}, // BPMP_CACHE
      {TZRAM_BASE, TZRAM_SIZE},           // TZRAM
  };

  for (const auto &r : regions) {
    uint64_t map_base = r.base & ~0xFFFULL;
    uint64_t map_end = (r.base + r.size + 0xFFF) & ~0xFFFULL;
    uint64_t map_size = map_end - map_base;
    uc_mem_map(uc, map_base, map_size, UC_PROT_ALL);
  }

  uc_hook_add(uc, &h_read, UC_HOOK_MEM_READ, (void *)hook_mmio_read, state,
              0x50000000, 0x7FFFFFFF);
  uc_hook_add(uc, &h_write, UC_HOOK_MEM_WRITE, (void *)hook_mmio_write, state,
              0x50000000, 0x7FFFFFFF);

  uc_hook_add(uc, &h_unmapped,
              UC_HOOK_MEM_READ_UNMAPPED | UC_HOOK_MEM_WRITE_UNMAPPED,
              (void *)hook_unmapped, state, 1, 0);

  uc_hook_add(uc, &h_code, UC_HOOK_CODE, (void *)hook_code, state, 1, 0);
}
