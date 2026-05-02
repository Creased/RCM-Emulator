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
    // Port Z IN. Bit 1 active-low = SD card detect.
    uint32_t val = state->sd_inserted.load() ? 0x00 : (1u << 1);
    return val;
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

// (Touch controller is the STMFTS at I2C_3 / slave 0x49, handled separately
// in t210/i2c3.cpp. Nothing else lives at I2C_1 / slave 0x4C besides TMP451.)

#define MAX77620_REG_ONOFFSTAT 0x15
#define MAX77620_ONOFFSTAT_EN0 BIT(2)

// ---- Packet-mode I2C (BM92T36 USB-PD on I2C_1 @ 0x18) ----
// MAX17050/MAX77620/BQ24193 use the simple "normal" path (CMD_DATA1 reads).
// BM92T36 uses Hekate's i2c_xfer_packet, which streams a multi-word header
// through TX_FIFO and reads data back through RX_FIFO. This is a small FSM
// that watches TX_FIFO writes, captures the slave/register/direction, and
// pre-fills an RX buffer when a read header arrives.
struct PacketState {
    int      hdr_idx       = 0;   // word index since last PROT magic
    uint8_t  dev_addr      = 0;
    uint32_t payload_size  = 0;   // bytes
    bool     is_read       = false;
    uint8_t  reg_addr      = 0;   // captured from prior write phase
    uint8_t  rx_buf[64]    = {0};
    uint32_t rx_size       = 0;
    uint32_t rx_pos        = 0;
};
static PacketState pkt_i2c1;
static PacketState pkt_i2c5;

#define I2C_PACKET_PROT_I2C  (1u << 4)
#define I2C_HEADER_READ      (1u << 19)

static void bm92t36_fill_rx(EmuState *state, uint8_t reg, uint8_t *buf, uint32_t size) {
    // All multi-byte values are little-endian on the wire (Hekate reassembles
    // with `(buf[1] << 8) | buf[0]`). FW_TYPE is the exception — Hekate uses
    // `(buf[0] << 4) | buf[1]` and expects 0x36, so buf[0]=3, buf[1]=6.
    auto put = [&](uint32_t i, uint8_t v) { if (i < size) buf[i] = v; };
    switch (reg) {
    case 0x03: // STATUS1: bit 7 = cable inserted
        put(0, state->usb_pd_inserted.load() ? 0x80 : 0x00);
        break;
    case 0x4B: // FW_TYPE_REG -> VER_36 = 0x36
        put(0, 0x03); put(1, 0x06);
        break;
    case 0x4D: // MAN_ID_REG -> MAN_ROHM = 0x04B5
        put(0, 0xB5); put(1, 0x04);
        break;
    case 0x4E: // DEV_ID_REG -> DEV_BM92T = 0x03B0
        put(0, 0xB0); put(1, 0x03);
        break;
    case 0x08:   // READ_PDOS_SRC: byte0 = PDO-bytes count, then 4-byte PDOs
    case 0x28: { // CURRENT_PDO:    same layout, 1 PDO
        // Synthesize a single Fixed-type PDO from the EmuState values.
        // pd_object_t bitfields (LSB->MSB): amp:10, volt:10, info:10, type:2.
        uint32_t amp_lsb  = (uint32_t)(state->usb_pd_amperage_ma.load() / 10) & 0x3FF;
        uint32_t volt_lsb = (uint32_t)(state->usb_pd_voltage_mv.load() / 50) & 0x3FF;
        uint32_t pdo = amp_lsb | (volt_lsb << 10);
        put(0, 4);
        put(1, (uint8_t)(pdo >>  0));
        put(2, (uint8_t)(pdo >>  8));
        put(3, (uint8_t)(pdo >> 16));
        put(4, (uint8_t)(pdo >> 24));
        break;
    }
    default:
        break;
    }
}

static void packet_populate_rx(EmuState *state, bool on_i2c5, PacketState &p) {
    p.rx_pos  = 0;
    p.rx_size = std::min((uint32_t)sizeof(p.rx_buf), p.payload_size);
    memset(p.rx_buf, 0, sizeof(p.rx_buf));
    if (!on_i2c5 && p.dev_addr == 0x18) {
        bm92t36_fill_rx(state, p.reg_addr, p.rx_buf, p.rx_size);
    }
    // Add other packet-mode slaves here as needed.
}

uint32_t i2c_read(EmuState *state, uint64_t addr) {
  bool on_i2c5 = (addr >= I2C5_BASE);
  uint32_t base = on_i2c5 ? I2C5_BASE : I2C1_BASE;
  uint32_t offset = (uint32_t)(addr - base);

  PacketState &pkt = on_i2c5 ? pkt_i2c5 : pkt_i2c1;

  switch (offset) {
  case 0x1C:          // I2C_STATUS
    return 0;         // Transaction complete, no error, not busy
  case 0x8C:          // I2C_CONFIG_LOAD
    return 0;         // MSTR_CONFIG_LOAD (bit 0) cleared = load complete
  case 0x68:          // I2C_INT_STATUS
    return (1 << 11); // BUS_CLEAR_DONE (bit 11)
  case 0x54: {        // I2C_RX_FIFO  (packet-mode receive)
    uint32_t word = 0;
    uint32_t n = std::min((uint32_t)4, pkt.rx_size - pkt.rx_pos);
    for (uint32_t i = 0; i < n; i++) {
      word |= (uint32_t)pkt.rx_buf[pkt.rx_pos++] << (i * 8);
    }
    return word;
  }
  case 0x58:          // I2C_PACKET_TRANSFER_STATUS
    // Hekate waits for ((status >> 4) & 0xFFF) == size-1 after each phase.
    // Return the last-captured payload size shifted; phase always completes
    // synchronously in our emulator.
    return (pkt.payload_size ? (pkt.payload_size - 1) : 0) << 4;
  case 0x60: {        // I2C_FIFO_STATUS
    // Bits[3:0] = RX_FIFO_FULL_CNT (entries available, each entry = 4 bytes).
    if (pkt.rx_size > pkt.rx_pos) {
      uint32_t words = (pkt.rx_size - pkt.rx_pos + 3) / 4;
      return words & 0xF;
    }
    return 0;
  }
  case 0x10:          // I2C_CMD_DATA2 (bytes 4-7) — no slaves currently need a >4 byte response
    return 0;
  case 0x0C:          // I2C_CMD_DATA1
    // TMP451 SoC/PCB thermal sensor (slave 0x4C on I2C_1).
    // Hekate reads (per bdk/thermal/tmp451.c) the integer °C from
    //   PCB: 0x00, SoC: 0x01
    // and the fractional byte from
    //   SoC dec: 0x10, PCB dec: 0x15
    // The fractional byte's high nibble is units of 1/16 °C; Hekate decodes
    //   tenths_of_C = ((dec >> 4) * 625) / 100
    // So encoding from a UI value of °C×10:
    //   lsb = (c10 * 8) / 5    // total LSBs (1 LSB = 1/16 °C = 0.625 c10)
    //   int_byte =  lsb >> 4
    //   dec_byte = (lsb & 0xF) << 4
    if (!on_i2c5 && i2c_slave_addr == 0x4C) {
      auto encode_lsb = [](int16_t c10) -> uint16_t {
        return (uint16_t)((int32_t)c10 * 8 / 5);
      };
      uint16_t soc_lsb = encode_lsb(state->soc_temp_c10.load());
      uint16_t pcb_lsb = encode_lsb(state->pcb_temp_c10.load());
      switch (i2c_reg_addr) {
      case 0x00: return (pcb_lsb >> 4) & 0xFF;          // PCB int (local)
      case 0x01: return (soc_lsb >> 4) & 0xFF;          // SoC int (remote)
      case 0x10: return (uint8_t)((soc_lsb & 0xF) << 4); // SoC dec
      case 0x15: return (uint8_t)((pcb_lsb & 0xF) << 4); // PCB dec
      default:   return 0;
      }
    }
    // MAX17050 fuel gauge (slave 0x36 on I2C_1).
    // All raw encodings here are the inverse of Hekate's max17050_get_property
    // formulas in bdk/power/max17050.c. Switch hardware uses Rsense=5mΩ with
    // CGAIN=2 → ADJ_RSENSE = 10mΩ, which sets the per-LSB units below.
    if (!on_i2c5 && i2c_slave_addr == 0x36) {
      switch (i2c_reg_addr) {
      case 0x05: { // RepCap   — 0.5 mAh/LSB (= mAh * 2)
        return (uint16_t)(state->bat_capacity_mah.load() * 2);
      }
      case 0x06: { // RepSOC   — %·256, Hekate displays (raw >> 8)
        return (uint16_t)(state->bat_soc_pct.load() << 8);
      }
      case 0x07: { // Age      — %·256, Hekate displays (raw >> 8)
        return (uint16_t)((uint16_t)state->bat_age_pct.load() << 8);
      }
      case 0x08: { // TEMP     — °C/256 signed, UI is °C·10
        int32_t scaled = (int32_t)state->bat_temp_c10.load() * 256 / 10;
        return (uint16_t)(int16_t)scaled;
      }
      case 0x09:   // VCELL    — 0.625 mV/LSB on the upper 13 bits, i.e. (raw >> 3) * 625 / 1000 = mV
      case 0x19:   // AvgVCELL — same encoding
      case 0xFB: { // OCVInternal — same encoding
        uint16_t mv = (i2c_reg_addr == 0xFB) ? state->bat_ocv_mv.load() : state->bat_vcell_mv.load();
        return (uint16_t)(((uint32_t)mv * 8000) / 625);
      }
      case 0x0A:   // Current   — 156.25 µA/LSB signed, UI is mA
      case 0x0B: { // AvgCurrent — same encoding
        int32_t raw = (int32_t)state->bat_current_ma.load() * 64 / 10;
        return (uint16_t)(int16_t)raw;
      }
      case 0x10: { // FullCAP    — 0.5 mAh/LSB
        return (uint16_t)(state->bat_full_cap_mah.load() * 2);
      }
      case 0x17: { // Cycles
        return state->bat_cycles.load();
      }
      case 0x18: { // DesignCap  — 0.5 mAh/LSB
        return (uint16_t)(state->bat_design_cap_mah.load() * 2);
      }
      case 0x1B: { // MinMaxVolt — packed (max << 8) | min, units of 20 mV
        uint16_t lo = (uint16_t)(state->bat_min_volt_mv.load() / 20) & 0xFF;
        uint16_t hi = (uint16_t)(state->bat_max_volt_mv.load() / 20) & 0xFF;
        return (hi << 8) | lo;
      }
      case 0x21:   // DevName — must be 0x00AC for max17050_get_version() to succeed
        return 0x00AC;
      case 0x3A: { // V_empty — (raw >> 7) * 10 = mV
        return (uint16_t)(((uint32_t)state->bat_v_empty_mv.load() / 10) << 7);
      }
      default:
        return 0;
      }
    }
    // MAX77620 PMIC (slave 0x3C on I2C_5).
    if (on_i2c5 && i2c_slave_addr == 0x3C) {
      switch (i2c_reg_addr) {
      case 0x15: { // ONOFFSTAT — EN0 bit reflects power button
        return state->btn_power.load() ? (1 << 2) : 0;
      }
      case 0x5B: return state->pmic_silicon_rev.load() & 0xF; // CID3: low nibble shown as "v%d"
      case 0x5C: return state->pmic_otp.load();                // CID4: 0x35 Erista, 0x53 Mariko
      case 0x5D: return 0; // CID5: ES version
      default:   return 0;
      }
    }
    // MAX77621 CPU/GPU regulator (slave 0x1B on I2C_5). Hekate reads CHIPID1
    // (reg 0x04) and prints the byte verbatim as the version number.
    if (on_i2c5 && i2c_slave_addr == 0x1B) {
      if (i2c_reg_addr == 0x04) return state->cpu_pmic_version.load();
      return 0;
    }
    // BQ24193 charger (slave 0x6B on I2C_1).
    // Each register is reverse-encoded from a decoded EmuState value (mA / mV
    // / °C) so the user-facing tweak reads in real units; the formulas mirror
    // bq24193_get_property() in bdk/power/bq24193.c.
    if (!on_i2c5 && i2c_slave_addr == 0x6B) {
      auto encode_input_current = [](uint16_t ma) -> uint8_t {
        // Table-quantized: pick nearest legal bucket.
        static const uint16_t tbl[8] = {100,150,500,900,1200,1500,2000,3000};
        uint8_t best = 0; int best_d = 0x7FFFFFFF;
        for (uint8_t i = 0; i < 8; i++) {
          int d = (int)tbl[i] - (int)ma; if (d < 0) d = -d;
          if (d < best_d) { best_d = d; best = i; }
        }
        return best;
      };
      auto clamp_div = [](uint16_t v, uint16_t base, uint16_t step, uint8_t maxbits) -> uint8_t {
        if (v < base) v = base;
        uint16_t units = (v - base) / step;
        uint16_t cap = (1u << maxbits) - 1;
        if (units > cap) units = cap;
        return (uint8_t)units;
      };
      switch (i2c_reg_addr) {
      case 0x00: { // InputSource: VINDPM[6:3] | INLIMIT[2:0]
        uint8_t ilim   = encode_input_current(state->chg_input_current_ma.load());
        uint8_t vindpm = clamp_div(state->chg_input_voltage_mv.load(), 3880, 80, 4);
        return (uint32_t)((vindpm << 3) | ilim);
      }
      case 0x01: { // PORConfig: keep CHGCONFIG=charger-en, set SYSMIN[3:1]
        uint8_t sysmin = clamp_div(state->chg_system_min_mv.load(), 3000, 100, 3);
        return (uint32_t)((1u << 4) | (sysmin << 1));
      }
      case 0x02: { // ChrgCurr: ICHG[7:2]
        uint8_t ichg = clamp_div(state->chg_fast_current_ma.load(), 512, 64, 6);
        return (uint32_t)(ichg << 2);
      }
      case 0x04: { // ChrgVolt: VREG[7:2]
        uint8_t vreg = clamp_div(state->chg_charge_voltage_mv.load(), 3504, 16, 6);
        return (uint32_t)(vreg << 2);
      }
      case 0x06: { // IRCompThermal: THERM[1:0]
        uint8_t c = state->chg_thermal_c.load();
        uint8_t therm = (c >= 110) ? 3 : (c >= 90) ? 2 : (c >= 70) ? 1 : 0;
        return (uint32_t)therm;
      }
      case 0x08: {            // Status: bits[7:6]=VBUS, [5:4]=CHRG, [2]=PG
        uint32_t v = 0;
        v |= (uint32_t)(state->chg_vbus_stat.load() & 0x3) << 6;
        v |= (uint32_t)(state->chg_chrg_stat.load() & 0x3) << 4;
        if (state->chg_power_good.load()) v |= (1 << 2);
        return v;
      }
      case 0x09: { // FaultReg: bits[2:0] = THERM_STAT (charger NTC thermistor).
        // Hekate decodes this independently of MAX17050's TEMP, but on real
        // hardware both sensors track the battery, so derive it from the
        // battery temp slider for consistency. Code map (per gui_info.c):
        //   0=Normal, 2=Warm, 3=Cool, 5=Cold, 6=Hot
        int16_t t10 = state->bat_temp_c10.load();
        uint8_t therm;
        if      (t10 <    0) therm = 5; // Cold
        else if (t10 <  100) therm = 3; // Cool
        else if (t10 <  450) therm = 0; // Normal
        else if (t10 <  500) therm = 2; // Warm
        else                 therm = 6; // Hot
        return therm;
      }
      case 0x0A: return 0x2F; // VendorPart — must be 0x2F for bq24193_get_version()
      default:   return 0;
      }
    }
    return 0;
  default:
    return 0;
  }
}

void i2c_write(EmuState *state, uint64_t addr, uint32_t val) {
  bool on_i2c5 = (addr >= I2C5_BASE);
  uint32_t base = on_i2c5 ? I2C5_BASE : I2C1_BASE;
  uint32_t offset = (uint32_t)(addr - base);

  PacketState &pkt = on_i2c5 ? pkt_i2c5 : pkt_i2c1;

  switch (offset) {
  case I2C_CMD_ADDR0:
    i2c_slave_addr = (val >> 1) & 0x7F;
    break;
  case I2C_CMD_DATA1:
    i2c_reg_addr = val & 0xFF;
    break;
  case 0x50: {        // I2C_TX_FIFO  (packet-mode dispatch)
    // Each packet starts with the PROT magic word; subsequent words follow a
    // fixed layout: [size-1, header, payload...]. The header carries dev_addr
    // and a READ flag; for write phases the first payload byte is the slave's
    // register address that the matching read phase will target.
    if (val == I2C_PACKET_PROT_I2C) {
      pkt.hdr_idx = 1;  // word 0 (PROT) just consumed
      return;
    }
    int idx = pkt.hdr_idx++;
    if (idx == 1) {
      pkt.payload_size = (val & 0xFFF) + 1;
    } else if (idx == 2) {
      pkt.dev_addr = (val >> 1) & 0x7F;
      pkt.is_read  = (val & I2C_HEADER_READ) != 0;
      i2c_slave_addr = pkt.dev_addr; // mirror for any cross-path lookups
      if (pkt.is_read) {
        packet_populate_rx(state, on_i2c5, pkt);
      }
    } else if (!pkt.is_read && idx == 3) {
      // First (and for our slaves, only) payload byte = register address.
      pkt.reg_addr = val & 0xFF;
    }
    break;
  }
  case 0x5C:          // I2C_FIFO_CONTROL — TX/RX flush at the start of each xfer
    if (val & 0x3) {
      pkt.hdr_idx = 0;
      pkt.rx_size = 0;
      pkt.rx_pos  = 0;
      // reg_addr intentionally preserved across the write→read transition
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

// Forward decls — these handlers live further down this file but are reached
// from misc_read's catch-all routing for DSI accesses.
uint32_t dsi_read(EmuState *state, uint64_t addr);
void     dsi_write(EmuState *state, uint64_t addr, uint32_t val);

uint32_t misc_read(EmuState *state, uint64_t addr) {
  if (addr >= PINMUX_BASE && addr < PINMUX_BASE + PINMUX_SIZE) {
    return pinmux_reg[(addr - PINMUX_BASE) / 4];
  }
  // APB_MISC_GP_HIDREV - hardware revision.
  // Bits 11:8 = chip ID (0x21 for both T210 / T210B01),
  // Bits  7:4 = major rev (1 = Erista T210, 2 = Mariko T210B01),
  // Bits  3:0 = minor.
  // Hekate's hw_get_chip_id() does `(HIDREV >> 4) & 0xF` and compares against
  // GP_HIDREV_MAJOR_T210B01 (=2) to decide h_cfg.t210b01.
  if (addr == APB_MISC_BASE + 0x804)
    return state->is_mariko.load() ? 0x20 : 0x10;

  // UART
  if (addr >= 0x70006000 && addr < 0x70006500) {
    uint32_t offset = (addr - 0x70006000) % 0x40;
    if (offset == 0x14)
      return 0x60; // LSR: THRE | TMTY always ready
    return 0;
  }
  // DSI
  if (addr >= DSI_BASE && addr < DSI_BASE + DSI_SIZE) {
    return dsi_read(state, addr);
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
    else if (offset == 0x24) {
      // PRESENT_STATE. SDMMC1 honours the SD-insert toggle; SDMMC4 (eMMC)
      // is always present.
      if (base == SDMMC1_BASE && !state->sd_inserted.load())
        result = 0; // no card present
      else
        result = 0x01F70000; // CARD_PRESENT | CD_STABLE | CD_LVL | DAT_LINE_LEVEL
    }
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

      // SDMMC1 with no SD card: signal CMD_TIMEOUT_ERROR for every command.
      // Hekate's sd_init_retry sees the timeout, falls back to checking
      // gpio_read(PORT_Z, 1), and bails with "Failed to init SD card".
      if (base == SDMMC1_BASE && !state->sd_inserted.load()) {
        norintsts |= 0x0001;
        errintsts |= (1 << 0);
        rsp[0] = rsp[1] = rsp[2] = rsp[3] = 0;
        return;
      }

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
          // Tegra's SDMMC uses register 0x58 as the SDMA system-address
          // register (Hekate writes the destination buffer pointer directly
          // there in sdmmc_driver.c::_sdmmc_dma_init). adma_addr in our state
          // captures that write, so it IS the destination, not a descriptor
          // pointer. Fall back to sysad (SDHCI standard 0x00) if 0x58 wasn't
          // programmed.
          dma_addr = adma_addr ? adma_addr : sysad;
          if (dma_addr) uc_mem_write(uc, dma_addr, ext_csd, 512);
          norintsts |= 0x0002; // TRANSFER_COMPLETE
        }
        break;
      case 1:
        rsp[0] = 0xC0FF8000;
        break;
      case 2: { // SEND_CID
        // Build the 16-byte CID from EmuState atomics, then pack into the
        // 4 R2 response registers. Hekate's _get_rsp shifts the 4 registers
        // left 8 bits (CRC strip) when assembling raw_cid, so we put the
        // CID bytes into rspreg pre-shifted right by 8.
        uint8_t cid[16] = {0};
        if (base == SDMMC1_BASE) {
          // SD CID per _sd_storage_parse_cid (bdk/storage/sdmmc.c).
          uint64_t pn = state->sd_cid_prod_name.load();
          cid[0]  = state->sd_cid_manfid.load();
          cid[1]  = (state->sd_cid_oemid.load() >> 8) & 0xFF;
          cid[2]  =  state->sd_cid_oemid.load()       & 0xFF;
          cid[3]  =  pn        & 0xFF;
          cid[4]  = (pn >> 8)  & 0xFF;
          cid[5]  = (pn >> 16) & 0xFF;
          cid[6]  = (pn >> 24) & 0xFF;
          cid[7]  = (pn >> 32) & 0xFF;
          cid[8]  = ((state->sd_cid_hwrev.load() & 0xF) << 4)
                  |  (state->sd_cid_fwrev.load() & 0xF);
          uint32_t sn = state->sd_cid_serial.load();
          cid[9]  = (sn >> 24) & 0xFF;
          cid[10] = (sn >> 16) & 0xFF;
          cid[11] = (sn >> 8)  & 0xFF;
          cid[12] =  sn        & 0xFF;
          uint8_t  yr = (uint8_t)(state->sd_cid_year.load() - 2000);
          cid[13] = (yr >> 4) & 0x0F;
          cid[14] = ((yr & 0x0F) << 4) | (state->sd_cid_month.load() & 0x0F);
        } else {
          // eMMC CID per _mmc_storage_parse_cid (MMC v4: 8-bit oemid + 6-byte
          // prod_name + prv + 32-bit serial + 4-bit month + 4-bit year offset
          // from 2013 because we report ext_csd.rev >= 5).
          uint64_t pn = state->emmc_cid_prod_name.load();
          cid[0]  = state->emmc_cid_manfid.load();
          cid[1]  = 0;
          cid[2]  = state->emmc_cid_oemid.load();
          cid[3]  =  pn        & 0xFF;
          cid[4]  = (pn >> 8)  & 0xFF;
          cid[5]  = (pn >> 16) & 0xFF;
          cid[6]  = (pn >> 24) & 0xFF;
          cid[7]  = (pn >> 32) & 0xFF;
          cid[8]  = (pn >> 40) & 0xFF;
          cid[9]  = state->emmc_cid_prv.load();
          uint32_t sn = state->emmc_cid_serial.load();
          cid[10] = (sn >> 24) & 0xFF;
          cid[11] = (sn >> 16) & 0xFF;
          cid[12] = (sn >> 8)  & 0xFF;
          cid[13] =  sn        & 0xFF;
          uint8_t  yr = (uint8_t)(state->emmc_cid_year.load() - 2013);
          cid[14] = ((state->emmc_cid_month.load() & 0x0F) << 4) | (yr & 0x0F);
        }
        rsp[3] = (cid[0]  << 16) | (cid[1]  << 8) |  cid[2];
        rsp[2] = (cid[3]  << 24) | (cid[4]  << 16) | (cid[5]  << 8) | cid[6];
        rsp[1] = (cid[7]  << 24) | (cid[8]  << 16) | (cid[9]  << 8) | cid[10];
        rsp[0] = (cid[11] << 24) | (cid[12] << 16) | (cid[13] << 8) | cid[14];
        break;
      }
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
  case 0x100: return state->fuse_0x100.load();
  case 0x110: return state->fuse_0x110.load();
  case 0x1A0: return state->fuse_0x1A0.load();
  case 0x148: return state->fuse_0x148.load();
  case 0x118: return state->fuse_0x118.load();
  // FUSE_RESERVED_ODM4 - bits 7:3 carry the DRAM ID on T210, with bits 14:12
  // appended on T210B01 to extend the range past 7.
  case 0x1D8: return state->fuse_0x1D8.load();
  default:    return 0;
  }
}

void fuse_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  (void)addr;
  (void)val;
}

// ==================== EMC (DRAM mode register reads) ====================
//
// Hekate's HW-info screen calls sdram_read_mrx(MRx), which:
//   1. writes EMC(EMC_MRR) with (rank << 30) | (mrx << 16) on the broadcast
//      bank at EMC_BASE,
//   2. polls EMC(EMC_EMC_STATUS) bit 20 (MRR_DIVLD) until set,
//   3. reads EMC_CH0(EMC_MRR) and EMC_CH1(EMC_MRR) from the per-channel
//      banks at EMC0_BASE / EMC1_BASE.
//
// We capture the requested mode register on the EMC_MRR write and route the
// per-channel reads back to the matching EmuState atomic.

static constexpr uint32_t EMC_ADR_CFG       = 0x010;
static constexpr uint32_t EMC_MRR           = 0x0EC;
static constexpr uint32_t EMC_EMC_STATUS    = 0x2B4;
static constexpr uint32_t EMC_FBIO_CFG7     = 0x584;
static constexpr uint32_t EMC_STATUS_MRR_DIVLD = 1u << 20;

static uint32_t g_last_mrr_mrx = 5;

static uint8_t emc_mrx_value(EmuState *state, uint32_t mrx) {
  switch (mrx) {
  case 5: return state->dram_vendor.load();
  case 6: return state->dram_rev_id1.load();
  case 7: return state->dram_rev_id2.load();
  case 8: return state->dram_density.load();
  default: return 0;
  }
}

uint32_t emc_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr & 0xFFF);
  bool per_channel = (addr >= EMC0_BASE);

  if (per_channel) {
    if (offset == EMC_MRR)
      return emc_mrx_value(state, g_last_mrr_mrx);
    return 0;
  }

  switch (offset) {
  case EMC_ADR_CFG:    return 0;                          // single rank
  case EMC_FBIO_CFG7:  return (1u << 1) | (1u << 2);      // ch0 + ch1 enabled
  case EMC_EMC_STATUS: return EMC_STATUS_MRR_DIVLD;       // MRR data always valid
  case EMC_MRR:        return emc_mrx_value(state, g_last_mrr_mrx);
  default:             return 0;
  }
}

void emc_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  uint32_t offset = (uint32_t)(addr & 0xFFF);
  if (offset == EMC_MRR && addr < EMC0_BASE) {
    g_last_mrr_mrx = (val >> 16) & 0xFF;
  }
}

// ==================== DSI (display panel ID over MIPI-DSI) ====================
//
// Hekate's display init calls display_dsi_read(MIPI_DCS_GET_DISPLAY_ID, 3, ...)
// to identify the LCD panel. The read sequence:
//   1. send (cmd << 8) | MIPI_DSI_DCS_READ to DSI_WR_DATA, then write
//      DSI_TRIGGER = HOST. We capture the requested DCS reg here.
//   2. write DSI_HOST_CONTROL with bit 3 (IMM_BTA) set, then poll until that
//      bit clears. We auto-clear it on the next read and prep an RX FIFO.
//   3. read DSI_STATUS for fifo count, then drain DSI_RD_DATA. We expose
//      three words: DSI_ESCAPE_CMD, (3 << 8) | DCS_LONG_RD_RES, panel_id_raw.
//
// On any unrecognized DCS read, _panel_id_raw stays at the 0xCCCCCC sentinel
// Hekate seeds, which renders as "Failed to get info!". Default panel reply
// matches a JDI LAM062M109A (0x099310 -> decoded 0x0910).

static constexpr uint32_t DSI_RD_DATA      = 0x9 << 2;  // 0x024
static constexpr uint32_t DSI_WR_DATA      = 0xA << 2;  // 0x028
static constexpr uint32_t DSI_HOST_CONTROL = 0xF << 2;  // 0x03C
static constexpr uint32_t DSI_TRIGGER      = 0x13 << 2; // 0x04C
static constexpr uint32_t DSI_STATUS       = 0x15 << 2; // 0x054

static constexpr uint8_t MIPI_DSI_DCS_READ            = 0x06;
static constexpr uint8_t MIPI_DCS_GET_DISPLAY_ID      = 0x04;
static constexpr uint8_t DSI_ESCAPE_CMD               = 0x87;
static constexpr uint8_t DCS_LONG_RD_RES              = 0x1C;

static uint8_t  g_dsi_pending_dcs_cmd = 0;
static uint32_t g_dsi_rx_fifo[8]      = {0};
static uint32_t g_dsi_rx_count        = 0;
static uint32_t g_dsi_rx_pos          = 0;

static void dsi_prepare_response(EmuState *state) {
  if (g_dsi_pending_dcs_cmd != MIPI_DCS_GET_DISPLAY_ID) {
    g_dsi_rx_count = 0;
    return;
  }
  g_dsi_rx_fifo[0] = DSI_ESCAPE_CMD;
  g_dsi_rx_fifo[1] = (3u << 8) | DCS_LONG_RD_RES;
  g_dsi_rx_fifo[2] = state->panel_id_raw.load() & 0xFFFFFF;
  g_dsi_rx_count   = 3;
  g_dsi_rx_pos     = 0;
}

uint32_t dsi_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - DSI_BASE);
  switch (offset) {
  case DSI_STATUS: {
    uint32_t left = (g_dsi_rx_pos < g_dsi_rx_count) ? (g_dsi_rx_count - g_dsi_rx_pos) : 0;
    return left & 0x1F; // DSI_STATUS_RX_FIFO_SIZE mask
  }
  case DSI_RD_DATA:
    if (g_dsi_rx_pos < g_dsi_rx_count)
      return g_dsi_rx_fifo[g_dsi_rx_pos++];
    return 0;
  case DSI_HOST_CONTROL: return 0; // IMM_BTA always clear (we ack instantly)
  case DSI_TRIGGER:      return 0; // trigger always clear
  default:               return 0;
  }
  (void)state;
}

void dsi_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - DSI_BASE);
  if (offset == DSI_WR_DATA && (val & 0xFF) == MIPI_DSI_DCS_READ) {
    g_dsi_pending_dcs_cmd = (val >> 8) & 0xFF;
  } else if (offset == DSI_HOST_CONTROL && (val & (1u << 3))) {
    dsi_prepare_response(state);
  }
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
    } else if ((address >= EMC_BASE  && address < EMC_BASE  + EMC_SIZE) ||
               (address >= EMC0_BASE && address < EMC0_BASE + EMC_SIZE) ||
               (address >= EMC1_BASE && address < EMC1_BASE + EMC_SIZE)) {
      result = emc_read(state, address);
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
    } else if (address >= DSI_BASE && address < DSI_BASE + DSI_SIZE) {
      dsi_write(state, address, val);
    } else if (address >= PMC_BASE && address < PMC_BASE + PMC_SIZE) {
      pmc_write(state, address, val);
    } else if (address >= 0x60007000 && address < 0x60007000 + 0x1000) {
      flow_write(state, address, val);
    } else if (address >= CLK_RST_BASE &&
               address < CLK_RST_BASE + CLK_RST_SIZE) {
      clk_rst_write(state, address, val);
    } else if (address >= FUSE_BASE && address < FUSE_BASE + FUSE_SIZE) {
      fuse_write(state, address, val);
    } else if ((address >= EMC_BASE  && address < EMC_BASE  + EMC_SIZE) ||
               (address >= EMC0_BASE && address < EMC0_BASE + EMC_SIZE) ||
               (address >= EMC1_BASE && address < EMC1_BASE + EMC_SIZE)) {
      emc_write(state, address, val);
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
      {EMC0_BASE, EMC_SIZE},
      {EMC1_BASE, EMC_SIZE},
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
