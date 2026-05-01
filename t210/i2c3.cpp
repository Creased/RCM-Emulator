#include "i2c3.h"
#include "../emu_state.h"
#include "memory_map.h"

#include <array>
#include <cstdio>
#include <cstring>
#include <vector>

// ============================================================================
//  I²C3 + STMFTS (FTS4) touchscreen stub
//
//  Reference:
//   - hekate/bdk/soc/i2c.c       — Tegra I²C controller register protocol
//   - hekate/bdk/input/touch.{c,h} — STMFTS / FTS4 wire protocol on I²C3@0x49
//
//  Hekate uses two I²C transfer modes:
//   * Normal mode: writes/reads ≤8 bytes via CMD_DATA1/CMD_DATA2 with a single
//     CMD_ADDR0+CNFG round-trip. Used for short FTS4 commands.
//   * Packet mode: variable-length, FIFO-driven. A 3-word header is pushed to
//     TX_FIFO, then payload words. Combined write+read sequences use a
//     REP_START header followed by a READ header. Used for FTS4 register
//     reads (HW_REG_READ, FB_REG_READ) and event polling.
//
//  This stub captures TX bytes from both modes into `tx_buf`, runs an
//  `execute()` step that interprets the FTS4 command and produces RX bytes
//  in `rx_buf`, and serves those bytes back through CMD_DATA1/2 (normal) or
//  RX_FIFO (packet).
// ============================================================================

namespace {

// ---- FTS4 command opcodes (subset we need) ----
constexpr uint8_t  FTS4_CMD_READ_INFO            = 0x80;
constexpr uint8_t  FTS4_CMD_READ_STATUS          = 0x84;
constexpr uint8_t  FTS4_CMD_READ_ONE_EVENT       = 0x85;
constexpr uint8_t  FTS4_CMD_READ_ALL_EVENT       = 0x86;
constexpr uint8_t  FTS4_CMD_LATEST_EVENT         = 0x87;
constexpr uint8_t  FTS4_CMD_MS_MT_SENSE_OFF      = 0x92;
constexpr uint8_t  FTS4_CMD_MS_MT_SENSE_ON       = 0x93;
constexpr uint8_t  FTS4_CMD_SYSTEM_RESET         = 0xA0;
constexpr uint8_t  FTS4_CMD_CLEAR_EVENT_STACK    = 0xA1;
constexpr uint8_t  FTS4_CMD_HW_REG_READ          = 0xB6; // also HW_REG_WRITE
constexpr uint8_t  FTS4_CMD_SWITCH_SENSE_MODE    = 0xC3;
constexpr uint8_t  FTS4_CMD_FB_REG_READ          = 0xD0; // also FB_REG_WRITE

// ---- FTS4 HW register addresses (accessed via 0xB6) ----
constexpr uint16_t FTS4_HW_REG_CHIP_ID_INFO_ALT  = 0x0002; // returns chip_id at bytes [3..4]
constexpr uint16_t FTS4_HW_REG_CHIP_ID_INFO      = 0x0004; // returns chip_id at bytes [1..2]
constexpr uint16_t FTS4_HW_REG_EVENT_COUNT       = 0x0023;
constexpr uint16_t FTS4_HW_REG_SYS_RESET         = 0x0028;

// ---- FTS4 chip identity ----
constexpr uint16_t FTS4_CHIP_ID                  = 0x3670;

// ---- FTS4 event opcodes (low nibble of event[0]) ----
constexpr uint8_t  FTS4_EV_NO_EVENT              = 0x00;
constexpr uint8_t  FTS4_EV_MULTI_TOUCH_ENTER     = 0x03;
constexpr uint8_t  FTS4_EV_MULTI_TOUCH_LEAVE     = 0x04;
constexpr uint8_t  FTS4_EV_MULTI_TOUCH_MOTION    = 0x05;
constexpr uint8_t  FTS4_EV_CONTROLLER_READY      = 0x10;

// ---- Tegra I²C packet header flags (mmio.h-style bit positions) ----
constexpr uint32_t I2C_HEADER_CONT_XFER          = 1u << 15;
constexpr uint32_t I2C_HEADER_REP_START          = 1u << 16;
constexpr uint32_t I2C_HEADER_READ               = 1u << 19;

// ---- I²C controller register offsets (within I2C3_BASE) ----
constexpr uint32_t I2C_CNFG                = 0x00;
constexpr uint32_t I2C_CMD_ADDR0           = 0x04;
constexpr uint32_t I2C_CMD_DATA1           = 0x0C;
constexpr uint32_t I2C_CMD_DATA2           = 0x10;
constexpr uint32_t I2C_STATUS              = 0x1C;
constexpr uint32_t I2C_TX_FIFO             = 0x50;
constexpr uint32_t I2C_RX_FIFO             = 0x54;
constexpr uint32_t I2C_PACKET_TRANSFER_STATUS = 0x58;
constexpr uint32_t I2C_FIFO_CONTROL        = 0x5C;
constexpr uint32_t I2C_FIFO_STATUS         = 0x60;
constexpr uint32_t I2C_INT_STATUS          = 0x68;
constexpr uint32_t I2C_CONFIG_LOAD         = 0x8C;

constexpr uint32_t CNFG_CMD1_READ          = 1u << 6;   // 0=write, 1=read
constexpr uint32_t CNFG_NORMAL_MODE_GO     = 1u << 9;
constexpr uint32_t CNFG_PACKET_MODE_GO     = 1u << 10;
constexpr uint32_t INT_STATUS_BUS_CLEAR_DONE = 1u << 11;

// ---- I²C controller state ----
uint8_t  slave_addr     = 0;     // 7-bit target slave
bool     dir_read       = false; // last direction set via CMD_ADDR0 / packet header
uint32_t cmd_data[2]    = {0};   // staged CMD_DATA1/CMD_DATA2 for normal-mode writes
uint32_t last_cnfg      = 0;     // last value written to CNFG; the driver does a
                                  // read-modify-write to set NORMAL_MODE_GO, so we
                                  // must echo this back through i2c3_read.

// Packet-mode header parser
enum PktState { PKT_HDR_W1, PKT_HDR_W2, PKT_HDR_W3, PKT_PAYLOAD };
PktState pkt_state      = PKT_HDR_W1;
uint32_t pkt_size       = 0;     // expected payload bytes (size-1 from header word 2)
uint32_t pkt_recv       = 0;     // payload bytes received so far
bool     pkt_is_combined = false; // true if write phase of REP_START combined op (defer execute)

// FTS4 command/response buffers
std::vector<uint8_t> tx_buf;     // accumulated TX bytes (cmd byte first)
std::vector<uint8_t> rx_buf;     // staged RX bytes
size_t               rx_pos = 0; // read pointer into rx_buf

// FTS4 event stack (head = oldest event). Each entry is one 8-byte event.
std::vector<std::array<uint8_t, 8>> ev_stack;

// ---- Helpers ----

void push_event(uint8_t op, uint8_t finger, uint16_t x, uint16_t y, uint16_t pressure) {
  std::array<uint8_t, 8> ev = {0};
  ev[0] = (uint8_t)((finger << 4) | (op & 0x0F));
  ev[1] = (uint8_t)((x >> 4) & 0xFF);
  ev[2] = (uint8_t)((y >> 4) & 0xFF);
  ev[3] = (uint8_t)(((x & 0xF) << 4) | (y & 0xF));
  ev[4] = (uint8_t)(pressure & 0xFF);
  ev[5] = (uint8_t)((pressure >> 8) & 0xFF);
  ev[6] = 0x40; // pressure modifier sentinel — touch.c uses 0x40 fallback
  ev[7] = 0x00;
  if (ev_stack.size() < 32) ev_stack.push_back(ev);
}

void push_controller_ready() {
  std::array<uint8_t, 8> ev = {FTS4_EV_CONTROLLER_READY, 0, 0, 0, 0, 0, 0, 0};
  if (ev_stack.size() < 32) ev_stack.push_back(ev);
}

// If SDL has flagged a pending touch event, pack it into the FTS4 stack.
void try_emit_pending_touch(EmuState *state) {
  bool was_pending = state->tc_event_pending.exchange(false);
  if (!was_pending) return;
  uint8_t  op  = state->tc_event_op.load();
  uint16_t x   = state->tc_x.load();
  uint16_t y   = state->tc_y.load();
  // pressure: a small non-zero value keeps z below the 500-palm-reject threshold
  // in touch.c (z = (pressure_lo|pressure_hi<<8) << 6 / (mod+0x40)).
  uint16_t pressure = (op == FTS4_EV_MULTI_TOUCH_LEAVE) ? 0 : 0x10;
  push_event(op, state->tc_finger_id, x, y, pressure);
}

// Pop oldest event into 8-byte buffer; on empty, return NO_EVENT.
void pop_event_to(uint8_t *buf) {
  if (ev_stack.empty()) {
    std::memset(buf, 0, 8);
    buf[0] = FTS4_EV_NO_EVENT;
    return;
  }
  std::memcpy(buf, ev_stack.front().data(), 8);
  ev_stack.erase(ev_stack.begin());
}

// Run the buffered TX as an FTS4 command; populate rx_buf for the read phase.
void execute(EmuState *state, uint32_t expected_rx_size) {
  rx_buf.clear();
  rx_pos = 0;
  if (tx_buf.empty()) return;

  uint8_t cmd = tx_buf[0];

  if (slave_addr != 0x49) {
    // Not the touchscreen — return zeros (no other slaves modelled on I²C3).
    if (dir_read) rx_buf.assign(expected_rx_size ? expected_rx_size : 1, 0);
    return;
  }

  switch (cmd) {
  case FTS4_CMD_HW_REG_READ: { // 0xB6 — also HW_REG_WRITE
    if (tx_buf.size() < 3) break;
    uint16_t reg = ((uint16_t)tx_buf[1] << 8) | tx_buf[2];
    if (dir_read) {
      // HW_REG_READ
      if (reg == FTS4_HW_REG_CHIP_ID_INFO) {
        // touch.c expects: chip_id = buf[1]<<8 | buf[2]; fw = buf[3]<<8 | buf[4]
        rx_buf = {0x00,
                  (uint8_t)(FTS4_CHIP_ID >> 8), (uint8_t)(FTS4_CHIP_ID & 0xFF),
                  0x00, 0x01,    // fw_ver = 0x0001
                  0x00, 0x00};   // config_id, config_ver
      } else if (reg == FTS4_HW_REG_CHIP_ID_INFO_ALT) {
        // Anti-clone check: chip_id at bytes [3..4]
        rx_buf = {0x00, 0x00, 0x00,
                  (uint8_t)(FTS4_CHIP_ID >> 8), (uint8_t)(FTS4_CHIP_ID & 0xFF),
                  0x00, 0x00};
      } else if (reg == FTS4_HW_REG_EVENT_COUNT) {
        try_emit_pending_touch(state);
        // touch_get_event_count: returns buf[1] >> 1
        uint8_t cnt = (uint8_t)ev_stack.size();
        rx_buf = {0x00, (uint8_t)(cnt << 1)};
      } else {
        rx_buf.assign(expected_rx_size ? expected_rx_size : 4, 0);
      }
    } else {
      // HW_REG_WRITE: tx_buf[1..2] = reg, tx_buf[3..] = value
      if (reg == FTS4_HW_REG_SYS_RESET && tx_buf.size() >= 4 && tx_buf[3] == 0x80) {
        ev_stack.clear();
        push_controller_ready();
      }
      // Other HW writes: ignored.
    }
    break;
  }
  case FTS4_CMD_FB_REG_READ: { // 0xD0
    // FB read of FW info etc — return zeros, Hekate just memcpys the result.
    rx_buf.assign(expected_rx_size ? expected_rx_size : 4, 0);
    break;
  }
  case FTS4_CMD_LATEST_EVENT:
  case FTS4_CMD_READ_ONE_EVENT:
  case FTS4_CMD_READ_ALL_EVENT: {
    try_emit_pending_touch(state);
    rx_buf.resize(8);
    pop_event_to(rx_buf.data());
    break;
  }
  case FTS4_CMD_SYSTEM_RESET:
    ev_stack.clear();
    push_controller_ready();
    break;
  case FTS4_CMD_CLEAR_EVENT_STACK:
    ev_stack.clear();
    break;
  case FTS4_CMD_MS_MT_SENSE_ON:
  case FTS4_CMD_MS_MT_SENSE_OFF:
  case FTS4_CMD_SWITCH_SENSE_MODE:
  case FTS4_CMD_READ_INFO:
  case FTS4_CMD_READ_STATUS:
  default:
    // Unmodelled writes are ACKed silently; unmodelled reads return zeros.
    if (dir_read) rx_buf.assign(expected_rx_size ? expected_rx_size : 1, 0);
    break;
  }

  printf("[i2c3] EXEC slave=0x%02X dir=%c cmd=0x%02X tx=%zuB rx=%zuB stack=%zu\n",
         slave_addr, dir_read ? 'R' : 'W', cmd, tx_buf.size(), rx_buf.size(),
         ev_stack.size());
}

// Pack up to 4 bytes from rx_buf at the current read pointer into a u32.
uint32_t pull_rx_word() {
  uint32_t v = 0;
  for (size_t i = 0; i < 4 && rx_pos + i < rx_buf.size(); ++i) {
    v |= ((uint32_t)rx_buf[rx_pos + i]) << (i * 8);
  }
  rx_pos += 4;
  return v;
}

} // namespace

uint32_t i2c3_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - I2C3_BASE);
  switch (offset) {
  case I2C_CNFG:
    // The driver does CNFG = (CNFG & ~NORMAL_MODE_GO) | NORMAL_MODE_GO to kick
    // off a transfer; if we returned 0, the size field would be lost. Echo the
    // last written CNFG value back so the read-modify-write preserves config.
    return last_cnfg;
  case I2C_CMD_DATA1:
    return pull_rx_word();
  case I2C_CMD_DATA2:
    return pull_rx_word();
  case I2C_STATUS:
    return 0; // not busy, no NACK
  case I2C_RX_FIFO:
    return pull_rx_word();
  case I2C_PACKET_TRANSFER_STATUS: {
    // Bits 4..15 = bytes transferred. Driver compares == (size - 1).
    uint32_t bytes = pkt_size;
    return (bytes ? (bytes - 1) << 4 : 0);
  }
  case I2C_FIFO_STATUS: {
    size_t left = (rx_pos < rx_buf.size()) ? (rx_buf.size() - rx_pos) : 0;
    uint32_t words = (uint32_t)((left + 3) / 4);
    if (words > 8) words = 8;
    return words; // RX_FIFO_FULL_CNT in low 4 bits
  }
  case I2C_INT_STATUS:
    return INT_STATUS_BUS_CLEAR_DONE;
  case I2C_CONFIG_LOAD:
    return 0; // load complete
  default:
    return 0;
  }
  (void)state;
}

void i2c3_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - I2C3_BASE);
  switch (offset) {
  case I2C_CNFG: {
    last_cnfg = val;
    bool packet_go = val & CNFG_PACKET_MODE_GO;
    bool normal_go = val & CNFG_NORMAL_MODE_GO;
    bool is_read   = val & CNFG_CMD1_READ;
    uint32_t total = ((val >> 1) & 0x7) + 1; // (size-1)+1 in bits [3:1]

    if (normal_go) {
      if (!is_read) {
        // Capture TX bytes from CMD_DATA1/CMD_DATA2.
        tx_buf.clear();
        for (uint32_t i = 0; i < total; ++i) {
          uint32_t word = (i < 4) ? cmd_data[0] : cmd_data[1];
          uint8_t  b    = (word >> ((i % 4) * 8)) & 0xFF;
          tx_buf.push_back(b);
        }
        dir_read = false;
        execute(state, 0);
      } else {
        // Read direction — rx_buf was prepared by the preceding write+execute.
        // Driver will read CMD_DATA1 (and DATA2 if total > 4) next.
        dir_read = true;
        // If no rx prepared (e.g. raw read with no preceding write), produce zeros.
        if (rx_buf.empty()) rx_buf.assign(total, 0);
        rx_pos = 0;
      }
    }
    if (packet_go) {
      // Reset packet header parser; subsequent TX_FIFO writes start a fresh packet.
      pkt_state = PKT_HDR_W1;
      pkt_size  = 0;
      pkt_recv  = 0;
    }
    break;
  }
  case I2C_CMD_ADDR0:
    slave_addr = (val >> 1) & 0x7F;
    dir_read   = (val & 1);
    break;
  case I2C_CMD_DATA1:
    cmd_data[0] = val;
    break;
  case I2C_CMD_DATA2:
    cmd_data[1] = val;
    break;
  case I2C_TX_FIFO: {
    switch (pkt_state) {
    case PKT_HDR_W1:
      pkt_state = PKT_HDR_W2;
      break;
    case PKT_HDR_W2:
      pkt_size  = val + 1;
      pkt_state = PKT_HDR_W3;
      break;
    case PKT_HDR_W3: {
      bool is_read      = val & I2C_HEADER_READ;
      bool is_rep_start = val & I2C_HEADER_REP_START;
      // Re-extract slave addr from header (bits 1..7 = addr<<1, bit 0 = R/W flag)
      uint8_t hdr_slave = (val >> 1) & 0x7F;
      slave_addr        = hdr_slave;
      dir_read          = is_read;

      if (is_read) {
        // Combined read phase: produce RX from previously buffered TX.
        execute(state, pkt_size);
        pkt_state = PKT_HDR_W1;
      } else {
        // Write phase. Defer execute if a read header is expected to follow.
        tx_buf.clear();
        pkt_recv         = 0;
        pkt_is_combined  = is_rep_start;
        pkt_state        = PKT_PAYLOAD;
      }
      break;
    }
    case PKT_PAYLOAD:
      for (int b = 0; b < 4 && pkt_recv < pkt_size; ++b) {
        tx_buf.push_back((val >> (b * 8)) & 0xFF);
        pkt_recv++;
      }
      if (pkt_recv >= pkt_size) {
        if (!pkt_is_combined) {
          // Pure write (CONT_XFER) — execute now with no expected RX.
          execute(state, 0);
        }
        // Combined writes defer execution to the upcoming READ header.
        pkt_state = PKT_HDR_W1;
      }
      break;
    }
    break;
  }
  case I2C_FIFO_CONTROL:
    if (val & 0x1) { // RX_FIFO_FLUSH
      rx_pos = rx_buf.size();
    }
    break;
  case I2C_INT_STATUS:
    // Write-1-clear from driver — no internal state to clear.
    break;
  case I2C_CONFIG_LOAD:
    // Auto-clears in real HW; we already report 0 on read.
    break;
  default:
    break;
  }
}
