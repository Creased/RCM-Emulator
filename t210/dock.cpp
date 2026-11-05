#include "dock.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <vector>

#include "../emu_state.h"
#include "memory_map.h"
#include "mmio.h"

/*
 * See dock.h. Sources: switchroot's drivers/misc/bm92txx.c (register map,
 * commands, VDM formats, the Nintendo dock's VDMs), HOS's usb sysmodule as
 * read in IDA (boot-time reset), hwtest's own measurements on a Mariko with
 * a Nintendo dock and a Ugreen adapter (status words, DPCD, branch ids), the
 * DisplayPort 1.2 DPCD map, L4T's dpaux/sor/dp drivers for the SoC side.
 */

namespace {

EmuState *g_state = nullptr;

// ==========================================================================
// Time and a tiny event queue
// ==========================================================================

uint64_t now_us() { return g_state ? g_state->emu_usec : 0; }

struct Event {
  uint64_t at;
  uint32_t gen;                    // dropped when the chip resets
  std::function<void()> fn;
};
std::vector<Event> events;
uint32_t chip_gen = 1;

void after_us(uint64_t delay, std::function<void()> fn) {
  events.push_back({now_us() + delay, chip_gen, std::move(fn)});
}

FILE *trace_file() {
  static FILE *f = nullptr;
  static bool tried = false;
  if (!tried) {
    tried = true;
    if (const char *p = getenv("RCM_EMU_DPTRACE"))
      f = fopen(p, "w");
  }
  return f;
}

#define DLOG(...) do { printf("[dock] " __VA_ARGS__); fflush(stdout); } while (0)

void trace(const char *what, uint32_t off, uint32_t val) {
  if (FILE *f = trace_file()) {
    fprintf(f, "%10llu %s %03X %08X\n", (unsigned long long)now_us(), what, off, val);
    fflush(f);
  }
}

// ==========================================================================
// BM92T36
// ==========================================================================

constexpr uint8_t REG_ALERT = 0x02, REG_STATUS1 = 0x03, REG_STATUS2 = 0x04,
                  REG_COMMAND = 0x05, REG_CONFIG1 = 0x06, REG_DEV_CAPS = 0x07,
                  REG_PDOS_SRC = 0x08, REG_CONFIG2 = 0x17, REG_DP_STATUS = 0x18,
                  REG_DP_ALERT_EN = 0x19, REG_VENDOR_CFG = 0x1A, REG_1C = 0x1C,
                  REG_1D = 0x1D, REG_AUTO_NGT_FIXED = 0x20, REG_AUTO_NGT_BATT = 0x23,
                  REG_SYS_CONFIG1 = 0x26, REG_SYS_CONFIG2 = 0x27,
                  REG_CURRENT_PDO = 0x28, REG_CURRENT_RDO = 0x2B,
                  REG_ALERT_ENABLE = 0x2E, REG_SYS_CONFIG3 = 0x2F,
                  REG_SET_RDO = 0x30, REG_PDOS_SNK = 0x33, REG_PDOS_PROV = 0x3C,
                  REG_FW_TYPE = 0x4B, REG_FW_REV = 0x4C, REG_MAN_ID = 0x4D,
                  REG_DEV_ID = 0x4E, REG_REV_ID = 0x4F, REG_INCOMING_VDM = 0x50,
                  REG_OUTGOING_VDM = 0x60;

constexpr uint16_t AL_SNK_FAULT = 1u << 0, AL_CMD_DONE = 1u << 2,
                   AL_PLUGPULL = 1u << 3, AL_DP_EVENT = 1u << 6,
                   AL_DR_SWAP = 1u << 10, AL_VDM_RECEIVED = 1u << 11,
                   AL_CONTRACT = 1u << 12, AL_PDO = 1u << 14;

constexpr uint16_t ST1_INSERT = 1u << 7, ST1_VSAFE = 1u << 10,
                   ST1_CMD_BUSY = 1u << 13, ST1_SPDSNK = 1u << 14;
constexpr int LASTCMD_COMPLETE = 0, LASTCMD_INVALID = 4, LASTCMD_REJECTED = 6;
constexpr int DR_NONE = 0, DR_UFP = 1, DR_DFP = 2;

constexpr uint16_t CMD_PS_RDY = 0x0505, CMD_GET_SRC_CAP = 0x0606,
                   CMD_SEND_RDO = 0x0707, CMD_HARD_RST = 0x0808,
                   CMD_SYS_RESET = 0x0D0D, CMD_SEND_VDM = 0x1111,
                   CMD_ACCEPT_VDM = 0x1616, CMD_DR_SWAP = 0x1818,
                   CMD_SOFT_RST = 0x2C2C, CMD_DP_ENTER = 0x3131,
                   CMD_DP_STOP = 0x3232, CMD_START_HPD = 0x3434,
                   CMD_DP_CFG_HPD = 0x3636, CMD_STOP_HPD = 0x3939,
                   CMD_STOP_HPD_EXIT_DP = 0x3B3B;

struct Pdo { uint32_t mv, ma; bool drd; };

struct Bm92 {
  bool reset_held = false;      // PV5 high
  uint64_t answers_from = 0;    // I2C ACKs from here on (boot after reset)

  uint16_t alert = 0, alert_enable = 0xFFFF;
  int lastcmd = LASTCMD_COMPLETE;
  bool busy = false;
  uint16_t config1 = 0xC0E0, config2 = 0x0000;
  uint16_t sys_config1 = 0x9749, sys_config2 = 0x0244, sys_config3 = 0;
  uint16_t dp_alert_en = 0, vendor_cfg = 0;
  uint16_t dev_caps = 0x02A2, fw_type = 0x0603, fw_rev = 0x0D7C;
  uint16_t man_id = 0x04B5, dev_id = 0x03B0, rev_id = 0x0000;
  uint8_t set_rdo[5] = {4, 0, 0, 0, 0};
  uint8_t outgoing[29] = {};
  uint8_t incoming[29] = {};
  uint8_t staged[29] = {};      // a reply waiting for ACCEPT_VDM
  bool staged_valid = false;
  uint8_t auto_fixed[5] = {4}, auto_batt[5] = {4};

  // The cable and the partner as the chip sees them.
  bool attached = false;
  int dr = DR_NONE;
  int pdo_no = 0;               // the PDO of the contract, 1-based
  uint32_t rdo = 0;
  bool nin_mode = false, dp_entered = false, dp_configured = false;
  uint8_t dp_pin = 0;
  bool hpd = false;
  bool hub_on = false;
  uint8_t led[4] = {};
};

Bm92 chip;

DockKind kind() {
  return g_state ? (DockKind)g_state->dock_kind.load() : DOCK_NONE;
}
bool tv_present() { return g_state && g_state->dock_tv.load(); }
bool has_dp() { return kind() == DOCK_NINTENDO || kind() == DOCK_THIRD_PARTY; }

std::vector<Pdo> partner_pdos() {
  switch (kind()) {
  case DOCK_NINTENDO:
    // The dock's identity profile, then its one real profile: what L4T
    // looks for (DOCK_ID 5 V 0.5 A, then 15 V 2.6..3 A) and what hwtest saw.
    return {{5000, 500, true}, {15000, 3000, false}};
  case DOCK_THIRD_PARTY:
    return {{5000, 2000, true}, {9000, 2000, false}, {15000, 2600, false},
            {20000, 4700, false}};
  case DOCK_CHARGER:
    return {{5000, 1500, false}, {15000, 2600, false}};
  default:
    if (g_state && g_state->usb_pd_inserted.load())
      return {{g_state->usb_pd_voltage_mv.load(), g_state->usb_pd_amperage_ma.load(), false}};
    return {};
  }
}

bool cable_in() { return kind() != DOCK_NONE || (g_state && g_state->usb_pd_inserted.load()); }

uint32_t pdo_word(const Pdo &p) {
  uint32_t info = p.drd ? ((1u << 5) | (1u << 6) | (1u << 7)) : 0;
  return ((p.ma / 10) & 0x3FF) | (((p.mv / 50) & 0x3FF) << 10) | (info << 20);
}

uint16_t status1() {
  uint16_t s = 0;
  if (chip.attached)
    s |= ST1_INSERT | ST1_VSAFE | ST1_SPDSNK;
  s |= (uint16_t)((chip.dr & 3) << 8);
  s |= (uint16_t)((chip.lastcmd & 7) << 4);
  if (chip.busy)
    s |= ST1_CMD_BUSY;
  return s;
}

uint16_t dp_status() {
  if (!chip.dp_entered)
    return 0;
  uint16_t s = (1u << 15) | (1u << 14);                    // DP_EN, INSERT
  if (chip.dp_configured)
    s |= (1u << 1) | (chip.dp_pin == 0x04 ? 0x0C : 0x18);  // configured, pins
  if (chip.hpd)
    s |= 1u << 7;                                         // SIGNAL_ON
  return s;
}

void raise(uint16_t bits) {
  chip.alert |= bits;
}

void put_le(uint8_t *p, uint32_t v, int n) {
  for (int i = 0; i < n; i++)
    p[i] = (uint8_t)(v >> (8 * i));
}

// A structured VDM reply as INCOMING_VDM holds it: length, header, VDOs.
void stage_reply(uint8_t cmd_ack, uint8_t hdr2, uint16_t svid,
                 const std::vector<uint32_t> &vdos) {
  memset(chip.staged, 0, sizeof(chip.staged));
  chip.staged[0] = (uint8_t)(4 + 4 * vdos.size());
  chip.staged[1] = cmd_ack;
  chip.staged[2] = hdr2;
  chip.staged[3] = (uint8_t)svid;
  chip.staged[4] = (uint8_t)(svid >> 8);
  for (size_t i = 0; i < vdos.size() && 5 + 4 * i + 4 <= sizeof(chip.staged); i++)
    put_le(&chip.staged[5 + 4 * i], vdos[i], 4);
  chip.staged_valid = true;
}

void hpd_update(bool level) {
  if (chip.hpd == level)
    return;
  chip.hpd = level;
  DLOG("HPD %s\n", level ? "high (TV on the dock)" : "low");
  if (chip.dp_alert_en)
    raise(AL_DP_EVENT);
}

// Cable on: the chip comes up on 5 V and reports the partner's offer.
void attach() {
  chip.attached = true;
  chip.dr = DR_UFP;
  chip.pdo_no = 1;
  chip.lastcmd = LASTCMD_COMPLETE;
  raise(AL_PLUGPULL | AL_CONTRACT | AL_PDO);
  DLOG("cable in: %s, %zu PDO(s)\n",
       kind() == DOCK_NINTENDO ? "Nintendo dock 057E:2003"
       : kind() == DOCK_THIRD_PARTY ? "adapter 1D5C:7102"
       : kind() == DOCK_CHARGER ? "Nintendo charger 057E:2004" : "power source",
       partner_pdos().size());
}

void chip_reset(bool from_pin) {
  chip_gen++;                    // pending events die with the old state
  hpd_update(false);
  bool held = chip.reset_held;
  chip = Bm92();
  chip.reset_held = held;
  uint64_t boot = from_pin ? 100000 : 30000;
  chip.answers_from = now_us() + (from_pin ? 100000 : 0);
  if (cable_in())
    after_us(boot, [] { attach(); });
}

// ---- the partner's answers to VDMs ----------------------------------------

void partner_vdm(const uint8_t *msg, uint32_t len) {
  if (len < 4)
    return;
  uint8_t cmd = msg[0], type = msg[1];
  uint16_t svid = (uint16_t)(msg[2] | (msg[3] << 8));
  bool structured = type & 0x80;
  DockKind k = kind();
  if (!structured) {
    // Nintendo dock vendor messages: [REQ, 0, 7E 05, rw, src, cmd, 0, ...]
    if (k != DOCK_NINTENDO || svid != 0x057E || len < 8 || !chip.nin_mode)
      return;
    uint8_t ncmd = msg[6];
    memset(chip.staged, 0, sizeof(chip.staged));
    uint8_t data[4] = {};
    int size = 8;
    switch (ncmd) {
    case 0x16:                    // device state: what hwtest read
      data[0] = 0x00; data[1] = 0x00; data[2] = 0x1D; data[3] = 0x00;
      size = 12;
      break;
    case 0x01:                    // LED control
      memcpy(chip.led, &msg[8], len >= 12 ? 4 : 0);
      DLOG("dock LED set %02X %02X %02X %02X\n", msg[8], msg[9], msg[10], msg[11]);
      size = 12;
      break;
    case 0x20:                    // USB hub control
      chip.hub_on = msg[4] == 1;
      DLOG("dock USB hub %s\n", chip.hub_on ? "on" : "off");
      break;
    case 0x1C: case 0x1E:         // DP signal disable, hub reset
      break;
    default:
      return;
    }
    chip.staged[0] = (uint8_t)size;
    chip.staged[1] = 0x40;        // REPLY
    chip.staged[2] = 0x00;
    chip.staged[3] = 0x7E;
    chip.staged[4] = 0x05;
    chip.staged[5] = 0x00;        // not busy
    chip.staged[6] = 0x02;        // from the dock
    chip.staged[7] = (uint8_t)(ncmd + 1);
    chip.staged[8] = 0x00;
    if (size == 12)
      memcpy(&chip.staged[9], data, 4);
    chip.staged_valid = true;
    return;
  }
  uint8_t ack = (uint8_t)(0x40 | (cmd & 0x1F));
  switch (cmd & 0x1F) {
  case 0x01: {                    // Discover Identity
    uint16_t vid = k == DOCK_THIRD_PARTY ? 0x1D5C : 0x057E;
    uint16_t pid = k == DOCK_THIRD_PARTY ? 0x7102 : k == DOCK_CHARGER ? 0x2004 : 0x2003;
    uint32_t type_ama = k == DOCK_CHARGER ? 0 : 5;
    uint32_t idh = vid | (1u << 26) | (type_ama << 27) | (1u << 30);
    stage_reply(ack, 0x80, 0xFF00, {idh, 0x00000000, (uint32_t)pid << 16 | 0x0100, 0x00000000});
    break;
  }
  case 0x02:                      // Discover SVIDs
    if (k == DOCK_THIRD_PARTY)
      stage_reply(ack, 0x80, 0xFF00, {0x0000FF01});
    else if (k == DOCK_NINTENDO)
      stage_reply(ack, 0x80, 0xFF00, {0xFF01057E});
    else
      stage_reply(ack, 0x80, 0xFF00, {0x0000057E});
    break;
  case 0x03:                      // Discover Modes
    if (svid == 0xFF01 && has_dp())
      // The Nintendo dock reports a plug but puts its pins in the receptacle
    // field (L4T: "dock reports Plug but uses Receptacle"); both are set.
    stage_reply(ack, 0x80, 0xFF01, {k == DOCK_NINTENDO ? 0x000C0C05u : 0x000C0045u});
    else
      stage_reply((uint8_t)(0x80 | (cmd & 0x1F)), 0x80, svid, {});
    break;
  case 0x04:                      // Enter Mode
    if (svid == 0x057E && k == DOCK_NINTENDO) {
      chip.nin_mode = true;
      DLOG("Nintendo alt mode entered\n");
      stage_reply(ack, 0x81, 0x057E, {});
    } else {
      stage_reply((uint8_t)(0x80 | 0x04), (uint8_t)(0x80 | (type & 7)), svid, {});
    }
    break;
  case 0x05:                      // Exit Mode
    if (svid == 0x057E)
      chip.nin_mode = false;
    stage_reply(ack, type, svid, {});
    break;
  default:
    stage_reply((uint8_t)(0x80 | (cmd & 0x1F)), type, svid, {});
    break;
  }
}

// ---- commands --------------------------------------------------------------

void done(int lastcmd, uint16_t extra = 0) {
  chip.busy = false;
  chip.lastcmd = lastcmd;
  raise(AL_CMD_DONE | extra);
}

void command(uint16_t cmd) {
  if (!chip.attached && cmd != CMD_SYS_RESET) {
    after_us(2000, [] { done(LASTCMD_INVALID); });
    return;
  }
  chip.busy = true;
  switch (cmd) {
  case CMD_SYS_RESET:
    DLOG("SYS_RESET\n");
    chip_reset(false);
    break;
  case CMD_SEND_RDO: {
    uint32_t rdo = chip.set_rdo[1] | chip.set_rdo[2] << 8 | chip.set_rdo[3] << 16 |
                   (uint32_t)chip.set_rdo[4] << 24;
    int obj = (int)(rdo >> 28);
    auto pdos = partner_pdos();
    if (obj < 1 || obj > (int)pdos.size()) {
      after_us(20000, [] { done(LASTCMD_REJECTED); });
      break;
    }
    after_us(25000, [obj, rdo] {
      chip.pdo_no = obj;
      chip.rdo = rdo;
      auto p = partner_pdos()[obj - 1];
      DLOG("contract: PDO %d, %u mV %u mA, drawing up to %u mA\n", obj, p.mv, p.ma,
           ((rdo >> 10) & 0x3FF) * 10);
      done(LASTCMD_COMPLETE, AL_CONTRACT);
    });
    break;
  }
  case CMD_PS_RDY:
  case CMD_GET_SRC_CAP:
  case CMD_START_HPD:
  case CMD_STOP_HPD:
    after_us(3000, [] { done(LASTCMD_COMPLETE); });
    break;
  case CMD_DR_SWAP:
    after_us(10000, [] {
      bool drd = !partner_pdos().empty() && partner_pdos()[0].drd;
      if (!drd) {
        done(LASTCMD_REJECTED);
        return;
      }
      chip.dr = chip.dr == DR_UFP ? DR_DFP : DR_UFP;
      DLOG("data role swap: console is now %s\n", chip.dr == DR_DFP ? "the host (DFP)" : "a device (UFP)");
      done(LASTCMD_COMPLETE, AL_DR_SWAP);
    });
    break;
  case CMD_SEND_VDM: {
    uint8_t len = chip.outgoing[0];
    std::vector<uint8_t> msg(chip.outgoing + 1, chip.outgoing + 1 + (len > 28 ? 28 : len));
    after_us(2000, [] { done(LASTCMD_COMPLETE); });
    after_us(8000, [msg] {
      partner_vdm(msg.data(), (uint32_t)msg.size());
      if (chip.staged_valid)
        raise(AL_VDM_RECEIVED);
    });
    break;
  }
  case CMD_ACCEPT_VDM:
    after_us(2000, [] {
      if (chip.staged_valid) {
        memcpy(chip.incoming, chip.staged, sizeof(chip.incoming));
        chip.staged_valid = false;
      }
      done(LASTCMD_COMPLETE);
    });
    break;
  case CMD_DP_ENTER:
    // The chip runs DP discovery itself and leaves the partner's DP
    // capabilities VDO in INCOMING_VDM, without a VDM header.
    after_us(60000, [] {
      if (!has_dp() || chip.dr != DR_DFP) {
        done(LASTCMD_REJECTED);
        return;
      }
      chip.dp_entered = true;
      memset(chip.incoming, 0, sizeof(chip.incoming));
      chip.incoming[0] = 4;
      uint32_t caps = kind() == DOCK_NINTENDO ? 0x000C0C05u : 0x000C0045u;
      put_le(&chip.incoming[1], caps, 4);
      DLOG("DisplayPort alt mode entered, caps %08X\n", caps);
      done(LASTCMD_COMPLETE);
    });
    break;
  case CMD_DP_CFG_HPD: {
    uint8_t b0 = chip.outgoing[1], b1 = chip.outgoing[2], b2 = chip.outgoing[3];
    after_us(40000, [b0, b1, b2] {
      // The Nintendo dock refused a configure VDO without "UFP_U as UFP_D"
      // (bit 1); it ignores where the pins are. A plain adapter wants them
      // in the byte its capabilities named (receptacle here).
      uint8_t pin = kind() == DOCK_NINTENDO ? (b1 | b2) : b1;
      if (!chip.dp_entered || !(b0 & 0x02) || !(b0 & 0x04) || !(pin & 0x0C)) {
        DLOG("DP configure refused (VDO %02X %02X %02X)\n", b0, b1, b2);
        done(LASTCMD_REJECTED);
        return;
      }
      chip.dp_configured = true;
      chip.dp_pin = (pin & 0x08) ? 0x08 : 0x04;
      DLOG("DisplayPort configured, pin %c\n", chip.dp_pin == 0x08 ? 'D' : 'C');
      done(LASTCMD_COMPLETE);
      if (tv_present())
        after_us(150000, [] { if (chip.dp_configured) hpd_update(true); });
    });
    break;
  }
  case CMD_DP_STOP:
  case CMD_STOP_HPD_EXIT_DP:
    after_us(10000, [] {
      chip.dp_configured = chip.dp_entered = false;
      hpd_update(false);
      done(LASTCMD_COMPLETE);
    });
    break;
  case CMD_HARD_RST:
  case CMD_SOFT_RST:
    after_us(30000, [] {
      chip.dp_configured = chip.dp_entered = chip.nin_mode = false;
      hpd_update(false);
      chip.dr = DR_UFP;
      chip.pdo_no = 1;
      done(LASTCMD_COMPLETE, AL_CONTRACT | AL_PDO);
    });
    break;
  default:
    after_us(2000, [] { done(LASTCMD_INVALID); });
    break;
  }
}

uint32_t pdo_bytes(uint8_t *out, uint32_t cap) {
  auto pdos = partner_pdos();
  uint32_t n = 0;
  out[n++] = (uint8_t)(4 * pdos.size());
  for (auto &p : pdos)
    if (n + 4 <= cap) {
      put_le(&out[n], pdo_word(p), 4);
      n += 4;
    }
  return n;
}

// ==========================================================================
// DisplayPort sink: the dock's converter and the TV behind it
// ==========================================================================

uint8_t dpcd[0x800];
uint8_t edid[256];
uint32_t edid_ptr = 0;
uint32_t aux_i2c_addr = 0;
uint64_t crc_frames_base = 0;

void edid_build() {
  static const uint8_t base[128] = {
      0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x15, 0xB5, 0x01, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x01, 0x1E, 0x01, 0x03, 0x80, 0x50, 0x2D, 0x78,
      0x0A, 0xEE, 0x91, 0xA3, 0x54, 0x4C, 0x99, 0x26, 0x0F, 0x50, 0x54, 0x21,
      0x08, 0x00, 0x81, 0xC0, 0xD1, 0xC0, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
      0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
      // 1920x1080 at 60 Hz, 148.5 MHz
      0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C, 0x45, 0x00,
      0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E,
      // 1280x720 at 60 Hz, 74.25 MHz
      0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00,
      0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E,
      // name "EMULATED TV"
      0x00, 0x00, 0x00, 0xFC, 0x00, 'E', 'M', 'U', 'L', 'A', 'T', 'E', 'D',
      ' ', 'T', 'V', 0x0A, 0x20,
      // range limits 24-75 Hz, 15-81 kHz, 170 MHz
      0x00, 0x00, 0x00, 0xFD, 0x00, 0x18, 0x4B, 0x0F, 0x51, 0x11, 0x00, 0x0A,
      0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
      0x01, 0x00};
  memcpy(edid, base, 128);
  // CEA-861 extension: 1080p60 (native), 720p60, 480p, 640x480; HDMI VSDB.
  static const uint8_t cea[] = {0x02, 0x03, 0x0F, 0x00, 0x44, 0x90, 0x04, 0x03,
                                0x01, 0x65, 0x03, 0x0C, 0x00, 0x10, 0x00};
  memset(edid + 128, 0, 128);
  memcpy(edid + 128, cea, sizeof(cea));
  for (int b = 0; b < 2; b++) {
    uint8_t sum = 0;
    for (int i = 0; i < 127; i++)
      sum += edid[b * 128 + i];
    edid[b * 128 + 127] = (uint8_t)(0x100 - sum);
  }
}

void dpcd_build() {
  memset(dpcd, 0, sizeof(dpcd));
  bool nin = kind() == DOCK_NINTENDO;
  dpcd[0x000] = 0x12;                       // DPCD 1.2
  dpcd[0x001] = 0x14;                       // HBR2
  dpcd[0x002] = (uint8_t)(0xC0 | (nin ? 4 : 2)); // enhanced framing, TPS3, lanes
  dpcd[0x003] = 0x01;                       // 0.5% down-spread
  dpcd[0x004] = 0x01;
  dpcd[0x005] = 0x05;                       // a downstream port, DVI/HDMI
  dpcd[0x006] = 0x01;                       // 8b/10b
  dpcd[0x007] = 0x01;                       // one downstream port
  dpcd[0x008] = 0x02;
  dpcd[0x00A] = 0x06;
  dpcd[0x00E] = 0x00;                       // 100/400 us training waits
  dpcd[0x080] = 0x0B;                       // DFP0: HDMI, HPD aware
  dpcd[0x101] = 0x00;
  dpcd[0x108] = 0x01;
  dpcd[0x246] = 0x20;                       // TEST_CRC_SUPPORTED
  dpcd[0x500] = 0x00; dpcd[0x501] = 0xE0; dpcd[0x502] = 0x4C;   // Realtek
  memcpy(&dpcd[0x503], nin ? "Dp1.2" : "Dp1.4", 5);
  dpcd[0x600] = 0x01;
}

bool sink_reachable() {
  return has_dp() && chip.dp_configured && tv_present() && chip.hpd;
}

// ==========================================================================
// SOR1 and the link
// ==========================================================================

constexpr uint64_t SOR1_BASE_ADDR = 0x54580000;
uint32_t sor[0x400];                        // word registers
bool lanes_powered = false;                 // the lane sequencer's last PU
uint32_t sor_super_state1 = 0;              // committed by SUPER_STATE0.UPDATE
uint32_t sor_state1 = 0;                    // committed by STATE0.UPDATE
uint32_t lane_status[4] = {};               // CR(1) EQ(2) SYMLOCK(4) per lane

enum : uint32_t {
  S_SUPER_STATE0 = 0x01, S_SUPER_STATE1 = 0x02, S_STATE0 = 0x03, S_STATE1 = 0x04,
  S_CRC_CNTRL = 0x11, S_CLK_CNTRL = 0x13, S_PWR = 0x15, S_TEST = 0x16,
  S_PLL0 = 0x17, S_PLL1 = 0x18, S_PLL2 = 0x19, S_PLL3 = 0x1A, S_CRCA = 0x1D,
  S_CRCB = 0x1E, S_SEQ_CTL = 0x20, S_LANE_SEQ_CTL = 0x21, S_XBAR_CTRL = 0x4A,
  S_LINKCTL0 = 0x4C, S_DC0 = 0x4E, S_PR0 = 0x52, S_DP_CONFIG0 = 0x58,
  S_PADCTL0 = 0x5C, S_SPARE0 = 0x60, S_TPG = 0x6D,
};

uint32_t car(uint32_t off) { return mmio_bus_read(g_state, CLK_RST_BASE + off, 4); }

// The physical pad that carries logical lane `l` (see dock.h): through the
// crossbar unless it is in bypass. In bypass, and with XSEL as the identity,
// logical lane n leaves on the pad the TRM names "DP lane n" - pad 2 for 0.
int pad_of_lane(uint32_t l) {
  static const int phys_of_index[4] = {2, 1, 0, 3};
  uint32_t x = sor[S_XBAR_CTRL];
  if (x & 1)
    return phys_of_index[l & 3];
  for (int n = 0; n < 4; n++)
    if (((x >> (2 + 3 * n)) & 7) == l)
      return phys_of_index[n];
  return -1;
}

int lanes_of(uint32_t lanecount_field) {
  switch (lanecount_field) {
  case 1: return 1;
  case 3: return 2;
  case 15: return 4;
  default: return 0;
  }
}

// Why connector lane j would not carry a signal, or nullptr. The Switch
// wires DP lane j to physical pad j.
const char *lane_fault(uint32_t j) {
  if (!(car(0x280) & (1u << 23)))         return "SOR1 clock off";
  if (car(0x28C) & (1u << 23))            return "SOR1 held in reset";
  if (!(car(0x410) & (1u << 14)))         return "SOR1 not on the brick (macro) clock";
  if (!(car(0x590) & (1u << 27)))         return "PLLDP not locked";
  if (sor[S_PLL0] & 5)                    return "SOR macro PLL powered down";
  if (sor[S_PLL2] & (3u << 22))           return "SOR bandgap or port powered down";
  if (!lanes_powered)                     return "lane sequencer never powered the lanes up";
  if (!(sor[S_LINKCTL0] & 1))             return "DP link not enabled (LINKCTL)";
  if ((int)j >= lanes_of((sor[S_LINKCTL0] >> 16) & 0x1F)) return "lane beyond LINKCTL's lane count";
  int pad = pad_of_lane(j);
  if (pad != (int)j)                      return "crossbar sends this lane to the wrong pad";
  if (!(sor[S_PADCTL0] & (1u << pad)))    return "the lane's pad is powered down (PADCTL)";
  if (!((sor[S_DC0] >> (8 * pad)) & 0xFF)) return "no drive current on the lane's pad";
  uint32_t speed = (sor[S_CLK_CNTRL] >> 2) & 0x1F;
  if (speed != dpcd[0x100])               return "SOR link speed differs from LINK_BW_SET";
  return nullptr;
}

// The pixel clock display head B runs at: CLK_SOURCE_DISP2 (0x13C) picks
// the source, PLLD2 (BASE 0x4B8, SS_CFG 0x570, SS_CTRL2 0x578) makes it.
// Fvco = Fref / M * (N + 0.5 + SDM_DIN / 8192) with the sigma-delta on.
// Fref is the 38.4 MHz oscillator after OSC_CTRL's PLL_REF_DIV (27:26),
// /1 on the Switch: coreboot's 38.4 MHz table runs PLLDP at M = 2 to stay
// inside its 12..38 MHz comparison window, and SOR1 trained at 2.7 Gbps on
// a PLLDP set up that way. The post divider is the kernel's "qlin" map
// (clk-tegra210.c pll_qlin_pdiv_to_hw): code 8 is /10, code 9 is /12.
uint64_t head_b_pclk_khz() {
  static const uint32_t qlin[17] = {1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 15, 16, 18, 20, 24, 30, 32};
  uint32_t src = car(0x13C) >> 29;
  if (src != 5)
    return 0;                                // only PLLD2_OUT0 is modelled
  uint32_t base = car(0x4B8);
  if (!(base & (1u << 30)))
    return 0;
  uint32_t m = base & 0xFF, n = (base >> 8) & 0xFF, p = (base >> 19) & 0x1F;
  if (!m || p > 16)
    return 0;
  uint64_t fref_hz = 38400000ULL >> ((car(0x050) >> 26) & 3);
  uint64_t num = (uint64_t)n * 8192;
  if (car(0x570) & (1u << 31))               // EN_SDM
    num += 4096 + (int16_t)(car(0x578) & 0xFFFF);
  uint64_t vco_hz = fref_hz * num / 8192 / m;
  uint32_t div = car(0x13C) & 0xFF;          // DISP2_CLK_DIVISOR: /(N/2+1)
  uint64_t hz = vco_hz / qlin[p] * 2 / (div + 2);
  return hz / 1000;
}

uint32_t lane_pattern(uint32_t j) { return (sor[S_TPG] >> (8 * j)) & 0xF; }

uint32_t dpcd_lanes() { return dpcd[0x101] & 0x1F; }

// Re-evaluate the sink's per-lane status after anything the link depends on.
void link_eval() {
  uint32_t n = dpcd_lanes();
  uint32_t want = dpcd[0x102] & 3;
  for (uint32_t j = 0; j < 4; j++) {
    if (j >= n || !sink_reachable() || lane_fault(j)) {
      lane_status[j] = 0;
      continue;
    }
    uint32_t sent = lane_pattern(j);
    if (want == 1 && sent == 1)
      lane_status[j] |= 1;
    else if ((want == 2 || want == 3) && sent == want && (lane_status[j] & 1))
      lane_status[j] |= 6;
    else if (want == 0 && sent == 0 && (sor[S_TPG] >> (8 * j) & 0x10)) {
      // normal operation: the lane keeps what it trained to
    } else if (want == 0 && sent != 0) {
      lane_status[j] &= 1;
    }
  }
}

bool link_trained(uint32_t *lanes_out = nullptr) {
  uint32_t n = dpcd_lanes();
  if (lanes_out)
    *lanes_out = n;
  if (!n || !sink_reachable())
    return false;
  link_eval();
  for (uint32_t j = 0; j < n; j++)
    if ((lane_status[j] & 7) != 7)
      return false;
  return true;
}

uint8_t dpcd_read_byte(uint32_t a) {
  link_eval();
  switch (a) {
  case 0x200: return tv_present() ? 0x41 : 0x00;           // one sink, CP ready
  case 0x202: return (uint8_t)(lane_status[0] | lane_status[1] << 4);
  case 0x203: return (uint8_t)(lane_status[2] | lane_status[3] << 4);
  case 0x204: {
    uint32_t n = dpcd_lanes();
    bool all = n > 0;
    for (uint32_t j = 0; j < n; j++)
      all &= (lane_status[j] & 7) == 7;
    return (uint8_t)((all ? 1 : 0) | 0x80);
  }
  case 0x205: return link_trained() && (dpcd[0x102] & 3) == 0 ? 1 : 0;
  case 0x206: case 0x207: return 0;                       // keep the levels
  case 0x246: {
    TvState t;
    dock_tv_state(g_state, &t);
    uint8_t c = 0;
    if (t.video && (dpcd[0x270] & 1))
      c = (uint8_t)(((now_us() - crc_frames_base) / 16667) & 0xF);
    return (uint8_t)(0x20 | c);
  }
  default:
    if (a >= 0x240 && a < 0x246)
      return (dpcd[0x270] & 1) ? 0x5A : 0;
    return a < sizeof(dpcd) ? dpcd[a] : 0;
  }
}

void dpcd_write_byte(uint32_t a, uint8_t v) {
  if (a >= sizeof(dpcd))
    return;
  if (a == 0x270 && (v & 1) && !(dpcd[0x270] & 1))
    crc_frames_base = now_us();
  if (a == 0x100 || a == 0x101) {
    for (auto &s : lane_status)
      s = 0;
  }
  dpcd[a] = v;
  if (a == 0x102 && (v & 3) == 1)
    for (auto &s : lane_status)
      s = 0;
}

// ==========================================================================
// DPAUX1
// ==========================================================================

constexpr uint64_t DPAUX1_BASE_ADDR = 0x54040000;
uint32_t aux[0x80];

enum : uint32_t {
  A_INTR_EN = 0x01, A_INTR = 0x05, A_DATA_WR = 0x09, A_DATA_RD = 0x19,
  A_ADDR = 0x29, A_CTL = 0x2D, A_STAT = 0x31, A_HPD_CONFIG = 0x3D,
  A_HPD_IRQ_CONFIG = 0x41, A_PADCTL = 0x49, A_SPARE = 0x4D,
};

bool hpd_pad_sfio() {
  // GPIO CC1: bank 7, port 0 in bank. GPIO_CNF bit 1 clear = SFIO.
  return !(mmio_bus_read(g_state, GPIO_BASE + 0x700, 4) & (1u << 1));
}

bool dpaux_clocked() {
  return (car(0x298) & (1u << 15)) && !(car(0x2A4) & (1u << 15));
}

uint32_t aux_hpd_bit() {
  return (chip.hpd && hpd_pad_sfio() && dpaux_clocked()) ? (1u << 28) : 0;
}

void aux_transact() {
  uint32_t ctl = aux[A_CTL];
  uint32_t cmd = (ctl >> 12) & 0xF;
  uint32_t len = (ctl & 0xFF) + 1;
  uint32_t addr = aux[A_ADDR] & 0xFFFFF;
  bool address_only = ctl & (1u << 8);
  uint8_t wr[16], rd[16] = {};
  for (int i = 0; i < 16; i++)
    wr[i] = (uint8_t)(aux[A_DATA_WR + 4 * (i / 4)] >> (8 * (i % 4)));
  uint32_t stat = 0, got = 0;
  bool pad_up = !(aux[A_SPARE] & 1);
  if (!dpaux_clocked() || !pad_up || !sink_reachable()) {
    stat = 1u << 8;                          // TIMEOUT_ERROR
  } else if (cmd == 8) {                     // AUXWR
    for (uint32_t i = 0; i < len && i < 16; i++)
      dpcd_write_byte(addr + i, wr[i]);
    stat = 0;
  } else if (cmd == 9) {                     // AUXRD
    for (uint32_t i = 0; i < len && i < 16; i++)
      rd[i] = dpcd_read_byte(addr + i);
    got = len > 16 ? 16 : len;
  } else {                                   // I2C over AUX
    bool write = cmd == 0 || cmd == 4;
    bool read = cmd == 1 || cmd == 5;
    if (addr != 0x50 && addr != 0x30) {
      stat = 4u << 16;                       // I2C NACK
    } else if (address_only) {
      aux_i2c_addr = addr;
    } else if (write) {
      if (addr == 0x50)
        edid_ptr = wr[0];
      else
        edid_ptr = (uint32_t)wr[0] * 256;    // E-DDC segment
    } else if (read && addr == 0x50) {
      for (uint32_t i = 0; i < len && i < 16; i++)
        rd[i] = edid[(edid_ptr + i) & 0xFF];
      edid_ptr += len;
      got = len > 16 ? 16 : len;
    }
  }
  for (int w = 0; w < 4; w++) {
    uint32_t v = 0;
    for (int b = 0; b < 4; b++)
      v |= (uint32_t)rd[w * 4 + b] << (8 * b);
    aux[A_DATA_RD + 4 * w] = v;
  }
  aux[A_STAT] = stat | got;
  aux[A_CTL] &= ~(1u << 16);
  aux[A_INTR] |= 1u << 3;                    // TX_DONE
  trace("aux", cmd << 20 | (addr & 0xFFFF), len | stat);
}

// ==========================================================================

void tick_events() {
  for (;;) {
    size_t best = events.size();
    for (size_t i = 0; i < events.size(); i++)
      if (events[i].at <= now_us() && (best == events.size() || events[i].at < events[best].at))
        best = i;
    if (best == events.size())
      return;
    Event e = std::move(events[best]);
    events.erase(events.begin() + (long)best);
    if (e.gen == chip_gen)
      e.fn();
  }
}

bool last_cable = false;
DockKind last_kind = DOCK_NONE;

} // namespace

// ==========================================================================

void dock_reset(EmuState *state) {
  g_state = state;
  events.clear();
  chip = Bm92();
  memset(sor, 0, sizeof(sor));
  memset(aux, 0, sizeof(aux));
  sor[S_XBAR_CTRL] = 0x8D111A23;           // TRM reset: bypass + link swap
  sor[S_PLL0] = 0x5;                        // PLL and VCO powered down
  sor[S_PLL2] = 3u << 22;
  sor[S_SPARE0] = 0x2;
  sor[S_STATE1] = 0x40;
  aux[A_SPARE] = 0;                         // DPAUX pad powered at reset
  lanes_powered = false;
  sor_super_state1 = sor_state1 = 0;
  memset(lane_status, 0, sizeof(lane_status));
  dpcd_build();
  edid_build();
  last_cable = cable_in();
  last_kind = kind();
  chip_reset(false);
  chip.answers_from = 0;
}

void dock_tick(EmuState *state) {
  g_state = state;
  if (kind() != last_kind || cable_in() != last_cable) {
    // The config window (or ini) swapped what is on the cable.
    bool was = last_cable;
    last_kind = kind();
    last_cable = cable_in();
    dpcd_build();
    if (!last_cable && was) {
      chip_gen++;
      chip.attached = false;
      chip.dr = DR_NONE;
      chip.dp_entered = chip.dp_configured = chip.nin_mode = false;
      hpd_update(false);
      chip.alert |= AL_PLUGPULL;
      DLOG("cable out\n");
    } else if (last_cable && !chip.reset_held) {
      after_us(30000, [] { attach(); });     // debounce, then the chip reports it
    }
  }
  if (!events.empty())
    tick_events();
}

bool bm92_present(EmuState *state) {
  g_state = state;
  dock_tick(state);
  return !chip.reset_held && now_us() >= chip.answers_from;
}

void bm92_reset_line(EmuState *state, bool high) {
  g_state = state;
  if (high == chip.reset_held)
    return;
  chip.reset_held = high;
  if (high) {
    chip_gen++;
    events.clear();
    hpd_update(false);
    chip = Bm92();
    chip.reset_held = true;
    DLOG("PV5 high: PD controller held in reset\n");
  } else {
    DLOG("PV5 low: PD controller boots\n");
    chip_reset(true);
  }
}

void bm92_write(EmuState *state, const uint8_t *buf, uint32_t n) {
  g_state = state;
  dock_tick(state);
  if (!n || !bm92_present(state))
    return;
  uint8_t reg = buf[0];
  const uint8_t *d = buf + 1;
  uint32_t m = n - 1;
  auto w16 = [&](uint16_t &r) { if (m >= 2) r = (uint16_t)(d[0] | d[1] << 8); };
  auto blk = [&](uint8_t *r, uint32_t cap) {
    memset(r, 0, cap);
    for (uint32_t i = 0; i < m && i < cap; i++)
      r[i] = d[i];
  };
  switch (reg) {
  case REG_COMMAND:
    if (m >= 2)
      command((uint16_t)(d[0] | d[1] << 8));
    break;
  case REG_CONFIG1:      w16(chip.config1); break;
  case REG_CONFIG2:      w16(chip.config2); break;
  case REG_DP_ALERT_EN:  w16(chip.dp_alert_en); break;
  case REG_VENDOR_CFG:   w16(chip.vendor_cfg); break;
  case REG_SYS_CONFIG1:  w16(chip.sys_config1); break;
  case REG_SYS_CONFIG2:  w16(chip.sys_config2); break;
  case REG_SYS_CONFIG3:  w16(chip.sys_config3); break;
  case REG_ALERT_ENABLE: w16(chip.alert_enable); break;
  case REG_SET_RDO:      if (m) blk(chip.set_rdo, sizeof(chip.set_rdo)); break;
  case REG_OUTGOING_VDM: if (m) blk(chip.outgoing, sizeof(chip.outgoing)); break;
  case REG_AUTO_NGT_FIXED: if (m) blk(chip.auto_fixed, sizeof(chip.auto_fixed)); break;
  case REG_AUTO_NGT_BATT:  if (m) blk(chip.auto_batt, sizeof(chip.auto_batt)); break;
  default: break;
  }
}

void bm92_read(EmuState *state, uint8_t reg, uint8_t *out, uint32_t n) {
  g_state = state;
  dock_tick(state);
  memset(out, 0, n);
  uint8_t tmp[32] = {};
  uint32_t len = 2;
  auto v16 = [&](uint16_t v) { tmp[0] = (uint8_t)v; tmp[1] = (uint8_t)(v >> 8); };
  switch (reg) {
  case REG_ALERT:
    v16(chip.alert);
    chip.alert = 0;                     // clear on read (L4T relies on it)
    break;
  case REG_STATUS1:      v16(status1()); break;
  case REG_STATUS2:      v16(0); break;
  case REG_CONFIG1:      v16(chip.config1); break;
  case REG_DEV_CAPS:     v16(chip.dev_caps); break;
  case REG_CONFIG2:      v16(chip.config2); break;
  case REG_DP_STATUS:    v16(dp_status()); break;
  case REG_DP_ALERT_EN:  v16(chip.dp_alert_en); break;
  case REG_VENDOR_CFG:   v16(chip.vendor_cfg); break;
  case REG_1C:           v16(chip.dp_entered ? 0x0C05 : 0); break;
  case REG_1D:           v16(0); break;
  case REG_SYS_CONFIG1:  v16(chip.sys_config1); break;
  case REG_SYS_CONFIG2:  v16(chip.sys_config2); break;
  case REG_SYS_CONFIG3:  v16(chip.sys_config3); break;
  case REG_ALERT_ENABLE: v16(chip.alert_enable); break;
  case REG_FW_TYPE:      v16(chip.fw_type); break;
  case REG_FW_REV:       v16(chip.fw_rev); break;
  case REG_MAN_ID:       v16(chip.man_id); break;
  case REG_DEV_ID:       v16(chip.dev_id); break;
  case REG_REV_ID:       v16(chip.rev_id); break;
  case REG_PDOS_SRC:     len = chip.attached ? pdo_bytes(tmp, sizeof(tmp)) : 1; break;
  case REG_CURRENT_PDO: {
    auto pdos = partner_pdos();
    tmp[0] = 4;
    if (chip.attached && chip.pdo_no >= 1 && chip.pdo_no <= (int)pdos.size())
      put_le(&tmp[1], pdo_word(pdos[chip.pdo_no - 1]), 4);
    len = 5;
    break;
  }
  case REG_CURRENT_RDO:
    tmp[0] = 4;
    put_le(&tmp[1], chip.rdo, 4);
    len = 5;
    break;
  case REG_SET_RDO:      memcpy(tmp, chip.set_rdo, 5); len = 5; break;
  case REG_INCOMING_VDM: memcpy(tmp, chip.incoming, 29); len = 29; break;
  case REG_OUTGOING_VDM: memcpy(tmp, chip.outgoing, 29); len = 29; break;
  case REG_AUTO_NGT_FIXED: memcpy(tmp, chip.auto_fixed, 5); len = 5; break;
  case REG_AUTO_NGT_BATT:  memcpy(tmp, chip.auto_batt, 5); len = 5; break;
  default: len = 0; break;
  }
  for (uint32_t i = 0; i < n && i < len && i < sizeof(tmp); i++)
    out[i] = tmp[i];
}

bool bm92_alert_line(EmuState *state) {
  g_state = state;
  dock_tick(state);
  return !(chip.alert & chip.alert_enable) || chip.reset_held;
}

bool dock_hpd(EmuState *state) {
  g_state = state;
  dock_tick(state);
  return chip.hpd;
}

uint32_t dpaux1_read(EmuState *state, uint32_t off) {
  g_state = state;
  dock_tick(state);
  uint32_t r = (off >> 2) & 0x7F;
  switch (r) {
  case A_STAT: return (aux[A_STAT] & ~(1u << 28)) | aux_hpd_bit();
  case A_INTR: {
    uint32_t v = aux[A_INTR];
    // Plug/unplug events latch on HPD edges.
    static bool last = false;
    bool h = aux_hpd_bit() != 0;
    if (h != last) {
      aux[A_INTR] |= h ? 1u : 2u;
      last = h;
      v = aux[A_INTR];
    }
    return v;
  }
  default: return aux[r];
  }
}

void dpaux1_write(EmuState *state, uint32_t off, uint32_t val) {
  g_state = state;
  dock_tick(state);
  uint32_t r = (off >> 2) & 0x7F;
  trace("aux-w", off, val);
  switch (r) {
  case A_INTR:
    aux[A_INTR] &= ~val;                    // write 1 to clear
    break;
  case A_CTL:
    aux[A_CTL] = val;
    if (val & (1u << 31))                   // RST
      break;
    if (val & (1u << 16))
      aux_transact();
    break;
  case A_STAT:
    break;
  default:
    aux[r] = val;
    break;
  }
}

uint32_t sor1_read(EmuState *state, uint32_t off) {
  g_state = state;
  dock_tick(state);
  uint32_t r = (off >> 2) & 0x3FF;
  switch (r) {
  case S_SUPER_STATE0:
  case S_STATE0:
    return 0;                                // UPDATE self-clears
  case S_PWR: {
    uint32_t v = sor[S_PWR] & ~(1u << 31);  // SETTING_NEW done at once
    if ((sor_super_state1 & 0xC) == 0xC)
      v |= 1u << 28;                         // MODE NORMAL once attached normal
    return v;
  }
  case S_TEST: {
    uint32_t v = sor[S_TEST] & ~((1u << 10) | (3u << 8));
    if (sor_super_state1 & 0x8)
      v |= 1u << 10;                         // ATTACHED
    v |= (sor_super_state1 & 3) << 8;        // ACT_HEAD_OPMODE
    return v;
  }
  case S_LANE_SEQ_CTL:
    return sor[r] & ~(1u << 31);
  case S_PLL1: {
    // The termination comparator: high while TERMADJ is above the value
    // that matches the external resistor (the prod setting is 0x9).
    uint32_t termadj = (sor[S_PLL1] >> 9) & 0xF;
    return (sor[S_PLL1] & ~(1u << 15)) | (termadj > 9 ? (1u << 15) : 0);
  }
  case S_CRCA: {
    TvState t;
    dock_tv_state(state, &t);
    return (sor[S_CRC_CNTRL] & 1) && t.video ? 1 : 0;
  }
  case S_CRCB:
    return 0x1234ABCD;
  case 0x12: {                               // DP_DEBUG_MVID
    TvState t;
    dock_tv_state(state, &t);
    if (!t.video)
      return 0;
    uint64_t ls_khz = dpcd[0x100] == 0x14 ? 540000 : dpcd[0x100] == 0x0A ? 270000 : 162000;
    uint32_t nval = sor[0x5A] & 0xFFFFFF;    // DP_MN0
    if (!nval)
      nval = 0x8000;
    return (uint32_t)(head_b_pclk_khz() * nval / ls_khz) & 0xFFFFFF;
  }
  case 0x5A:                                 // DP_MN0: N_VAL resets to 0x8000
    return sor[0x5A] ? sor[0x5A] : 0x8000;
  default:
    return sor[r];
  }
}

void sor1_write(EmuState *state, uint32_t off, uint32_t val) {
  g_state = state;
  dock_tick(state);
  uint32_t r = (off >> 2) & 0x3FF;
  trace("sor1-w", off, val);
  switch (r) {
  case S_SUPER_STATE0:
    if (val & 1) {
      uint32_t prev = sor_super_state1;
      sor_super_state1 = sor[S_SUPER_STATE1];
      if (prev != sor_super_state1)
        DLOG("SOR1 %s, head %s, OR %s\n", (sor_super_state1 & 8) ? "attached" : "detached",
             (sor_super_state1 & 3) == 2 ? "awake" : "asleep",
             (sor_super_state1 & 4) ? "normal" : "safe");
    }
    return;
  case S_STATE0:
    if (val & 1)
      sor_state1 = sor[S_STATE1];
    return;
  case S_LANE_SEQ_CTL:
    sor[r] = val;
    if (val & (1u << 31))
      lanes_powered = !(val & (1u << 16));
    break;
  case S_PWR:
    sor[r] = val;
    return;
  default:
    sor[r] = val;
    break;
  }
  link_eval();
}

void dock_tv_state(EmuState *state, TvState *t) {
  g_state = state;
  memset(t, 0, sizeof(*t));
  t->connected = has_dp() && tv_present() && chip.attached;
  t->hpd = chip.hpd;
  uint32_t lanes = 0;
  t->link_trained = link_trained(&lanes);
  t->lanes = lanes;
  t->rate_code = dpcd[0x100];
  if (!has_dp()) {
    snprintf(t->status, sizeof(t->status), "no dock with video on the USB-C port");
    return;
  }
  if (!tv_present()) {
    snprintf(t->status, sizeof(t->status), "no TV on the dock");
    return;
  }
  if (!chip.attached) {
    snprintf(t->status, sizeof(t->status), "console not on the dock");
    return;
  }
  if (!chip.dp_configured) {
    snprintf(t->status, sizeof(t->status), "DisplayPort alt mode not configured by the PD controller");
    return;
  }
  if (!chip.hpd) {
    snprintf(t->status, sizeof(t->status), "waiting for HPD");
    return;
  }
  if (!t->link_trained) {
    const char *why = nullptr;
    uint32_t bad = 0;
    for (uint32_t j = 0; j < (lanes ? lanes : 1) && !why; j++)
      if ((why = lane_fault(j)))
        bad = j;
    if (!lanes)
      snprintf(t->status, sizeof(t->status), "link not configured (LANE_COUNT_SET 0)");
    else if (why)
      snprintf(t->status, sizeof(t->status), "lane %u: %s", bad, why);
    else
      snprintf(t->status, sizeof(t->status), "link not trained");
    return;
  }
  // Frames: SOR1 attached and awake, driven by head 1 over DP-A, and head 1
  // (DC B) running with its pixel clock routed to SOR1.
  uint32_t owner = sor_state1 & 0xF, proto = (sor_state1 >> 8) & 0xF;
  uint64_t dcb = 0x54240000;
  uint32_t cmd = mmio_bus_read(state, dcb + 0x032 * 4, 4);
  uint32_t winopt = mmio_bus_read(state, dcb + 0x402 * 4, 4);
  if ((sor_super_state1 & 0xF) != 0xE)
    snprintf(t->status, sizeof(t->status), "SOR1 not attached, awake and normal (SUPER_STATE1 %X)",
             sor_super_state1);
  else if (owner != 2 || proto != 8)
    snprintf(t->status, sizeof(t->status), "SOR1 STATE1 owner %u protocol %u: not head 1 over DP-A",
             owner, proto);
  else if (!(car(0x010) & (1u << 26)) || (car(0x004) & (1u << 26)))
    snprintf(t->status, sizeof(t->status), "display head B not clocked or in reset");
  else if (((cmd >> 5) & 3) != 1)
    snprintf(t->status, sizeof(t->status), "display head B stopped (DISPLAY_COMMAND %08X)", cmd);
  else if (!(winopt & (1u << 26)))
    snprintf(t->status, sizeof(t->status), "head B does not feed SOR1 (DISP_WIN_OPTIONS %08X)", winopt);
  else {
    t->video = true;
    snprintf(t->status, sizeof(t->status), "%u lane%s at %s", lanes, lanes == 1 ? "" : "s",
             dpcd[0x100] == 0x14 ? "5.4 Gbps" : dpcd[0x100] == 0x0A ? "2.7 Gbps" : "1.62 Gbps");
  }
}
