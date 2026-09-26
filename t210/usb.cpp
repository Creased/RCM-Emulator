// USB device mode, and the PC on the other end of the cable.
//
// bdk runs its gadgets (mass storage, HID) on one of two device controllers:
// the USB2 controller USB1 ("OTG", 0x7D000000) on Erista - bdk usbd.c, an
// EHCI-style device controller - and the XUSB device controller on Mariko
// (xusbd.c). A controller model implements UsbDc, which is all the host sees
// of it: attach, bus reset, SETUP packets, and data IN or OUT of an endpoint.
//
// The host plays the PC. It is plugged in with --usb-host (or [usb] host in
// the ini); without it nothing is on the cable, and a gadget waits for a
// connection the way it does on a console with no cable. Once the device
// attaches, the host resets the bus, enumerates, sets the configuration and
// runs the class:
//
//   mass storage  GET_MAX_LUN, then over Bulk-Only Transport INQUIRY, TEST
//                 UNIT READY, READ CAPACITY and two READ(10)s (the first 64
//                 sectors and the last 8), checked against the SD image when
//                 the capacity matches it; then it allows medium removal,
//                 ejects (START STOP UNIT, LoEj) and polls TEST UNIT READY
//                 until the gadget lets go, as Linux does
//   HID           the report descriptor, SET_IDLE, then interrupt IN reports
//
// Everything is logged as [usb-host]. The host only reads: nothing is ever
// written to the storage a gadget exports.
//
// The host advances whenever the payload touches the controller - every
// register access runs it as far as the device and the emulated clock allow
// - so it needs no thread and no main-loop hook, and runs are reproducible.

#include "usb.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <unicorn/unicorn.h>

#include "../emu_state.h"
#include "../platform.h"
#include "memory_map.h"

namespace {

// ---- What the host sees of a device controller ------------------------------

// Transfer results: bytes moved, or one of these.
constexpr int XFER_NAK   = -1; // endpoint not ready; try again later
constexpr int XFER_STALL = -2; // endpoint halted

struct UsbDc {
  virtual ~UsbDc() = default;
  virtual bool attached() = 0;                 // pull-up on, controller running
  virtual void bus_reset() = 0;                // reset, then high speed
  virtual bool reset_taken() = 0;              // the device handled the reset
  virtual void setup(const uint8_t pkt[8]) = 0;
  virtual bool setup_taken() = 0;              // the device read the SETUP
  // Data from IN endpoint `ep` into buf, at most max bytes. *short_pkt: the
  // device sent less than max, which ends a data stage.
  virtual int in(int ep, uint8_t *buf, int max, bool *short_pkt) = 0;
  // Data to OUT endpoint `ep`. Returns what the device accepted.
  virtual int out(int ep, const uint8_t *buf, int len) = 0;
  virtual uint32_t address() = 0;              // the USB address the device set
  virtual void detach() = 0;                   // the host saw the device go
};

EmuState *g_state = nullptr;

// RCM_USB_TRACE=1 logs every transfer, event and doorbell.
bool trace_on() {
  static int on = -1;
  if (on < 0) {
    const char *e = getenv("RCM_USB_TRACE");
    on = e && *e && *e != '0';
  }
  return on;
}
#define UTRACE(...)                                                            \
  do {                                                                         \
    if (trace_on())                                                            \
      printf("[usb-trace] " __VA_ARGS__);                                      \
  } while (0)

uint64_t now_us() { return g_state ? g_state->emu_usec : 0; }

bool mem_read(uint64_t addr, void *buf, size_t n) {
  return g_state && g_state->uc &&
         uc_mem_read(g_state->uc, addr, buf, n) == UC_ERR_OK;
}

bool mem_write(uint64_t addr, const void *buf, size_t n) {
  return g_state && g_state->uc &&
         uc_mem_write(g_state->uc, addr, buf, n) == UC_ERR_OK;
}

// ---- The host ---------------------------------------------------------------

constexpr uint64_t kDebounceUs = 100000; // USB 2.0 7.1.7.3: attach debounce
constexpr uint64_t kRecoveryUs = 10000;  // reset recovery
constexpr uint64_t kSetAddrUs  = 2000;   // USB 2.0 9.2.6.3: SET_ADDRESS recovery
// Between SET_CONFIGURATION and the first class request a PC binds its
// class driver; bdk's XUSB driver stalls GET_MAX_LUN until the gadget has
// come back from enumeration and asked for it, so an instant one fails.
constexpr uint64_t kBindUs     = 100000;
constexpr uint64_t kTimeoutUs  = 5000000;
constexpr uint64_t kEjectPollUs = 500000;

struct Endpoint {
  uint8_t addr = 0, attr = 0, interval = 0;
  uint16_t max_packet = 0;
};

// One control transfer: SETUP, an optional data stage, the status stage.
struct Control {
  enum Stage { SETUP, DATA, STATUS, DONE } stage = DONE;
  uint8_t pkt[8] = {};
  bool dir_in = false;
  uint16_t len = 0;
  std::vector<uint8_t> data; // IN: what came back; OUT: what to send
  size_t moved = 0;
  bool stalled = false;
};

// One Bulk-Only Transport command: CBW, an optional data phase, CSW.
struct Bot {
  enum Phase { CBW, DATA, CSW, DONE } phase = DONE;
  uint8_t cbw[31] = {};
  bool dir_in = true;
  uint32_t len = 0;
  std::vector<uint8_t> data;
  uint8_t csw[13] = {};
  size_t moved = 0;
  bool stalled = false;
};

enum class Step {
  Unplugged, WaitAttach, Debounce, WaitReset, Recover,
  GetDevice8, SetAddress, AddrRecovery, GetDevice, GetConfig9, GetConfig,
  GetLangs, GetVendor, GetProduct, GetSerial, SetConfig, Bind,
  MscMaxLun, MscInquiry, MscTur, MscSense, MscCapacity, MscReadFirst, MscReadLast,
  MscAllow, MscEject, MscAfterEject, MscEjectWait,
  HidReport, HidIdle, HidPoll,
  Idle, Failed,
};

class Host {
public:
  // Both device controllers; the host talks to whichever one attaches.
  void set_controllers(UsbDc *a, UsbDc *b) {
    dcs_[0] = a;
    dcs_[1] = b;
  }

  void run() {
    if (!g_state || !dcs_[0])
      return;
    bool plugged = g_state->usb_host.load();
    if (!plugged) {
      if (step_ != Step::Unplugged) {
        if (dc_)
          dc_->detach();
        step_ = Step::Unplugged;
        printf("[usb-host] unplugged\n");
      }
      return;
    }
    if (step_ == Step::Unplugged)
      step_ = Step::WaitAttach;
    if (step_ == Step::WaitAttach) {
      dc_ = nullptr;
      for (UsbDc *dc : dcs_)
        if (dc && dc->attached())
          dc_ = dc;
      if (!dc_)
        return;
    }
    // A device that goes away (controller stopped or reset) ends the session;
    // the next attach starts a new one.
    if (step_ != Step::WaitAttach && !dc_->attached()) {
      printf("[usb-host] device detached\n");
      dc_->detach();
      step_ = Step::WaitAttach;
      ctl_.stage = Control::DONE;
      bot_.phase = Bot::DONE;
    }
    // Several steps can complete in one call; stop once one has to wait.
    for (int guard = 0; guard < 64; guard++) {
      Step before = step_;
      tick();
      if (step_ == before && !progressed_)
        break;
      progressed_ = false;
    }
  }

  void reset() {
    step_ = Step::Unplugged;
    ctl_ = Control();
    bot_ = Bot();
  }

private:
  UsbDc *dcs_[2] = {};
  UsbDc *dc_ = nullptr;
  Step step_ = Step::Unplugged;
  bool progressed_ = false;
  uint64_t wait_until_ = 0, deadline_ = 0;
  Control ctl_;
  Bot bot_;

  // What enumeration found.
  std::vector<uint8_t> dev_, cfg_;
  uint8_t ep0_max_ = 64, cfg_value_ = 1, iface_ = 0;
  uint8_t iface_class_ = 0, iface_sub_ = 0, iface_proto_ = 0;
  uint16_t hid_report_len_ = 0;
  Endpoint bulk_in_, bulk_out_, intr_in_;
  std::string vendor_, product_, serial_;

  // Mass storage.
  uint32_t tag_ = 0, blocks_ = 0, block_len_ = 0;
  int tur_tries_ = 0;
  uint64_t eject_deadline_ = 0;
  int reports_ = 0;

  void go(Step s) {
    step_ = s;
    progressed_ = true;
  }

  void fail(const char *what) {
    printf("[usb-host] %s - giving up on this device\n", what);
    go(Step::Failed);
  }

  // ---- Control transfers ----

  void control(uint8_t bm, uint8_t req, uint16_t val, uint16_t idx,
               uint16_t len, const std::vector<uint8_t> &out = {}) {
    ctl_ = Control();
    ctl_.pkt[0] = bm;
    ctl_.pkt[1] = req;
    ctl_.pkt[2] = val & 0xFF;
    ctl_.pkt[3] = val >> 8;
    ctl_.pkt[4] = idx & 0xFF;
    ctl_.pkt[5] = idx >> 8;
    ctl_.pkt[6] = len & 0xFF;
    ctl_.pkt[7] = len >> 8;
    ctl_.dir_in = bm & 0x80;
    ctl_.len = len;
    ctl_.data = ctl_.dir_in ? std::vector<uint8_t>(len) : out;
    ctl_.stage = Control::SETUP;
    deadline_ = now_us() + kTimeoutUs;
    dc_->setup(ctl_.pkt);
  }

  // True once the transfer is over (done or stalled); false while waiting.
  bool control_poll() {
    Control &c = ctl_;
    for (;;) {
      switch (c.stage) {
      case Control::SETUP:
        if (!dc_->setup_taken())
          return false;
        c.stage = c.len ? Control::DATA : Control::STATUS;
        break;
      case Control::DATA: {
        int n;
        bool short_pkt = false;
        if (c.dir_in)
          n = dc_->in(0, c.data.data() + c.moved, (int)(c.len - c.moved),
                      &short_pkt);
        else
          n = dc_->out(0, c.data.data() + c.moved, (int)(c.len - c.moved));
        if (n == XFER_STALL) {
          c.stalled = true;
          c.stage = Control::DONE;
          return true;
        }
        if (n == XFER_NAK)
          return false;
        c.moved += n;
        if (c.moved >= c.len || short_pkt) {
          c.data.resize(c.moved);
          c.stage = Control::STATUS;
        }
        break;
      }
      case Control::STATUS: {
        // The status stage runs the other way: a zero-length OUT after an
        // IN data stage, a zero-length IN otherwise.
        int n;
        bool short_pkt = false;
        uint8_t none = 0;
        if (c.dir_in && c.len)
          n = dc_->out(0, nullptr, 0);
        else
          n = dc_->in(0, &none, 0, &short_pkt);
        if (n == XFER_STALL)
          c.stalled = true;
        else if (n == XFER_NAK)
          return false;
        c.stage = Control::DONE;
        return true;
      }
      case Control::DONE:
        return true;
      }
    }
  }

  // ---- Bulk-Only Transport ----

  void bot(const uint8_t *cdb, int cdb_len, bool dir_in, uint32_t len) {
    bot_ = Bot();
    uint8_t *w = bot_.cbw;
    uint32_t tag = ++tag_;
    memcpy(w, "USBC", 4);
    memcpy(w + 4, &tag, 4);
    memcpy(w + 8, &len, 4);
    w[12] = dir_in ? 0x80 : 0x00;
    w[13] = 0;                     // LUN 0
    w[14] = (uint8_t)cdb_len;
    memcpy(w + 15, cdb, cdb_len);
    bot_.dir_in = dir_in;
    bot_.len = len;
    bot_.data.resize(len);
    bot_.phase = Bot::CBW;
    deadline_ = now_us() + kTimeoutUs;
  }

  // True once the command is over; false while waiting.
  bool bot_poll() {
    Bot &b = bot_;
    int out_ep = bulk_out_.addr & 0xF, in_ep = bulk_in_.addr & 0xF;
    for (;;) {
      switch (b.phase) {
      case Bot::CBW: {
        int n = dc_->out(out_ep, b.cbw + b.moved, (int)(31 - b.moved));
        if (n == XFER_STALL) {
          b.stalled = true;
          b.phase = Bot::DONE;
          return true;
        }
        if (n == XFER_NAK)
          return false;
        b.moved += n;
        if (b.moved == 31) {
          b.moved = 0;
          b.phase = b.len ? Bot::DATA : Bot::CSW;
        }
        break;
      }
      case Bot::DATA: {
        bool short_pkt = false;
        int n = b.dir_in
            ? dc_->in(in_ep, b.data.data() + b.moved, (int)(b.len - b.moved),
                      &short_pkt)
            : dc_->out(out_ep, b.data.data() + b.moved, (int)(b.len - b.moved));
        if (n == XFER_STALL) {
          // A halted data phase is still followed by the CSW.
          b.stalled = true;
          b.phase = Bot::CSW;
          break;
        }
        if (n == XFER_NAK)
          return false;
        b.moved += n;
        if (b.moved >= b.len || short_pkt) {
          b.data.resize(b.moved);
          b.moved = 0;
          b.phase = Bot::CSW;
        }
        break;
      }
      case Bot::CSW: {
        bool short_pkt = false;
        int n = dc_->in(in_ep, b.csw + b.moved, (int)(13 - b.moved), &short_pkt);
        if (n == 0 && b.moved == 0)
          return false;                     // a zero-length packet ending the data
        if (n == XFER_STALL) {
          b.stalled = true;
          b.phase = Bot::DONE;
          return true;
        }
        if (n == XFER_NAK)
          return false;
        b.moved += n;
        if (b.moved >= 13 || short_pkt) {
          b.phase = Bot::DONE;
          return true;
        }
        break;
      }
      case Bot::DONE:
        return true;
      }
    }
  }

  // The CSW's status byte, or -1 if it is not a valid CSW for this command.
  int bot_status() const {
    uint32_t tag;
    memcpy(&tag, bot_.csw + 4, 4);
    if (memcmp(bot_.csw, "USBS", 4) != 0 || tag != tag_)
      return -1;
    return bot_.csw[12];
  }

  // ---- Descriptors ----

  static std::string string_desc(const std::vector<uint8_t> &d) {
    std::string s;
    for (size_t i = 2; i + 1 < d.size() && i < d[0]; i += 2)
      s += (d[i] >= 0x20 && d[i] < 0x7F && !d[i + 1]) ? (char)d[i] : '?';
    return s;
  }

  void parse_config() {
    iface_class_ = iface_sub_ = iface_proto_ = 0;
    bulk_in_ = bulk_out_ = intr_in_ = Endpoint();
    hid_report_len_ = 0;
    if (cfg_.size() >= 9)
      cfg_value_ = cfg_[5];
    bool first_iface = true;
    for (size_t i = 0; i + 2 <= cfg_.size() && cfg_[i] >= 2;
         i += cfg_[i]) {
      const uint8_t *d = &cfg_[i];
      size_t n = std::min<size_t>(d[0], cfg_.size() - i);
      if (d[1] == 4 && n >= 9) {          // interface
        if (!first_iface)
          break;                          // the first interface is enough
        first_iface = false;
        iface_ = d[2];
        iface_class_ = d[5];
        iface_sub_ = d[6];
        iface_proto_ = d[7];
      } else if (d[1] == 0x21 && n >= 9) { // HID descriptor
        hid_report_len_ = d[7] | d[8] << 8;
      } else if (d[1] == 5 && n >= 7) {   // endpoint
        Endpoint e;
        e.addr = d[2];
        e.attr = d[3];
        e.max_packet = d[4] | d[5] << 8;
        e.interval = d[6];
        int type = e.attr & 3;
        if (type == 2 && (e.addr & 0x80))
          bulk_in_ = e;
        else if (type == 2)
          bulk_out_ = e;
        else if (type == 3 && (e.addr & 0x80))
          intr_in_ = e;
      }
    }
  }

  const char *class_name() const {
    if (iface_class_ == 0x08 && iface_proto_ == 0x50)
      return "mass storage, bulk-only";
    if (iface_class_ == 0x03)
      return "HID";
    return "a class this host does not drive";
  }

  // ---- Mass storage checks ----

  // Compare what was read with the SD image, when the disk is the size of it.
  const char *compare_sd(uint32_t lba, const std::vector<uint8_t> &got) {
    if (!g_state || g_state->sd_fd < 0)
      return "no SD image to compare with";
    int64_t size = file_size64(g_state->sd_fd);
    if (size / 512 != (int64_t)blocks_ + 1)
      return "not the SD image's size, not compared";
    std::vector<uint8_t> want(got.size());
    if (pread(g_state->sd_fd, want.data(), want.size(),
              (long long)lba * 512) != (ssize_t)want.size())
      return "SD image unreadable";
    return memcmp(want.data(), got.data(), got.size()) == 0
               ? "matches the SD image"
               : "DIFFERS from the SD image";
  }

  static uint32_t crc32(const std::vector<uint8_t> &d) {
    uint32_t c = 0xFFFFFFFF;
    for (uint8_t b : d) {
      c ^= b;
      for (int k = 0; k < 8; k++)
        c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1)));
    }
    return ~c;
  }

  static uint32_t be32(const uint8_t *p) {
    return (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3];
  }

  void read10(uint32_t lba, uint16_t count) {
    uint8_t cdb[10] = {0x28, 0, (uint8_t)(lba >> 24), (uint8_t)(lba >> 16),
                       (uint8_t)(lba >> 8), (uint8_t)lba, 0,
                       (uint8_t)(count >> 8), (uint8_t)count, 0};
    bot(cdb, 10, true, (uint32_t)count * block_len_);
  }

  // A BOT command finished: false (and the session failed) if it did not end
  // with a good CSW.
  bool bot_ok(const char *what) {
    int st = bot_status();
    if (bot_.stalled || st != 0) {
      printf("[usb-host] %s: %s\n", what,
             st < 0 ? "no valid CSW" : bot_.stalled ? "stalled" : "failed");
      fail("mass storage command failed");
      return false;
    }
    return true;
  }

  // ---- The session ----

  void tick() {
    switch (step_) {
    case Step::Unplugged:
    case Step::Idle:
    case Step::Failed:
      return;

    case Step::WaitAttach:
      if (!dc_->attached())
        return;
      printf("[usb-host] device attached\n");
      wait_until_ = now_us() + kDebounceUs;
      go(Step::Debounce);
      return;

    case Step::Debounce:
      if (now_us() < wait_until_)
        return;
      dc_->bus_reset();
      deadline_ = now_us() + kTimeoutUs;
      go(Step::WaitReset);
      return;

    case Step::WaitReset:
      if (!dc_->reset_taken()) {
        if (now_us() > deadline_)
          fail("the device never took the bus reset");
        return;
      }
      wait_until_ = now_us() + kRecoveryUs;
      go(Step::Recover);
      return;

    case Step::Recover:
      if (now_us() < wait_until_)
        return;
      control(0x80, 6, 0x0100, 0, 64);          // GET_DESCRIPTOR device
      go(Step::GetDevice8);
      return;

    case Step::AddrRecovery:
      if (now_us() < wait_until_)
        return;
      control(0x80, 6, 0x0100, 0, 18);
      go(Step::GetDevice);
      return;

    case Step::Bind:
      if (now_us() < wait_until_)
        return;
      start_class();
      return;

    case Step::MscEjectWait: {
      if (now_us() < wait_until_)
        return;
      if (now_us() > deadline_)
        return fail("the device never let go after the eject");
      uint8_t cdb[6] = {0x00, 0, 0, 0, 0, 0};   // TEST UNIT READY
      bot(cdb, 6, false, 0);
      go(Step::MscAfterEject);
      return;
    }

    case Step::HidPoll:
      finish();                                 // polls the endpoint itself
      return;

    default:
      break;
    }

    // Everything else waits on a transfer: a BOT command in the mass storage
    // steps, a control transfer otherwise.
    bool is_bot = step_ >= Step::MscInquiry && step_ <= Step::MscAfterEject;
    if (!(is_bot ? bot_poll() : control_poll())) {
      if (now_us() > deadline_ && step_ != Step::MscAfterEject)
        fail("transfer timed out");
      return;
    }
    finish();
  }

  // The class driver binds: its first request.
  void start_class() {
    if (iface_class_ == 0x08 && iface_proto_ == 0x50 && bulk_in_.addr &&
        bulk_out_.addr) {
      control(0xA1, 0xFE, 0, iface_, 1);        // GET_MAX_LUN
      return go(Step::MscMaxLun);
    }
    if (iface_class_ == 0x03 && intr_in_.addr) {
      control(0x81, 6, 0x2200, iface_, hid_report_len_ ? hid_report_len_ : 256);
      return go(Step::HidReport);
    }
    printf("[usb-host] configured; nothing more to do\n");
    go(Step::Idle);
  }

  // The transfer of the current step is over: act on it and start the next.
  void finish() {
    switch (step_) {
    case Step::GetDevice8:
      if (ctl_.stalled || ctl_.data.size() < 8)
        return fail("GET_DESCRIPTOR(device) failed");
      ep0_max_ = ctl_.data[7];
      control(0x00, 5, 1, 0, 0);                // SET_ADDRESS 1
      return go(Step::SetAddress);

    case Step::SetAddress:
      if (ctl_.stalled)
        return fail("SET_ADDRESS stalled");
      wait_until_ = now_us() + kSetAddrUs;
      return go(Step::AddrRecovery);

    case Step::GetDevice:
      if (ctl_.stalled || ctl_.data.size() < 18)
        return fail("GET_DESCRIPTOR(device) failed");
      dev_ = ctl_.data;
      control(0x80, 6, 0x0200, 0, 9);           // configuration header
      return go(Step::GetConfig9);

    case Step::GetConfig9: {
      if (ctl_.stalled || ctl_.data.size() < 9)
        return fail("GET_DESCRIPTOR(configuration) failed");
      uint16_t total = ctl_.data[2] | ctl_.data[3] << 8;
      control(0x80, 6, 0x0200, 0, total);
      return go(Step::GetConfig);
    }

    case Step::GetConfig:
      if (ctl_.stalled || ctl_.data.size() < 9)
        return fail("GET_DESCRIPTOR(configuration) failed");
      cfg_ = ctl_.data;
      parse_config();
      control(0x80, 6, 0x0300, 0, 255);         // string 0: languages
      return go(Step::GetLangs);

    case Step::GetLangs:
      vendor_ = product_ = serial_ = "";
      if (dev_[14]) {
        control(0x80, 6, 0x0300 | dev_[14], 0x0409, 255);
        return go(Step::GetVendor);
      }
      [[fallthrough]];
    case Step::GetVendor:
      if (step_ == Step::GetVendor && !ctl_.stalled)
        vendor_ = string_desc(ctl_.data);
      if (dev_[15]) {
        control(0x80, 6, 0x0300 | dev_[15], 0x0409, 255);
        return go(Step::GetProduct);
      }
      [[fallthrough]];
    case Step::GetProduct:
      if (step_ == Step::GetProduct && !ctl_.stalled)
        product_ = string_desc(ctl_.data);
      if (dev_[16]) {
        control(0x80, 6, 0x0300 | dev_[16], 0x0409, 255);
        return go(Step::GetSerial);
      }
      [[fallthrough]];
    case Step::GetSerial:
      if (step_ == Step::GetSerial && !ctl_.stalled)
        serial_ = string_desc(ctl_.data);
      printf("[usb-host] high-speed device %04X:%04X \"%s\" \"%s\" serial "
             "\"%s\", address %u\n",
             dev_[8] | dev_[9] << 8, dev_[10] | dev_[11] << 8, vendor_.c_str(),
             product_.c_str(), serial_.c_str(), dc_->address());
      printf("[usb-host] configuration %u: %s", cfg_value_, class_name());
      if (bulk_in_.addr)
        printf(" (EP %02X IN, EP %02X OUT, %u-byte packets)", bulk_in_.addr,
               bulk_out_.addr, bulk_in_.max_packet);
      if (intr_in_.addr)
        printf(" (EP %02X IN, interrupt every %u)", intr_in_.addr,
               intr_in_.interval);
      printf("\n");
      control(0x00, 9, cfg_value_, 0, 0);       // SET_CONFIGURATION
      return go(Step::SetConfig);

    case Step::SetConfig:
      if (ctl_.stalled)
        return fail("SET_CONFIGURATION stalled");
      wait_until_ = now_us() + kBindUs;
      return go(Step::Bind);

    // ---- Mass storage ----

    case Step::MscMaxLun: {
      int luns = ctl_.stalled || ctl_.data.empty() ? 1 : ctl_.data[0] + 1;
      printf("[usb-host] %d LUN%s\n", luns, luns == 1 ? "" : "s");
      uint8_t cdb[6] = {0x12, 0, 0, 0, 36, 0};  // INQUIRY
      bot(cdb, 6, true, 36);
      return go(Step::MscInquiry);
    }

    case Step::MscInquiry: {
      if (!bot_ok("INQUIRY"))
        return;
      std::string id;
      for (size_t i = 8; i < 32 && i < bot_.data.size(); i++)
        id += (bot_.data[i] >= 0x20 && bot_.data[i] < 0x7F) ? (char)bot_.data[i]
                                                            : ' ';
      printf("[usb-host] LUN 0: \"%s\"%s\n", id.c_str(),
             bot_.data.size() > 1 && (bot_.data[1] & 0x80) ? ", removable" : "");
      uint8_t cdb[6] = {0x00, 0, 0, 0, 0, 0};   // TEST UNIT READY
      bot(cdb, 6, false, 0);
      tur_tries_ = 0;
      return go(Step::MscTur);
    }

    case Step::MscTur: {
      // A fresh device answers the first TEST UNIT READY with a unit
      // attention; ask for the sense data and try again, as a PC does.
      if (!bot_.stalled && bot_status() == 1 && ++tur_tries_ < 5) {
        uint8_t cdb[6] = {0x03, 0, 0, 0, 18, 0}; // REQUEST SENSE
        bot(cdb, 6, true, 18);
        return go(Step::MscSense);
      }
      if (!bot_ok("TEST UNIT READY"))
        return;
      uint8_t cdb[10] = {0x25};                 // READ CAPACITY(10)
      bot(cdb, 10, true, 8);
      return go(Step::MscCapacity);
    }

    case Step::MscSense: {
      if (!bot_ok("REQUEST SENSE"))
        return;
      if (bot_.data.size() >= 14)
        printf("[usb-host] TEST UNIT READY: sense key %X, ASC %02X/%02X\n",
               bot_.data[2] & 0xF, bot_.data[12], bot_.data[13]);
      uint8_t cdb[6] = {0x00, 0, 0, 0, 0, 0};
      bot(cdb, 6, false, 0);
      return go(Step::MscTur);
    }

    case Step::MscCapacity: {
      if (!bot_ok("READ CAPACITY"))
        return;
      if (bot_.data.size() < 8)
        return fail("short READ CAPACITY");
      blocks_ = be32(&bot_.data[0]);
      block_len_ = be32(&bot_.data[4]);
      uint64_t bytes = ((uint64_t)blocks_ + 1) * block_len_;
      printf("[usb-host] %u blocks of %u bytes (%llu MiB)\n", blocks_ + 1,
             block_len_, (unsigned long long)(bytes >> 20));
      if (block_len_ != 512 || blocks_ < 72)
        return fail("unexpected capacity");
      read10(0, 64);
      return go(Step::MscReadFirst);
    }

    case Step::MscReadFirst:
      if (!bot_ok("READ(10)"))
        return;
      printf("[usb-host] read blocks 0-63 (crc32 %08X): %s\n",
             crc32(bot_.data), compare_sd(0, bot_.data));
      read10(blocks_ - 7, 8);
      return go(Step::MscReadLast);

    case Step::MscReadLast: {
      if (!bot_ok("READ(10)"))
        return;
      printf("[usb-host] read blocks %u-%u (crc32 %08X): %s\n", blocks_ - 7,
             blocks_, crc32(bot_.data), compare_sd(blocks_ - 7, bot_.data));
      uint8_t cdb[6] = {0x1E, 0, 0, 0, 0, 0};   // PREVENT ALLOW: allow
      bot(cdb, 6, false, 0);
      return go(Step::MscAllow);
    }

    case Step::MscAllow: {
      if (!bot_ok("PREVENT ALLOW MEDIUM REMOVAL"))
        return;
      uint8_t cdb[6] = {0x1B, 0, 0, 0, 0x02, 0}; // START STOP UNIT: eject
      bot(cdb, 6, false, 0);
      return go(Step::MscEject);
    }

    case Step::MscEject: {
      if (!bot_ok("START STOP UNIT"))
        return;
      printf("[usb-host] ejected; polling until the device lets go\n");
      eject_deadline_ = now_us() + 60000000;
      wait_until_ = now_us() + kEjectPollUs;
      deadline_ = eject_deadline_;
      return go(Step::MscEjectWait);
    }

    case Step::MscAfterEject:
      // The gadget now reports the medium gone. Keep asking, as Linux does,
      // until it detaches; the answer does not matter.
      wait_until_ = now_us() + kEjectPollUs;
      deadline_ = eject_deadline_;
      return go(Step::MscEjectWait);

    // ---- HID ----

    case Step::HidReport:
      printf("[usb-host] HID report descriptor: %zu bytes%s\n",
             ctl_.data.size(), ctl_.stalled ? " (stalled)" : "");
      control(0x21, 0x0A, 0, iface_, 0);        // SET_IDLE
      return go(Step::HidIdle);

    case Step::HidIdle:
      // Idle rate 0: the device reports only when its input changes.
      printf("[usb-host] SET_IDLE %s; polling EP %02X for reports\n",
             ctl_.stalled ? "stalled" : "done", intr_in_.addr);
      reports_ = 0;
      deadline_ = now_us() + kTimeoutUs;
      return go(Step::HidPoll);

    case Step::HidPoll: {
      uint8_t buf[512];
      bool short_pkt = false;
      int n = dc_->in(intr_in_.addr & 0xF, buf,
                      std::min<int>(intr_in_.max_packet, sizeof(buf)),
                      &short_pkt);
      if (n == XFER_NAK || n == XFER_STALL)
        return;
      if (reports_ < 3) {
        printf("[usb-host] HID report %d:", reports_ + 1);
        for (int i = 0; i < n && i < 16; i++)
          printf(" %02X", buf[i]);
        printf("\n");
      }
      if (++reports_ == 3)
        printf("[usb-host] HID reports arriving; polling on\n");
      progressed_ = true;
      return;
    }

    default:
      return;
    }
  }
};

Host g_host;

// ---- USB2 device controller (USB1 / OTG, 0x7D000000) ------------------------
//
// The EHCI-derived device controller bdk's usbd.c drives (register layout in
// its usb_t210.h). Endpoint n has an OUT ("RX", bit n) and an IN ("TX", bit
// 16 + n) half, each with a 64-byte queue head at ENDPOINTLISTADDR + (2n +
// in) * 64 - bdk keeps them in the controller's own memory at +0x1000 - and
// a chain of transfer descriptors (dTDs) in DRAM:
//
//   dTD  +0  next dTD (bit 0 terminates)
//        +4  token: bytes left (30:16), IOC (15), ACTIVE (7), HALTED (6)
//        +8  five buffer page pointers
//   dQH  +0  capabilities   +4 current dTD   +8 next dTD   +C token
//        +28 the last SETUP packet
//
// Priming an endpoint (ENDPTPRIME) loads the first dTD and sets its bit in
// ENDPTSTATUS until the transfer retires; the host then moves the data
// through the dTD buffers, writes back what was left in each token, and sets
// the bit in ENDPTCOMPLETE. A SETUP lands in queue head 0 with ENDPTSETUPSTAT
// bit 0 set. The rest of the block - the UTMI PHY's registers at 0x400 and
// 0x800 - reads back what was written, bar PHY_CLK_VALID.

constexpr uint32_t R_USBCMD        = 0x130;
constexpr uint32_t R_USBSTS        = 0x134;
constexpr uint32_t R_DEVICEADDR    = 0x144; // PERIODICLISTBASE in device mode
constexpr uint32_t R_EPLISTADDR    = 0x148; // ASYNCLISTADDR in device mode
constexpr uint32_t R_PORTSC1       = 0x174;
constexpr uint32_t R_HOSTPC1_DEVLC = 0x1B4;
constexpr uint32_t R_OTGSC         = 0x1F4;
constexpr uint32_t R_USBMODE       = 0x1F8;
constexpr uint32_t R_SETUPSTAT     = 0x208;
constexpr uint32_t R_PRIME         = 0x20C;
constexpr uint32_t R_FLUSH         = 0x210;
constexpr uint32_t R_STATUS        = 0x214;
constexpr uint32_t R_COMPLETE      = 0x218;
constexpr uint32_t R_EPCTRL0       = 0x21C;
constexpr uint32_t R_SUSP_CTRL     = 0x400;

constexpr uint32_t USBCMD_RUN = 1u << 0, USBCMD_RESET = 1u << 1;
constexpr uint32_t USBSTS_UI = 1u << 0, USBSTS_PCI = 1u << 2,
                   USBSTS_URI = 1u << 6;
constexpr uint32_t PORTSC_CCS = 1u << 0, PORTSC_PE = 1u << 2,
                   PORTSC_HSP = 1u << 9;
constexpr uint32_t DEVLC_PHCD = 1u << 22, DEVLC_PSPD_HS = 2u << 25;
constexpr uint32_t EPCTRL_RX_STALL = 1u << 0, EPCTRL_RX_RESET = 1u << 6,
                   EPCTRL_RX_ENABLE = 1u << 7, EPCTRL_TX_STALL = 1u << 16,
                   EPCTRL_TX_RESET = 1u << 22, EPCTRL_TX_ENABLE = 1u << 23;
constexpr uint32_t SUSP_PHY_CLK_VALID = 1u << 7, SUSP_UTMIP_RESET = 1u << 11,
                   SUSP_UTMIP_PHY_ENB = 1u << 12;
constexpr uint32_t DTD_ACTIVE = 1u << 7, DTD_IOC = 1u << 15;

class Usb2d : public UsbDc {
public:
  Usb2d() { reset_controller(); }

  uint32_t read(uint32_t off) {
    switch (off) {
    case R_PORTSC1:
      return connected_ ? (PORTSC_CCS | PORTSC_PE | PORTSC_HSP) : 0;
    case R_HOSTPC1_DEVLC:
      return (reg(off) & ~(3u << 25)) | (connected_ ? DEVLC_PSPD_HS : 0);
    case R_SUSP_CTRL: {
      uint32_t v = reg(off) & ~SUSP_PHY_CLK_VALID;
      return phy_valid() ? v | SUSP_PHY_CLK_VALID : v;
    }
    default:
      return reg(off);
    }
  }

  void write(uint32_t off, uint32_t val) {
    switch (off) {
    case R_USBCMD:
      if (val & USBCMD_RESET) {
        reset_controller();
        return;
      }
      reg(off) = val;
      return;
    case R_USBSTS:
    case R_SETUPSTAT:
    case R_COMPLETE:
      reg(off) &= ~val;                     // write 1 to clear
      return;
    case R_OTGSC:
      // Interrupt status (22:16) is write 1 to clear; the rest reads back.
      reg(off) = (val & ~(0x7Fu << 16)) | (reg(off) & ~val & (0x7Fu << 16));
      return;
    case R_PRIME:
      prime(val);
      return;
    case R_FLUSH:
      reg(R_STATUS) &= ~val;
      reg(R_PRIME) &= ~val;
      return;
    case R_STATUS:
    case R_PORTSC1:
      return;                               // read-only here
    default:
      break;
    }
    if (off >= R_EPCTRL0 && off < R_EPCTRL0 + 16 * 4) {
      // The data-toggle resets are strobes; endpoint 0 is always enabled.
      val &= ~(EPCTRL_RX_RESET | EPCTRL_TX_RESET);
      if (off == R_EPCTRL0)
        val |= EPCTRL_RX_ENABLE | EPCTRL_TX_ENABLE;
      reg(off) = val;
      return;
    }
    if (off < kSize)
      reg(off) = val;
  }

  // USBCMD.RESET: the controller's registers (0x000-0x3FF) to their reset
  // state; the PHY's and the queue head memory are not part of it.
  void reset_controller() {
    memset(regs_, 0, 0x400);
    reg(R_EPCTRL0) = EPCTRL_RX_ENABLE | EPCTRL_TX_ENABLE;
    connected_ = false;
  }

  void full_reset() {
    memset(regs_, 0, sizeof(regs_));
    reset_controller();
  }

  // ---- UsbDc ----

  bool attached() override {
    return (reg(R_USBMODE) & 3) == 2 && (reg(R_USBCMD) & USBCMD_RUN) &&
           phy_valid() && !(reg(R_HOSTPC1_DEVLC) & DEVLC_PHCD);
  }

  void bus_reset() override {
    connected_ = true;
    reg(R_USBSTS) |= USBSTS_URI | USBSTS_PCI;
  }

  bool reset_taken() override { return !(reg(R_USBSTS) & USBSTS_URI); }

  void setup(const uint8_t pkt[8]) override {
    // A SETUP clears a stalled endpoint 0 and whatever was primed on it.
    reg(R_EPCTRL0) &= ~(EPCTRL_RX_STALL | EPCTRL_TX_STALL);
    reg(R_STATUS) &= ~(1u | 1u << 16);
    uint64_t qh = qh_addr(0);
    qh_write(qh + 0x28, pkt, 8);
    reg(R_SETUPSTAT) |= 1;
    reg(R_USBSTS) |= USBSTS_UI;
  }

  bool setup_taken() override { return !(reg(R_SETUPSTAT) & 1); }

  int in(int ep, uint8_t *buf, int max, bool *short_pkt) override {
    uint32_t bit = 1u << (16 + ep);
    if (reg(R_EPCTRL0 + ep * 4) & EPCTRL_TX_STALL)
      return XFER_STALL;
    if (!(reg(R_STATUS) & bit))
      return XFER_NAK;
    int moved = transfer(2 * ep + 1, buf, max, true);
    *short_pkt = moved < max;
    return moved;
  }

  int out(int ep, const uint8_t *buf, int len) override {
    uint32_t bit = 1u << ep;
    if (reg(R_EPCTRL0 + ep * 4) & EPCTRL_RX_STALL)
      return XFER_STALL;
    if (!(reg(R_STATUS) & bit))
      return XFER_NAK;
    return transfer(2 * ep, const_cast<uint8_t *>(buf), len, false);
  }

  uint32_t address() override { return reg(R_DEVICEADDR) >> 25; }

  void detach() override { connected_ = false; }

private:
  static constexpr uint32_t kSize = 0x4000;
  uint32_t regs_[kSize / 4] = {};
  bool connected_ = false;

  uint32_t &reg(uint32_t off) { return regs_[(off & (kSize - 1)) / 4]; }

  bool phy_valid() {
    uint32_t s = reg(R_SUSP_CTRL);
    return (s & SUSP_UTMIP_PHY_ENB) && !(s & SUSP_UTMIP_RESET);
  }

  // Queue heads live wherever ENDPOINTLISTADDR points: bdk uses the
  // controller's own memory, but DRAM works the same way.
  uint64_t qh_addr(int index) { return (uint64_t)reg(R_EPLISTADDR) + index * 64; }

  bool in_controller(uint64_t a) {
    return a >= USB_OTG_BASE && a + 4 <= USB_OTG_BASE + kSize;
  }

  uint32_t qh_read32(uint64_t a) {
    if (in_controller(a))
      return reg((uint32_t)(a - USB_OTG_BASE));
    uint32_t v = 0;
    mem_read(a, &v, 4);
    return v;
  }

  void qh_write32(uint64_t a, uint32_t v) {
    if (in_controller(a))
      reg((uint32_t)(a - USB_OTG_BASE)) = v;
    else
      mem_write(a, &v, 4);
  }

  void qh_write(uint64_t a, const uint8_t *p, int n) {
    for (int i = 0; i < n; i += 4) {
      uint32_t v = 0;
      memcpy(&v, p + i, std::min(4, n - i));
      qh_write32(a + i, v);
    }
  }

  void prime(uint32_t mask) {
    for (int bit = 0; bit < 32; bit++) {
      if (!(mask & (1u << bit)))
        continue;
      int ep = bit & 15, in = bit >> 4;
      uint64_t qh = qh_addr(2 * ep + in);
      uint32_t next = qh_read32(qh + 8);
      if (next & 1)
        continue;                           // nothing queued
      uint32_t dtd = next & ~0x1Fu;
      uint32_t token = 0;
      mem_read(dtd + 4, &token, 4);
      qh_write32(qh + 4, dtd);
      qh_write32(qh + 0xC, token);
      reg(R_STATUS) |= 1u << bit;
    }
    reg(R_PRIME) = 0;                       // priming is instant
  }

  // Move data through the primed dTD chain of queue head `index`: into buf
  // for an IN endpoint, out of it for OUT. Each dTD it reaches retires with
  // what it did not move left in its token. The transfer completes on a
  // short dTD or at the end of the chain; if the data runs out exactly at a
  // dTD boundary with more dTDs queued, the endpoint stays primed on the
  // next one. A zero-length transfer retires the first dTD.
  int transfer(int index, uint8_t *buf, int len, bool to_host) {
    uint64_t qh = qh_addr(index);
    uint32_t dtd = qh_read32(qh + 4) & ~0x1Fu;
    UTRACE("ep %d %s: dTD @%08X, host %s %d bytes\n", index / 2,
           to_host ? "IN" : "OUT", dtd, to_host ? "takes up to" : "sends", len);
    int moved = 0;
    bool ioc = false, primed_on = false;
    uint32_t last = dtd, last_token = qh_read32(qh + 0xC), next = 1;
    for (int guard = 0; guard < 64; guard++) {
      uint32_t d[7] = {};
      if (!dtd || !mem_read(dtd, d, sizeof(d)) || !(d[1] & DTD_ACTIVE))
        break;
      uint32_t size = (d[1] >> 16) & 0x7FFF;
      uint32_t n = std::min<uint32_t>(size, (uint32_t)(len - moved));
      // Page 0 carries the starting offset; pages 1-4 are 4 KiB frames.
      uint32_t off0 = d[2] & 0xFFF;
      for (uint32_t i = 0; i < n;) {
        uint32_t pos = off0 + i;
        if ((pos >> 12) > 4)
          break;
        uint32_t addr = (d[2 + (pos >> 12)] & ~0xFFFu) | (pos & 0xFFF);
        uint32_t chunk = std::min<uint32_t>(n - i, 0x1000 - (pos & 0xFFF));
        if (to_host)
          mem_read(addr, buf + moved + i, chunk);
        else
          mem_write(addr, buf + moved + i, chunk);
        i += chunk;
      }
      moved += (int)n;
      uint32_t token = (d[1] & ~(0x7FFFu << 16) & ~DTD_ACTIVE) |
                       ((size - n) & 0x7FFF) << 16;
      mem_write(dtd + 4, &token, 4);
      ioc |= (d[1] & DTD_IOC) != 0;
      last = dtd;
      last_token = token;
      next = d[0];
      if (n < size || (next & 1))
        break;                              // short, or the chain ended
      dtd = next & ~0x1Fu;
      if (moved == len) {
        primed_on = true;                   // more queued, nothing to give
        break;
      }
    }
    if (primed_on) {
      uint32_t token = 0;
      mem_read(dtd + 4, &token, 4);
      qh_write32(qh + 4, dtd);
      qh_write32(qh + 8, dtd);
      qh_write32(qh + 0xC, token);
      return moved;
    }
    qh_write32(qh + 4, last);
    qh_write32(qh + 8, next);
    qh_write32(qh + 0xC, last_token);
    uint32_t bit = index & 1 ? 1u << (16 + index / 2) : 1u << (index / 2);
    reg(R_STATUS) &= ~bit;
    reg(R_COMPLETE) |= bit;
    if (ioc)
      reg(R_USBSTS) |= USBSTS_UI;
    return moved;
  }
};

Usb2d g_usb2d;

// ---- XUSB device controller (0x700D0000) ------------------------------------
//
// The device side of an xHCI controller, as bdk's xusbd.c drives it on
// Mariko (register layout in usb_t210.h, TRB and context layouts in xusbd.c).
// Endpoints are numbered as device context indexes: 0 is the control
// endpoint, 2 and 3 bulk OUT and IN. Each has a context (16 words at ECP +
// 64 * index: word 2 holds its transfer ring's dequeue pointer and DCS) and a
// ring of 16-byte TRBs the driver fills - Data and Status TRBs on the control
// ring, Normal TRBs on the bulk ones, a Link TRB closing each ring - ringing
// the doorbell (DB) after each. The controller answers on the event ring: two
// segments (ERST0/ERST1, sizes in ERSTSZ) it fills at EREP, flipping its
// cycle bit on each pass, raising ST.IP; the driver consumes up to ERDP. It
// posts a Port Status Change event on a bus reset, a Setup event for each
// SETUP (with a sequence number the driver echoes on the doorbell), and a
// Transfer event, with the bytes left and SUCCESS or SHORT_PKT, for each TRB
// the host completes. EP_HALT halts endpoints (EP_STCHG reports the change),
// EP_RELOAD reloads a context. PORTSC's change bits are write 1 to clear, and
// a write with LWS sets the link state: the driver arms the port by moving it
// to RxDetect, which is when the host sees it attach.

constexpr uint32_t X_DB = 0x04, X_ERSTSZ = 0x08, X_ERST0 = 0x10, X_ERST1 = 0x18,
                   X_ERDP = 0x20, X_EREP = 0x28, X_CTRL = 0x30, X_ST = 0x34,
                   X_PORTSC = 0x3C, X_ECP = 0x40, X_EP_HALT = 0x50,
                   X_EP_RELOAD = 0x58, X_EP_STCHG = 0x5C, X_EP_STOPPED = 0x78;
constexpr uint32_t XCTRL_ENABLE = 1u << 31;
constexpr uint32_t XST_RC = 1u << 0, XST_IP = 1u << 4;
constexpr uint32_t XPS_CCS = 1u << 0, XPS_PED = 1u << 1, XPS_PR = 1u << 4,
                   XPS_PLS_SHIFT = 5, XPS_PLS_MASK = 0xFu << 5,
                   XPS_PS_HS = 3u << 10, XPS_LWS = 1u << 16,
                   XPS_CHANGE = (1u << 17) | (1u << 19) | (0xFu << 20) |
                                (1u << 23),
                   XPS_CSC = 1u << 17, XPS_PRC = 1u << 21;
constexpr uint32_t PLS_U0 = 0, PLS_RXDETECT = 5;
constexpr uint32_t TRB_NORMAL = 1, TRB_DATA = 3, TRB_STATUS = 4, TRB_LINK = 6,
                   TRB_TRANSFER_EV = 32, TRB_PORT_EV = 34, TRB_SETUP_EV = 63;
constexpr uint32_t COMP_SUCCESS = 1, COMP_SHORT_PKT = 13;

class Xusbd : public UsbDc {
public:
  Xusbd() { reset_all(); }

  uint32_t read(uint32_t off) { return reg(off); }

  void write(uint32_t off, uint32_t val) {
    switch (off) {
    case X_DB:
      UTRACE("doorbell %08X\n", val);
      return;                               // TRBs are picked up when due
    case X_ST:
      reg(off) &= ~(val & (XST_RC | XST_IP));
      return;
    case X_PORTSC: {
      uint32_t v = reg(off) & ~(val & XPS_CHANGE);   // write 1 to clear
      if (val & XPS_LWS) {
        v = (v & ~XPS_PLS_MASK) | (val & XPS_PLS_MASK);
        armed_ = ((val & XPS_PLS_MASK) >> XPS_PLS_SHIFT) == PLS_RXDETECT;
      }
      reg(off) = v;
      return;
    }
    case X_EP_HALT:
      reg(X_EP_STCHG) |= reg(off) ^ val;    // the halt state changed
      reg(off) = val;
      return;
    case X_EP_STCHG:
    case X_EP_STOPPED:
      reg(off) &= ~val;
      return;
    case X_EP_RELOAD:
      for (int i = 0; i < 4; i++)
        if (val & (1u << i)) {
          load_ring(i);
          UTRACE("reload ep %d: ring %08X dcs %u\n", i, rings_[i].deq,
                 rings_[i].dcs);
        }
      return;                               // reads back 0: done
    case X_ECP:
      reg(off) = val;
      load_ring(0);                         // the control endpoint's context
      return;
    case X_EREP:
      reg(off) = val;
      enq_ = val & ~0xFu;
      ecs_ = val & 1;
      return;
    default:
      if (off < kSize)
        reg(off) = val;
      return;
    }
  }

  void reset_all() {
    memset(regs_, 0, sizeof(regs_));
    for (auto &r : rings_)
      r = Ring();
    armed_ = connected_ = false;
    enq_ = 0;
    ecs_ = 1;
    seq_ = 0;
  }

  // ---- UsbDc ----

  bool attached() override { return (reg(X_CTRL) & XCTRL_ENABLE) && armed_; }

  void bus_reset() override {
    connected_ = true;
    uint32_t ps = reg(X_PORTSC) & ~(XPS_PLS_MASK | (0xFu << 10) | XPS_PR);
    reg(X_PORTSC) = ps | XPS_CCS | XPS_PED | XPS_PS_HS |
                    (PLS_U0 << XPS_PLS_SHIFT) | XPS_CSC | XPS_PRC;
    post_event(0, 0, COMP_SUCCESS << 24, TRB_PORT_EV << 10);
  }

  bool reset_taken() override {
    return !(reg(X_PORTSC) & (XPS_CSC | XPS_PRC));
  }

  void setup(const uint8_t pkt[8]) override {
    uint32_t lo, hi;
    memcpy(&lo, pkt, 4);
    memcpy(&hi, pkt + 4, 4);
    reg(X_EP_HALT) &= ~1u;                  // a SETUP clears a stalled EP0
    post_event(lo, hi, (seq_++ & 0xFFFF) | COMP_SUCCESS << 24,
               TRB_SETUP_EV << 10);
  }

  // The Setup event is on the ring; the data and status stages wait on the
  // TRBs the driver queues in answer.
  bool setup_taken() override { return true; }

  int in(int ep, uint8_t *buf, int max, bool *short_pkt) override {
    int n = transfer(ep ? 3 : 0, true, buf, max);
    if (n >= 0)
      *short_pkt = n < max;
    return n;
  }

  int out(int ep, const uint8_t *buf, int len) override {
    return transfer(ep ? 2 : 0, false, const_cast<uint8_t *>(buf), len);
  }

  uint32_t address() override { return (reg(X_CTRL) >> 24) & 0x7F; }

  void detach() override { connected_ = false; }

private:
  static constexpr uint32_t kSize = 0xA000; // XHCI regs, then PCI (+0x8000), DEV (+0x9000)
  uint32_t regs_[kSize / 4] = {};
  bool armed_ = false, connected_ = false;
  uint32_t enq_ = 0, ecs_ = 1, seq_ = 0;

  struct Ring {
    uint32_t deq = 0;
    uint32_t dcs = 0;
    bool valid = false;
  } rings_[4];

  uint32_t &reg(uint32_t off) { return regs_[(off % kSize) / 4]; }

  uint32_t ctx_addr(int dci) { return (reg(X_ECP) & ~0xFu) + dci * 64; }

  void load_ring(int dci) {
    uint32_t w2 = 0;
    if (!reg(X_ECP) || !mem_read(ctx_addr(dci) + 8, &w2, 4))
      return;
    rings_[dci].deq = w2 & ~0xFu;
    rings_[dci].dcs = w2 & 1;
    rings_[dci].valid = rings_[dci].deq != 0;
  }

  // Write an event at the enqueue pointer, with the producer cycle bit.
  void post_event(uint32_t w0, uint32_t w1, uint32_t w2, uint32_t w3) {
    if (!enq_)
      return;
    uint32_t trb[4] = {w0, w1, w2, (w3 & ~1u) | ecs_};
    mem_write(enq_, trb, sizeof(trb));
    uint32_t seg0 = reg(X_ERST0) & ~0xFu, seg1 = reg(X_ERST1) & ~0xFu;
    uint32_t n0 = reg(X_ERSTSZ) & 0xFFFF, n1 = reg(X_ERSTSZ) >> 16;
    enq_ += 16;
    if (enq_ == seg0 + n0 * 16) {
      enq_ = seg1;
    } else if (enq_ == seg1 + n1 * 16) {
      enq_ = seg0;
      ecs_ ^= 1;
    }
    reg(X_EREP) = enq_ | ecs_;
    reg(X_ST) |= XST_IP;
  }

  // Complete the TRB at ring `dci`'s dequeue pointer: move data into buf (to
  // the host) or out of it, post its Transfer event, advance. The TRB has to
  // match: on the control ring a Data or Status TRB of this direction, on a
  // bulk ring a Normal TRB.
  int transfer(int dci, bool to_host, uint8_t *buf, int len) {
    if (reg(X_EP_HALT) & (1u << dci))
      return XFER_STALL;
    Ring &r = rings_[dci];
    if (!r.valid)
      return XFER_NAK;
    uint32_t t[4] = {};
    for (int guard = 0; guard < 4; guard++) {
      if (!mem_read(r.deq, t, sizeof(t)) || (t[3] & 1) != r.dcs)
        return XFER_NAK;
      if (((t[3] >> 10) & 0x3F) != TRB_LINK)
        break;
      if (t[3] & 2)                         // toggle cycle
        r.dcs ^= 1;
      r.deq = t[0] & ~0xFu;
    }
    uint32_t type = (t[3] >> 10) & 0x3F;
    bool dir_in = (t[3] >> 16) & 1;
    UTRACE("ep %d %s: TRB @%08X type %u dir %u len %u\n", dci,
           to_host ? "IN" : "OUT", r.deq, type, dir_in, t[2] & 0x1FFFF);
    if (dci == 0 ? !((type == TRB_DATA || type == TRB_STATUS) && dir_in == to_host)
                 : type != TRB_NORMAL)
      return XFER_NAK;
    uint32_t size = type == TRB_STATUS ? 0 : t[2] & 0x1FFFF;
    uint32_t n = std::min<uint32_t>(size, (uint32_t)std::max(len, 0));
    if (n) {
      if (to_host)
        mem_read(t[0], buf, n);
      else
        mem_write(t[0], buf, n);
    }
    uint32_t left = size - n;
    post_event(r.deq, 0, left | (left ? COMP_SHORT_PKT : COMP_SUCCESS) << 24,
               TRB_TRANSFER_EV << 10 | (uint32_t)dci << 16);
    r.deq += 16;
    // Keep the context's dequeue pointer current, as the controller does.
    uint32_t w2 = r.deq | r.dcs;
    mem_write(ctx_addr(dci) + 8, &w2, 4);
    return (int)n;
  }
};

Xusbd g_xusbd;

} // namespace

// ---- Bus interface ----------------------------------------------------------

// bdk copies SETUP packets out of the queue heads with memcpy() and clears
// them with memset(), so byte and halfword accesses are real: a narrow read
// returns its lanes of the word, and a narrow write to the queue head memory
// merges into it. Registers take a narrow write shifted into place.
uint32_t usb2d_read(EmuState *state, uint64_t addr, unsigned size) {
  g_state = state;
  g_host.set_controllers(&g_usb2d, &g_xusbd);
  g_host.run();
  uint32_t off = (uint32_t)(addr - USB_OTG_BASE);
  uint32_t v = g_usb2d.read(off & ~3u);
  return size < 4 ? v >> ((off & 3) * 8) : v;
}

void usb2d_write(EmuState *state, uint64_t addr, unsigned size, uint32_t val) {
  g_state = state;
  g_host.set_controllers(&g_usb2d, &g_xusbd);
  uint32_t off = (uint32_t)(addr - USB_OTG_BASE);
  if (size < 4) {
    uint32_t shift = (off & 3) * 8;
    uint32_t mask = (size == 1 ? 0xFFu : 0xFFFFu) << shift;
    val = (val << shift) & mask;
    if (off >= 0x1000)
      val |= g_usb2d.read(off & ~3u) & ~mask;
  }
  g_usb2d.write(off & ~3u, val);
  g_host.run();
}

uint32_t xusbd_read(EmuState *state, uint64_t addr, unsigned size) {
  g_state = state;
  g_host.set_controllers(&g_usb2d, &g_xusbd);
  g_host.run();
  uint32_t off = (uint32_t)(addr - XUSB_DEV_BASE);
  uint32_t v = g_xusbd.read(off & ~3u);
  return size < 4 ? v >> ((off & 3) * 8) : v;
}

void xusbd_write(EmuState *state, uint64_t addr, unsigned size, uint32_t val) {
  g_state = state;
  g_host.set_controllers(&g_usb2d, &g_xusbd);
  uint32_t off = (uint32_t)(addr - XUSB_DEV_BASE);
  if (size < 4) {
    uint32_t shift = (off & 3) * 8;
    uint32_t mask = (size == 1 ? 0xFFu : 0xFFFFu) << shift;
    val = ((val << shift) & mask) | (g_xusbd.read(off & ~3u) & ~mask);
  }
  g_xusbd.write(off & ~3u, val);
  g_host.run();
}

void usb_reset(EmuState *state) {
  g_state = state;
  g_usb2d.full_reset();
  g_xusbd.reset_all();
  g_host.reset();
}
