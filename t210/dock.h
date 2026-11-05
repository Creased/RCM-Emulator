#ifndef T210_DOCK_H
#define T210_DOCK_H

#include <cstdint>

struct EmuState;

/*
 * The USB-C port, the dock on it and the TV behind the dock.
 *
 * BM92T36 (I2C1 0x18): the PD controller's registers and commands as
 * switchroot's bm92txx.c and HOS's usb sysmodule drive them, with a partner
 * on the cable that answers the way the measured docks did: a Nintendo dock
 * (057E:2003, 5 V 0.5 A + 15 V 3 A, Nintendo alt mode, DP pin D), a third-
 * party adapter (Fresco Logic 1D5C:7102, four PDOs, DisplayPort SVID) or
 * the Nintendo charger (057E:2004, no video). Its interrupt is GPIO PK4,
 * active low; its reset is PV5.
 *
 * DPAUX1 (0x54040000): the AUX channel to the dock's DisplayPort-to-HDMI
 * converter (Realtek OUI 00-E0-4C, DPCD 1.2), its DPCD, and the TV's EDID
 * over I2C-over-AUX. HPD reaches DPAUX1 only while the hdmi_int_dp_hpd pad
 * is in SFIO mode, as measured.
 *
 * SOR1 (0x54580000): link training is judged lane by lane against what the
 * board does - DP lane 0 leaves the SoC on pad 0 and lane 1 on pad 1, so the
 * crossbar has to route them there (L4T's nvidia,xbar-ctrl <2 1 0 3 4>), and
 * a lane only trains with its pad powered, the macro PLL up, PLLDP locked
 * and SOR1 clocked from the brick at the rate the sink was told.
 */

enum DockKind : uint8_t {
  DOCK_NONE = 0,
  DOCK_NINTENDO = 1,
  DOCK_THIRD_PARTY = 2,   // Ugreen-style adapter: Fresco Logic 1D5C:7102
  DOCK_CHARGER = 3,       // Nintendo charger, no DisplayPort
};

void dock_reset(EmuState *state);
// Advance the dock's timers to state->emu_usec. Cheap; called often.
void dock_tick(EmuState *state);

// BM92T36. A write carries its register first; a read returns n bytes from
// `reg` on (block registers lead with their length byte, as on the chip).
bool bm92_present(EmuState *state);
void bm92_write(EmuState *state, const uint8_t *buf, uint32_t n);
void bm92_read(EmuState *state, uint8_t reg, uint8_t *out, uint32_t n);
// PK4 as the SoC sees it: false while an enabled alert is pending.
bool bm92_alert_line(EmuState *state);
// PV5 driven by the SoC: high holds the chip in reset.
void bm92_reset_line(EmuState *state, bool high);

// HPD from the dock's DisplayPort sink.
bool dock_hpd(EmuState *state);

// DPAUX1 and SOR1 register windows (byte offsets).
uint32_t dpaux1_read(EmuState *state, uint32_t off);
void     dpaux1_write(EmuState *state, uint32_t off, uint32_t val);
uint32_t sor1_read(EmuState *state, uint32_t off);
void     sor1_write(EmuState *state, uint32_t off, uint32_t val);

// What the TV shows, for its window.
struct TvState {
  bool connected;        // a dock with a TV behind it is on the cable
  bool hpd;              // the converter raised HPD to the SoC
  bool link_trained;     // every configured lane has CR, EQ, symbol lock
  bool video;            // SOR1 attached to a running head: frames flow
  uint32_t lanes, rate_code;
  char status[160];      // one line for the "no signal" screen
};
void dock_tv_state(EmuState *state, TvState *out);

#endif // T210_DOCK_H
