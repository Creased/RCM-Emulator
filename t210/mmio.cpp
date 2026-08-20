#include "mmio.h"
#include "../emu_state.h"
#include "i2c3.h"
#include "memory_map.h"
#include "pcie.h"
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

// Standard CRC32 (poly 0xEDB88320), matching hekate's crc32_calc(0, ...) which
// the payload uses to validate the GPT. Init/final via ~ like the reference.
static uint32_t emu_crc32(const uint8_t *buf, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int b = 0; b < 8; b++)
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
  }
  return ~crc;
}

// Synthesize a minimal but spec-valid Switch eMMC GPT so hwtest's [eMMC GPT]
// probe reads "EFI PART" with matching header/entry CRC32s instead of
// "signature missing" when no --rawnand image is loaded. Returns a buffer
// covering LBA 0..2 (protective-MBR sector left zero); header at LBA 1
// (offset 512), 128-byte entries at LBA 2 (offset 1024). Byte offset into the
// buffer == byte offset into the GPP, so the read path can copy directly.
static const uint8_t *emmc_synth_gpt(size_t *out_len) {
  // 6 sectors: LBA 0 (MBR) + LBA 1 (header) + LBA 2..4 (11*128 = 1408 B of
  // entries spans three sectors). Undersizing this overflows the buffer.
  static uint8_t gpt[6 * 512];
  static bool built = false;
  *out_len = sizeof(gpt);
  if (built) return gpt;
  memset(gpt, 0, sizeof(gpt));

  struct Part { const char *name; uint64_t first, last; };
  static const Part parts[] = {
      {"PRODINFO", 34, 8191},                 {"PRODINFOF", 8192, 16383},
      {"BCPKG2-1-Normal-Main", 16384, 32767}, {"BCPKG2-2-Normal-Sub", 32768, 49151},
      {"BCPKG2-3-SafeMode-Main", 49152, 65535},{"BCPKG2-4-SafeMode-Sub", 65536, 81919},
      {"BCPKG2-5-Repair-Main", 81920, 98303}, {"BCPKG2-6-Repair-Sub", 98304, 114687},
      {"SAFE", 114688, 245759},               {"SYSTEM", 245760, 5488639},
      {"USER", 5488640, 60014591},
  };
  const uint32_t nparts = sizeof(parts) / sizeof(parts[0]);

  uint8_t *entries = gpt + 2 * 512; // LBA 2
  for (uint32_t i = 0; i < nparts; i++) {
    uint8_t *e = entries + i * 128;
    memset(e + 0x00, 0x11, 16);         // partition type GUID (non-zero => used)
    memset(e + 0x10, 0x20 + i, 16);     // unique partition GUID
    *(uint64_t *)(e + 0x20) = parts[i].first;
    *(uint64_t *)(e + 0x28) = parts[i].last;
    for (uint32_t c = 0; parts[i].name[c] && c < 36; c++) // name UTF-16LE
      *(uint16_t *)(e + 0x38 + c * 2) = (uint16_t)parts[i].name[c];
  }
  uint32_t ent_crc = emu_crc32(entries, nparts * 128);

  uint8_t *hdr = gpt + 1 * 512; // LBA 1
  memcpy(hdr + 0x00, "EFI PART", 8);
  *(uint32_t *)(hdr + 0x08) = 0x00010000; // revision 1.0
  *(uint32_t *)(hdr + 0x0C) = 0x5C;       // header size 92
  *(uint32_t *)(hdr + 0x10) = 0;          // header CRC (filled after)
  *(uint64_t *)(hdr + 0x18) = 1;          // my LBA
  *(uint64_t *)(hdr + 0x20) = 60030975;   // alternate (backup) LBA
  *(uint64_t *)(hdr + 0x28) = 34;         // first usable LBA
  *(uint64_t *)(hdr + 0x30) = 60014591;   // last usable LBA
  memset(hdr + 0x38, 0xAB, 16);           // disk GUID
  *(uint64_t *)(hdr + 0x48) = 2;          // partition entry LBA
  *(uint32_t *)(hdr + 0x50) = nparts;     // number of entries
  *(uint32_t *)(hdr + 0x54) = 128;        // size of each entry
  *(uint32_t *)(hdr + 0x58) = ent_crc;    // entry-array CRC32
  *(uint32_t *)(hdr + 0x10) = emu_crc32(hdr, 0x5C); // header CRC32 (field zeroed)

  built = true;
  return gpt;
}

/*
 * Central MMIO dispatcher.
 *
 * Unicorn hooks for unmapped memory access route here.
 * We dispatch to the appropriate peripheral handler based on address range.
 */

// ==================== PINMUX ====================
//
// Pad control registers read back whatever the payload last wrote (the write
// hook caches every store), but a pad nobody has touched still has a reset
// value, and probes read those to prove a pad is where the BootROM left it.
// Only the pads a probe actually samples are listed; everything else keeps
// reading 0, which is the honest "not modelled" answer.
struct PinmuxDefault { uint16_t off; uint32_t val; };
static const PinmuxDefault pinmux_defaults[] = {
    // PH5 / BT_HOST_WAKE: E_INPUT | PARKED | TRISTATE | PULL_DOWN. Measured
    // as 0x0074 at payload entry on all four reference consoles.
    {0x1C8, 0x00000074},
};

static uint32_t pinmux_reset_default(uint64_t addr) {
  uint32_t off = (uint32_t)(addr - PINMUX_BASE);
  for (size_t i = 0; i < sizeof(pinmux_defaults) / sizeof(pinmux_defaults[0]); i++)
    if (pinmux_defaults[i].off == off)
      return pinmux_defaults[i].val;
  return 0;
}

// PINMUX_AUX bits 3:2 select the pad's internal pull.
enum PadPull { PAD_PULL_NONE = 0, PAD_PULL_DOWN = 1, PAD_PULL_UP = 2 };

static uint32_t pinmux_pull(uint64_t pinmux_addr) {
  uint32_t v = mmio_regs.count(pinmux_addr) ? mmio_regs[pinmux_addr]
                                            : pinmux_reset_default(pinmux_addr);
  return (v >> 2) & 3;
}

// ==================== UART ====================
//
// Five 16550-style controllers whose register blocks are NOT evenly spaced:
// bdk's soc/uart.c carries the offset table { 0, 0x40, 0x200, 0x300, 0x400 }.
// The dispatcher used to derive the port with (addr - UART_A) / 0x40, which
// is only correct for A and B -- it puts C at 8, D at 12 and E at 16, all
// past the end of every per-port array, so UART-C/D/E silently had no receive
// FIFO, no TX capture and a hard-wired LSR.
//
// The index order is A=0..E=4. UART_B must stay 1: the payload's debug log,
// the `[uartB]` stdout mirror, console_window's default port and
// input_script's receive FIFO all address it by that number.
static const uint32_t uart_bases[EmuState::N_UARTS] = {
    0x70006000, 0x70006040, 0x70006200, 0x70006300, 0x70006400};

#define UART_D 3

// Per-port electrical state the plain register cache cannot express.
struct UartPort {
  uint16_t divisor   = 0;      // latched DLL/DLM, i.e. the programmed baud
  uint8_t  mcr       = 0;      // last MCR write; bit 4 = internal loopback
  uint8_t  msr_delta = 0;      // MSR bits 3:0: set on change, cleared on read
  bool     cts       = false;  // peer drove its RTS_N low -> MSR bit 4
};
static UartPort uart_ports[EmuState::N_UARTS];

// Port index for an address inside one of the five 0x40-byte blocks, or -1
// for the gaps between them.
static int uart_port_of(uint64_t addr, uint32_t *offset) {
  for (int p = 0; p < (int)EmuState::N_UARTS; p++) {
    if (addr >= uart_bases[p] && addr < uart_bases[p] + 0x40) {
      *offset = (uint32_t)(addr - uart_bases[p]);
      return p;
    }
  }
  return -1;
}

// ==================== Bluetooth radio (Broadcom CYW4356) ====================
//
// The Switch's combo radio presents its Bluetooth core over two interfaces,
// and everything a payload can observe about it goes through one of them:
//
//   UART-D 0x70006300   H4/HCI at 115200 8N1, plus the chip's RTS_N arriving
//                       on the Tegra's CTS input as UART_MSR bit 4.
//   GPIO port H         PH1 WL_REG_ON     host output, shares the module CBUCK
//                       PH3 BT_DEV_WAKE   host output; in UART transport the
//                           chip never drives it (the same ball is SPI_INT, a
//                           chip output, only once the SPI strap has latched)
//                       PH4 BT_REG_ON     host output; LOW->HIGH is the POR
//                       PH5 BT_HOST_WAKE  chip side; a live part holds it high
//                       PH7 BT_GPIO5      host output
//
// The chip itself is three states driven entirely by BT_REG_ON:
//
//   OFF --(PH4 rises)--> POR --(~110 ms)--> READY --(PH4 falls)--> OFF
//
// POR is not instant: the internal PMU has to bring VDDC up behind the
// module's own CBUCK, and the datasheet allows up to 110 ms after the rails
// cross threshold -- rails that only start moving at the edge. Only in READY
// does the part drive RTS_N low and answer HCI, which is why the payload
// waits 200 ms after the edge before it concludes anything.
//
// A faulty module never gets there, and that is the entire difference between
// the three reference consoles that answer and the one that returns 2110-1118
// in HOS: BT_HOST_WAKE reads low, RTS_N never asserts, and not one byte comes
// back -- while the SoC side (loopback, clocks, pads, PMIC) tests perfectly.
enum BtPhase { BT_PHASE_OFF, BT_PHASE_POR, BT_PHASE_READY };

// Datasheet worst case for POR completion after the BT_REG_ON edge.
#define BT_POR_US 110000ull
// DLL for 115200 off PLLP_OUT0/2: (8*115200 + 408000000) / (16*115200) = 221.
// The chip only speaks its own rate, so gating the responder on this makes
// the payload's host-baud sweep silent for free -- exactly as on hardware.
#define BT_HCI_DIVISOR 221
// uart4_rx_pi5 pad control, sampled to decide whether an unfitted module
// leaves the receive line in a break condition.
#define UART_D_RX_PINMUX (PINMUX_BASE + 0x118)

struct BtChip {
  BtPhase  phase  = BT_PHASE_OFF;
  bool     reg_on = false;       // last sampled BT_REG_ON level
  uint64_t por_us = 0;           // emu_usec at the rising edge
  uint8_t  cmd[4 + 255];         // H4 command being assembled, host -> chip
  uint32_t cmd_n  = 0;
};
static BtChip bt_chip;

static bool bt_radio_fitted(EmuState *state) {
  return state && state->bt_radio.load() != BT_RADIO_ABSENT;
}

static bool bt_radio_alive(EmuState *state) {
  return state && state->bt_radio.load() == BT_RADIO_HEALTHY;
}

// Advance the power-up state machine. Called lazily from every access that
// could observe it rather than from a timer: emu_usec only moves while the
// CPU runs, so "now" is always current at the point of a register access.
static void bt_chip_tick(EmuState *state) {
  if (bt_chip.phase == BT_PHASE_POR &&
      state->emu_usec - bt_chip.por_us >= BT_POR_US)
    bt_chip.phase = BT_PHASE_READY;
}

// Re-evaluate the chip's RTS_N, which the host sees as UART_MSR bit 4. Only a
// healthy part that has finished POR asserts it; a change latches DCTS.
static void bt_uart_sync(EmuState *state) {
  bt_chip_tick(state);
  UartPort &up = uart_ports[UART_D];
  bool cts = bt_radio_alive(state) && bt_chip.phase == BT_PHASE_READY;
  if (cts != up.cts) {
    up.cts = cts;
    up.msr_delta |= 0x01; // DCTS
  }
}

// BT_HOST_WAKE (PH5) as the module drives it, for the window where the Tegra
// has the pad high-Z with no pull of its own. A live module holds it high
// through its own pull-up whether or not BT_REG_ON has been pulsed yet --
// which is what the good captures show while both REG_ON pins are still low,
// and it is the single cleanest good/bad discriminator in the whole probe.
static bool bt_host_wake(EmuState *state) { return bt_radio_alive(state); }

// Sticky LSR error bits contributed by the line itself. With no module fitted
// nothing drives BT_UART_TXD, so the Tegra's own pull-down on that pad holds
// the line at 0 -- a permanent break, which a 16550 latches as BRK | FERR. A
// module that is merely dead still parks its TXD as an input with an internal
// pull-up (datasheet p93 Table 29), so the line idles high and LSR stays
// clean: that is exactly what the faulty reference console reports, and it is
// the one place where "absent" and "faulty" differ from the register side.
static uint32_t bt_line_lsr_bits(EmuState *state, int port) {
  if (port != UART_D || bt_radio_fitted(state))
    return 0;
  if (pinmux_pull(UART_D_RX_PINMUX) != PAD_PULL_DOWN)
    return 0;
  return 0x18; // BRK | FERR
}

static void bt_queue(EmuState *state, const uint8_t *ev, size_t n) {
  for (size_t i = 0; i < n; i++)
    state->uart_rx_fifo[UART_D].push_back(ev[i]);
}

// One complete HCI command has arrived. Broadcom parts run their lower-layer
// stack out of on-die ROM, so a bare HCI_Reset is answered long before any
// firmware download -- which is the whole reason this probe works at all.
static void bt_chip_command(EmuState *state, uint16_t opcode) {
  switch (opcode) {
  case 0x0C03: { // HCI_Reset
    static const uint8_t cc[] = {0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00};
    bt_queue(state, cc, sizeof(cc));
    break;
  }
  case 0x1001: { // Read_Local_Version_Information
    // [0]type [1]evt 0x0E [2]plen [3]ncmd [4..5]opcode [6]status [7]hci_ver
    // [8..9]hci_rev [10]lmp_ver [11..12]manufacturer [13..14]lmp_subver,
    // the two 16-bit fields little-endian. Values are what all three good
    // reference consoles report; hci_rev is the one field the probe does not
    // print, so it is a plausible build number rather than a measurement.
    static const uint8_t cc[] = {
        0x04, 0x0E, 0x0C, 0x01, 0x01, 0x10, 0x00,
        0x08,       // HCI version 8 = Bluetooth 5.0
        0x48, 0x02, // HCI revision (not sampled by the probe)
        0x08,       // LMP version 8
        0x0F, 0x00, // manufacturer 0x000F = Broadcom
        0x09, 0x24, // lmp_subver 0x2409 = BCM4356
    };
    bt_queue(state, cc, sizeof(cc));
    break;
  }
  default: { // Unknown Command, so the H4 stream stays framed either way.
    const uint8_t cc[] = {0x04, 0x0E, 0x04, 0x01, (uint8_t)opcode,
                          (uint8_t)(opcode >> 8), 0x01};
    bt_queue(state, cc, sizeof(cc));
    break;
  }
  }
}

// One byte from the host. Nothing is ever emitted unsolicited: the payload
// counts what it has to flush before talking and every good capture shows
// `junk=0`, so a chatty model would break the very line it is meant to match.
static void bt_chip_rx(EmuState *state, uint8_t b) {
  bt_uart_sync(state);
  if (!bt_radio_alive(state) || bt_chip.phase != BT_PHASE_READY ||
      uart_ports[UART_D].divisor != BT_HCI_DIVISOR) {
    bt_chip.cmd_n = 0;
    return;
  }
  if (bt_chip.cmd_n == 0 && b != 0x01)
    return; // resync on the H4 command indicator
  if (bt_chip.cmd_n < sizeof(bt_chip.cmd))
    bt_chip.cmd[bt_chip.cmd_n++] = b;
  if (bt_chip.cmd_n < 4)
    return; // 01 <opcode lo> <opcode hi> <plen>
  if (bt_chip.cmd_n < 4u + bt_chip.cmd[3])
    return;
  uint16_t opcode = (uint16_t)bt_chip.cmd[1] | ((uint16_t)bt_chip.cmd[2] << 8);
  bt_chip.cmd_n = 0;
  bt_chip_command(state, opcode);
}

// BT_REG_ON (PH4) moved. The rising edge is the chip's power-on reset; the
// falling edge drops everything, which is what makes each of the payload's
// three arms a genuine cold power cycle rather than a warm poke -- the
// transport strap is latched once per POR and never re-evaluated.
static void bt_reg_on_set(EmuState *state, bool level) {
  if (level == bt_chip.reg_on)
    return;
  bt_chip.reg_on = level;
  if (level) {
    bt_chip.phase  = BT_PHASE_POR;
    bt_chip.por_us = state->emu_usec;
  } else {
    bt_chip.phase = BT_PHASE_OFF;
  }
  bt_chip.cmd_n = 0;
  state->uart_rx_fifo[UART_D].clear();
  bt_uart_sync(state);
  printf("[bt] BT_REG_ON %s (%s radio)\n", level ? "HIGH - POR" : "LOW - off",
         bt_radio_name(state->bt_radio.load()));
}

// ==================== GPIO ====================
// Button state is stored in EmuState and read via GPIO registers.
// VOL_UP = GPIO_X6 (port X, pin 6), VOL_DOWN = GPIO_X7, POWER = GPIO_X0 (PMC)

// PWM controller channel 1 drives the cooling fan (bdk t210.h).
#define PWM_CSR_1_OFF 0x10

// ---- GPIO port H -----------------------------------------------------------
//
// Port H is index 7, i.e. bank 1 slot 3: CNF 0x10C, OE 0x11C, OUT 0x12C,
// IN 0x13C. It is the only port whose input side is resolved per pin rather
// than mirrored from OUT, because it is the only one where the payload's
// conclusions depend on what happens when it deliberately lets go of a pad.
#define GPIO_H_CNF (GPIO_BASE + 0x10C)
#define GPIO_H_OE  (GPIO_BASE + 0x11C)
#define GPIO_H_OUT (GPIO_BASE + 0x12C)
#define GPIO_H_IN  (GPIO_BASE + 0x13C)

// PINMUX_AUX offset per port-H pin, 0 where the ball is not modelled. These
// are not contiguous: the pad control registers are ordered by ball name, not
// by GPIO port, so PH6 (the right Joy-Con attach detect) sits elsewhere
// entirely and PH2/PH6 are simply not needed here.
static const uint16_t gpio_h_pinmux[8] = {
    0x000, 0x1B8, 0x000, 0x1C0, 0x1C4, 0x1C8, 0x000, 0x1CC};

static uint32_t gpio_reg(uint64_t addr) {
  return mmio_regs.count(addr) ? mmio_regs[addr] : 0;
}

// Level of a port-H pin the Tegra is NOT driving: the pad simply follows
// whatever is pulling it, ours or the other end of the trace.
static int gpio_h_float(EmuState *state, int pin) {
  uint16_t pmx = gpio_h_pinmux[pin];
  if (pmx) {
    switch (pinmux_pull(PINMUX_BASE + pmx)) {
    case PAD_PULL_DOWN: return 0;
    case PAD_PULL_UP:   return 1;
    default:            break;
    }
  }
  // No Tegra pull, so whatever is on the far end of the trace decides.
  if (pin == 5)
    return bt_host_wake(state) ? 1 : 0; // BT_HOST_WAKE, chip-side pull-up
  if (pin == 6)
    return 1; // Joy-Con right attach: active low, and nothing is plugged in
  return 0;
}

static uint32_t gpio_h_in(EmuState *state) {
  uint32_t cnf = gpio_reg(GPIO_H_CNF);
  uint32_t oe  = gpio_reg(GPIO_H_OE);
  uint32_t out = gpio_reg(GPIO_H_OUT);
  uint32_t in  = 0;
  for (int pin = 0; pin < 8; pin++) {
    uint32_t m = 1u << pin;
    // A pad only reaches the GPIO controller at all while its CNF bit selects
    // GPIO over the muxed function, and it only reads back its own level
    // while output-enable is set. Anything else is an input.
    int level = ((cnf & m) && (oe & m)) ? ((out & m) ? 1 : 0)
                                        : gpio_h_float(state, pin);
    if (level)
      in |= m;
  }
  return in;
}

// Re-sample the two REG_ON straps after anything that could have moved port
// H. PH4 is BT_REG_ON, PH1 is WL_REG_ON - one per half of the CYW4356.
static void gpio_h_update(EmuState *state) {
  uint32_t driven = gpio_reg(GPIO_H_CNF) & gpio_reg(GPIO_H_OE) &
                    gpio_reg(GPIO_H_OUT);
  bt_reg_on_set(state, (driven & (1u << 4)) != 0);
  pcie_wl_reg_on(state, (driven & (1u << 1)) != 0);
}

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

  // Port Z IN. Bit 1 active-low = SD card detect.
  //
  // The offset is (port>>2)*0x100 + (port&3)*4 + 0x30; port Z is 25, so
  // 0x600 + 0x04 + 0x30 = 0x634. This used to answer at 0x61C - which is
  // bank 6's OE slot, not an IN register - so the read fell through to the
  // generic path and the "SD card ejected" tweak never reached the payload.
  // (The default happened to look right: an unhandled read returned 0, and
  // the signal is active-low, i.e. "inserted".)
  if (offset == 0x634) {
    uint32_t val = state->sd_inserted.load() ? 0x00 : (1u << 1);
    // PZ4 is the audio codec's LDO1 enable (device tree:
    // realtek,ldo1-en-gpios). A payload drives it as an output and then reads
    // the pad back to tell "I drove it and nothing answered" from "the pad
    // never moved" - so the input register has to reflect what the output
    // register is driving, or that check reports a dead pin on a healthy
    // console. Mirror OUT (port Z is 0x624) into IN for that pin.
    uint32_t out = mmio_regs.count(GPIO_BASE + 0x624)
                       ? mmio_regs[GPIO_BASE + 0x624] : 0;
    val |= out & (1u << 4);
    return val;
  }

  // Port S IN (port 18 -> 0x400 + 0x08 + 0x30). Pin 7 is the cooling-fan
  // tachometer.
  //
  // The fan puts out two pulses per revolution and a payload counts edges
  // over a sampling window: rpm = edges / 4 * (60000 / window_ms). A real
  // Mariko driven at duty 150 reports ~6750 RPM, i.e. 183 edges in 400 ms,
  // so an edge every ~2186 us. Without this the line never moved, 0 RPM came
  // back, and a fan test concluded "fan dead" on a healthy console.
  //
  // Only toggle while the PWM channel is actually driving the fan, so a
  // payload that checks "duty 0 -> should read 0 RPM" still sees that.
  if (offset == 0x438) {
    uint32_t csr = mmio_regs.count(PWM_BASE + PWM_CSR_1_OFF)
                       ? mmio_regs[PWM_BASE + PWM_CSR_1_OFF] : 0;
    bool ch_en   = (csr & (1u << 31)) != 0;
    bool abs_off = (csr & (1u << 24)) != 0;
    uint32_t inv_duty = (csr >> 16) & 0xFF;   // inverted: 236 = ~0%
    bool spinning = ch_en && !abs_off && inv_duty < 236;
    if (spinning && ((state->emu_usec / 2186ull) & 1ull))
      return (1u << 7);
    return 0;
  }

  // Port H IN. Resolved per pin (see gpio_h_in) instead of mirroring OUT,
  // because this is the port the Bluetooth probe measures by RELEASING pads:
  // PH5/BT_HOST_WAKE and PH3/BT_DEV_WAKE are read while the Tegra is
  // deliberately not driving them, and mirroring OUT there answers 0 to every
  // question regardless of what is on the board.
  if (offset == 0x13C) {
    uint32_t v = gpio_h_in(state);
    printf("[gpio] R: Port H IN = 0x%02X\n", v);
    return v;
  }

  // Tegra X1 GPIO is organized as 8 banks of 256 bytes; within each
  // bank a port occupies a 4-byte slot at offsets:
  //   CNF  +0x00..0x0F    (GPIO vs SPIO mode select per pin)
  //   OE   +0x10..0x1F    (output enable per pin)
  //   OUT  +0x20..0x2F    (driven value per pin)
  //   IN   +0x30..0x3F    (sampled value per pin)
  // The write hook stores every word into mmio_regs already, so reads
  // for CNF / OE / OUT just hand back what the payload last wrote.
  // For IN we don't model external drivers, so we mirror the latched
  // OUT value (one slot earlier, i.e. addr - 0x10) -- that makes
  //   gpio_write(PORT, PIN, HIGH); gpio_read(PORT, PIN);
  // round-trip for ports the payload drives itself (PV0/PV1 backlight
  // enable, PV2 panel reset, PK3 Joy-Con charge enable, PA5 5V rail
  // enable, etc.).
  uint32_t bank_off = offset & 0xFF;
  if (offset < 0x800) {
    if (bank_off < 0x30) {
      // CNF / OE / OUT: hand back the cached write.
      uint32_t v = mmio_regs.count(addr) ? mmio_regs[addr] : 0;
      printf("[gpio] R: offset 0x%X (CNF/OE/OUT cache) = 0x%08X\n", offset, v);
      return v;
    }
    if (bank_off < 0x40) {
      // IN: mirror of the matching OUT one slot earlier.
      uint64_t out_addr = addr - 0x10;
      uint32_t v = mmio_regs.count(out_addr) ? mmio_regs[out_addr] : 0;
      printf("[gpio] R: offset 0x%X (IN, mirror of OUT @ 0x%X) = 0x%08X\n",
             offset, (uint32_t)(offset - 0x10), v);
      return v;
    }
    // MSK_CNF 0x80 / MSK_OE 0x90 / MSK_OUT 0xA0: aliases that read back the
    // plain register they write through, 0x80 lower.
    if (bank_off >= 0x80 && bank_off < 0xB0) {
      uint32_t v = mmio_regs.count(addr - 0x80) ? mmio_regs[addr - 0x80] : 0;
      printf("[gpio] R: offset 0x%X (masked alias of 0x%X) = 0x%08X\n", offset,
             (uint32_t)(offset - 0x80), v);
      return v;
    }
  }

  printf("[gpio] R: offset 0x%X = 0\n", offset);
  return 0;
}

void gpio_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - GPIO_BASE);
  uint32_t bank_off = offset & 0xFF;

  // Masked writes: one store of (pins << 8) | value that moves only the named
  // pins. The controller applies them to the plain CNF / OE / OUT register
  // 0x80 lower, and that plain register is what every reader -- including
  // this model -- looks at. Caching the raw masked word (all the write hook
  // used to do) meant a payload that drives a pin exclusively through the
  // masked aliases never moved the pin at all: the Bluetooth probe writes
  // port H that way and nothing else, so PH4/BT_REG_ON never went high and
  // the probe reported "write did not latch".
  if (offset < 0x800 && bank_off >= 0x80 && bank_off < 0xB0) {
    uint64_t plain = addr - 0x80;
    uint32_t mask  = (val >> 8) & 0xFF;
    uint32_t cur   = mmio_regs.count(plain) ? mmio_regs[plain] : 0;
    mmio_regs[plain] = (cur & ~mask) | (val & mask);
    printf("[gpio] W: offset 0x%X masked -> 0x%X = 0x%08X\n", offset,
           (uint32_t)(offset - 0x80), mmio_regs[plain]);
  } else {
    printf("[gpio] W: offset 0x%X = 0x%08X\n", offset, val);
  }

  // Bank 1 holds ports E..H, and port H carries BT_REG_ON, so re-sample the
  // radio's power pin after anything that could have moved it.
  if (offset >= 0x100 && offset < 0x200)
    gpio_h_update(state);
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
// Last full CMD_DATA1 word, committed when I2C_CNFG starts the transfer.
static uint32_t i2c_cmd_data1 = 0;

// ALC5639 codec register file (slave 0x1C on I2C_1). Sparse, because the
// payload's init table touches only a few dozen of the 256 registers and
// every one it reads back is one it wrote. Values are stored the natural way
// round; the byte swap onto the wire happens at the read site.
static std::map<uint8_t, uint16_t> codec_regs;

static uint16_t codec_reg_get(uint8_t reg) {
  auto it = codec_regs.find(reg);
  return it == codec_regs.end() ? 0 : it->second;
}

// Which slaves actually answer, per bus. This is the emulator's answer to
// "is that chip fitted?", so it must list exactly what is modelled below and
// nothing else - a bus scan is a real diagnostic and an emulator that ACKs
// every address turns it into noise.
//
// Deliberately absent, because they are absent on the board this models:
//   I2C_1 0x1A  TC94B15WBG headphone amp - not fitted on Erista; hwtest
//               reports "not fitted on this board" and that is correct.
#define I2C_STATUS_NOACK (0xFu << 0)

static bool i2c_slave_present(EmuState *state, bool on_i2c5, uint8_t addr) {
  if (on_i2c5) {
    switch (addr) {
    case 0x1B:                    // MAX77621 CPU DC-DC
    case 0x33:                    // RTC / PMIC sub-block
    case MAX77620_I2C_ADDR:       // 0x3C PMIC
    case 0x68:                    // MAX77812 / GPU rail
      return true;
    default:
      return false;
    }
  }
  switch (addr) {
  case 0x18:                      // BM92T36 USB-PD
  case 0x36:                      // MAX17050 fuel gauge
  case 0x4C:                      // TMP451 thermal
  case 0x6B:                      // BQ24193 charger
    return true;
  case 0x1C:                      // ALC5639 codec - only once LDO1_EN is up
    return (mmio_regs.count(GPIO_BASE + 0x624)
                ? mmio_regs[GPIO_BASE + 0x624] : 0) & (1u << 4);
  default:
    (void)state;
    return false;
  }
}
// I2C_CNFG has to read back what was written: bdk sets NORMAL_MODE_GO with a
// read-modify-write (`cnfg = (cnfg & ~GO) | GO`), so returning 0 would drop the
// transfer size and direction bits before the transaction runs. [0]=I2C1, [1]=I2C5.
static uint32_t i2c_cnfg_reg[2] = {0, 0};

// (Touch controller is the STMFTS at I2C_3 / slave 0x49, handled separately
// in t210/i2c3.cpp. Nothing else lives at I2C_1 / slave 0x4C besides TMP451.)

#define MAX77620_REG_ONOFFSTAT 0x15
#define MAX77620_ONOFFSTAT_EN0 BIT(2)

// ---- MAX77620 register file ----
//
// Modelled as a real 256-byte register file rather than a handful of hardcoded
// reads, so that:
//   * rail voltages decode to the values max77620_config_default() programs
//     (bdk power/max7762x.c _pmic_regulators, uv_default column) instead of
//     everything reading back as the 0.600 V register-zero floor, and
//   * writes stick, so a payload that steps a rail and reads it back sees its
//     own value (the "voltage set/read" / "not settable" tests).
//
// Encoding: volt = (reg & mask) * step_uv + base_uv.
//   SD0..SD3   regs 0x16..0x19, step 12.5 mV, base 600 mV
//   LDO0..LDO8 regs 0x23,0x25,..,0x33, low 6 bits, step 25 mV (LDO0/1/4) or
//              50 mV (rest), base 800 mV; bits 7:6 = power mode (3 = normal).
// Rail-OK status: SD rails via STATSD (0x14, bit set = NOT ok), LDO rails via
// each LDOx_CFG2 POK bit (BIT(3)).
static uint8_t max77620_regs[256];
static bool max77620_regs_ready = false;

static void max77620_regs_init(EmuState *state) {
  if (max77620_regs_ready)
    return;
  max77620_regs_ready = true;

  // Two SD rails differ per SoC generation, measured on real consoles:
  //   SD1 (DRAM)    1.125 V Erista (LPDDR4)  vs 1.100 V Mariko (LPDDR4X)
  //   SD2 (LDO src) 1.350 V Erista           vs 1.325 V Mariko
  // bdk's _pmic_regulators lists 1.125/1.325 as uv_default for both, which is
  // not what either console actually runs; model the measured values so the
  // emulator can't "confirm" a wrong expectation.
  bool mariko = state && state->pmic_otp.load() == 0x53;
  uint32_t sd1_uv = mariko ? 1100000u : 1125000u;
  uint32_t sd2_uv = mariko ? 1325000u : 1350000u;

  // SD0 idle, measured: 1.050 V Mariko, 1.125 V Erista.
  max77620_regs[0x16] = (uint8_t)(((mariko ? 1050000u : 1125000u) - 600000u) / 12500u);
  max77620_regs[0x17] = (uint8_t)((sd1_uv - 600000u) / 12500u);    // SD1
  max77620_regs[0x18] = (uint8_t)((sd2_uv - 600000u) / 12500u);    // SD2
  max77620_regs[0x19] = (uint8_t)((1800000u - 600000u) / 12500u);  // SD3
  max77620_regs[0x14] = 0x00; // STATSD: 0 = every SD rail in regulation

  // Per-rail configured voltage and whether the rail is actually UP in RCM.
  //
  // The `on` column matters: a real Mariko boots with most LDOs DOWN - only
  // the display (LDO0), SDMMC1 (LDO2) and RTC (LDO4) rails are up before HOS
  // or a payload enables the rest. Modelling every LDO as on (the old
  // behaviour) meant the emulator judged rail voltages a real console never
  // presents at that point, e.g. LDO6 (touch/ALS) only comes up when a probe
  // calls touch_power_on().
  //
  // Both columns are measured on real consoles now. Mariko brings up
  // LDO0/2/4; Erista also has LDO7 (XUSB) up.
  struct LdoDef { uint8_t volt_reg; uint32_t uv; uint32_t step; bool on; };
  static const LdoDef ldos_mariko[] = {
      {0x23, 1200000, 25000, true },  // LDO0 display
      {0x25, 1050000, 25000, false},  // LDO1 XUSB/PCIE
      {0x27, 1800000, 50000, true },  // LDO2 SDMMC1
      {0x29, 3100000, 50000, false},  // LDO3 GC ASIC
      {0x2B,  800000, 12500, true },  // LDO4 RTC
      {0x2D, 3100000, 50000, false},  // LDO5 GC card
      {0x2F, 2800000, 50000, false},  // LDO6 touch + ALS
      {0x31, 1000000, 50000, false},  // LDO7 XUSB
      {0x33, 1000000, 50000, false},  // LDO8 XUSB/DP
  };
  static const LdoDef ldos_erista[] = {
      {0x23, 1200000, 25000, true },  // LDO0 display
      {0x25, 1050000, 25000, false},  // LDO1 XUSB/PCIE
      {0x27, 1800000, 50000, true },  // LDO2 SDMMC1
      {0x29, 3100000, 50000, false},  // LDO3 GC ASIC
      {0x2B, 1000000, 12500, true },  // LDO4 RTC (1.000 V here, 0.800 on Mariko)
      {0x2D, 3100000, 50000, false},  // LDO5 GC card
      {0x2F, 2800000, 50000, false},  // LDO6 touch + ALS
      {0x31, 1050000, 50000, true },  // LDO7 XUSB - up on Erista, down on Mariko
      {0x33, 1050000, 50000, false},  // LDO8 XUSB/DP
  };
  const LdoDef *ldos = mariko ? ldos_mariko : ldos_erista;
  for (int i = 0; i < 9; i++) {
    const LdoDef &l = ldos[i];
    uint8_t code = (uint8_t)(((l.uv - 800000u) / l.step) & 0x3F);
    max77620_regs[l.volt_reg] = (uint8_t)(code | (3u << 6)); // POWER_MODE_NORMAL
    // CFG2 POK is what max77620_regulator_get_status() reads for an LDO.
    max77620_regs[l.volt_reg + 1] = l.on ? (BIT(3) | BIT(2)) : 0x00;
  }
  // SD CFG1 registers (0x1D..0x20): flag the rails as power-OK too.
  for (uint8_t r = 0x1D; r <= 0x20; r++)
    max77620_regs[r] = BIT(1); // MPOK

  // PMIC GPIO config (0x36..0x3D) and the FPS masters (0x43..0x45), measured
  // on a real Mariko. These read back as 0 otherwise, which makes every pin
  // look like a low open-drain output and every FPS slot like 40 us.
  {
    // GPIO1 idles high on Erista and low on Mariko; everything else matches.
    static const uint8_t gpio_mariko[8] = {0x06, 0x00, 0x00, 0x00,
                                           0x01, 0x02, 0x02, 0x02};
    static const uint8_t gpio_erista[8] = {0x06, 0x06, 0x00, 0x00,
                                           0x01, 0x02, 0x02, 0x02};
    const uint8_t *gpio_cfg = mariko ? gpio_mariko : gpio_erista;
    for (int i = 0; i < 8; i++)
      max77620_regs[0x36 + i] = gpio_cfg[i];
    // FPS master config. TIME_PERIOD is 5 (1280 us) on Mariko and 7 on Erista.
    max77620_regs[0x43] = mariko ? 0x28 : 0x38;   // FPS0
    max77620_regs[0x44] = mariko ? 0x2A : 0x3A;   // FPS1, EN_SRC 1
    max77620_regs[0x45] = mariko ? 0x28 : 0x38;   // FPS2
    // ONOFFCNFG2 wake-source mask: POWER | ACOK | MBATT | ALARM1/2.
    max77620_regs[0x42] = 0x1F;

    // AME_GPIO (0x40) picks the alternate function per PMIC GPIO; a clear bit
    // is a plain GPIO. Measured 0x1E on Mariko and 0x1C on Erista -- the only
    // value in the whole Bluetooth section that varies by SoC generation.
    // Bit 4 is the 32K_OUT1 mux the vendor device tree asks for on GPIO4, and
    // bit 3 hands GPIO3 (the vdd_3v3 gate) to the Flexible Power Sequencer.
    // Unseeded this read 0x00, so a payload concluded the mux was unset and
    // programmed it itself.
    max77620_regs[0x40] = mariko ? 0x1E : 0x1C;
  }

  // CNFG1_32K (0x03): the LPO that clocks the combo radio. 32K_OK (bit 7) and
  // 32K_OUT0_EN (bit 2) are set on every healthy console; the remaining bits
  // differ per unit rather than per SoC generation (0xFC and 0xDC are both
  // measured on Erista and on Mariko), so the byte is a config knob.
  max77620_regs[0x03] = state ? state->pmic_cnfg1_32k.load() : 0xFC;
}

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

// ==================== I2C_2: Rohm BH1730 ambient light sensor ============
//
// Slave 0x29 on GEN2_I2C. bdk's als.c powers LDO6, brings up I2C_2 and reads
// the part/revision byte at register 0x12; the visible and IR ADC results come
// from the DATA0/DATA1 register pairs (little-endian, low byte first).
//
// Every BH1730 register access carries the command magic (0x80) in the address
// byte - BH1730_ADDR(reg) = 0x80 | reg - so the low 7 bits select the register.
//
// Without this bus modelled the ALS read back 0x00, and a payload correctly
// concluded "no chip / I2C fault" on a console that has a perfectly good
// sensor. Defaults are what a real Mariko reported in a lit room.
#define BH1730_I2C_ADDR   0x29
#define BH1730_ID_REG     0x12
#define BH1730_DATA0LOW   0x14   // visible
#define BH1730_DATA1LOW   0x16   // IR

static uint8_t  i2c2_slave = 0;
static uint8_t  i2c2_reg   = 0;
static uint32_t i2c2_cnfg  = 0;

uint32_t i2c2_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - I2C2_BASE);
  switch (offset) {
  case 0x00: return i2c2_cnfg;   // CNFG reads back (bdk RMWs it to set GO)
  case 0x1C: return 0;           // STATUS: transfer complete, no NACK
  case 0x8C: return 0;           // CONFIG_LOAD complete
  case 0x68: return (1 << 11);   // INT_STATUS: BUS_CLEAR_DONE
  case I2C_CMD_DATA1: {
    if (i2c2_slave != BH1730_I2C_ADDR)
      return 0;
    uint16_t vis = state->als_visible.load();
    uint16_t ir  = state->als_ir.load();
    switch (i2c2_reg & 0x7F) {   // strip the command magic
    case BH1730_ID_REG:       return state->als_part_id.load();
    case BH1730_DATA0LOW:     return vis & 0xFF;
    case BH1730_DATA0LOW + 1: return (vis >> 8) & 0xFF;
    case BH1730_DATA1LOW:     return ir & 0xFF;
    case BH1730_DATA1LOW + 1: return (ir >> 8) & 0xFF;
    default:                  return 0;
    }
  }
  default: return 0;
  }
}

void i2c2_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  switch ((uint32_t)(addr - I2C2_BASE)) {
  case 0x00:            i2c2_cnfg  = val; break;
  case I2C_CMD_ADDR0:   i2c2_slave = (val >> 1) & 0x7F; break;
  case I2C_CMD_DATA1:   i2c2_reg   = val & 0xFF; break;
  default: break;
  }
}

// Forward decl: the "normal" (CMD_DATA1) register model, reused below so both
// I2C transfer styles see identical device state.
static uint32_t i2c_device_reg_read(EmuState *state, bool on_i2c5, uint8_t slave,
                                    uint8_t reg);

static void packet_populate_rx(EmuState *state, bool on_i2c5, PacketState &p) {
    p.rx_pos  = 0;
    p.rx_size = std::min((uint32_t)sizeof(p.rx_buf), p.payload_size);
    memset(p.rx_buf, 0, sizeof(p.rx_buf));
    if (!on_i2c5 && p.dev_addr == 0x18) {
        bm92t36_fill_rx(state, p.reg_addr, p.rx_buf, p.rx_size);
        return;
    }
    // Every other slave: serve packet-mode reads from the same register models
    // the normal path uses. BDK's i2c_recv_buf_small() goes through
    // i2c_xfer_packet, so a payload that reads the PMIC/charger/gauge that way
    // used to get all-zeros here while the identical i2c_recv_byte() read
    // returned real data.
    for (uint32_t i = 0; i < p.rx_size; i++)
        p.rx_buf[i] = (uint8_t)i2c_device_reg_read(state, on_i2c5, p.dev_addr,
                                                   (uint8_t)(p.reg_addr + i));
}

uint32_t i2c_read(EmuState *state, uint64_t addr) {
  bool on_i2c5 = (addr >= I2C5_BASE);
  uint32_t base = on_i2c5 ? I2C5_BASE : I2C1_BASE;
  uint32_t offset = (uint32_t)(addr - base);

  PacketState &pkt = on_i2c5 ? pkt_i2c5 : pkt_i2c1;

  switch (offset) {
  case 0x00:          // I2C_CNFG — read back what was written (see note at decl)
    return i2c_cnfg_reg[on_i2c5 ? 1 : 0];
  case 0x1C:          // I2C_STATUS
    // Complete, not busy - and NOACK unless something actually lives at the
    // addressed slave. Answering ACK for every address made a bus census
    // report all 112 addresses as populated and, worse, made chips that are
    // NOT fitted on this board look present: hwtest concluded the TC94B15WBG
    // headphone amp answered at 0x1A and then decoded its zeroed registers as
    // real readings. An emulator that says yes to everything cannot be used
    // to test a probe whose whole job is deciding what is there.
    return i2c_slave_present(state, on_i2c5, i2c_slave_addr)
               ? 0u
               : I2C_STATUS_NOACK;
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
    // ALC5639 / RT5639 audio codec (slave 0x1C on I2C_1).
    //
    // Two things make this different from every other chip on this bus.
    //
    // BYTE ORDER. The codec's registers are 16 bit, MSB first on the wire
    // ("Read WORD Protocol"), and a payload reassembles them as
    // (buf[0] << 8) | buf[1]. bdk's _i2c_recv_normal fills buf with a plain
    // memcpy from CMD_DATA1, i.e. little-endian, so the value returned here
    // reaches the payload byte-swapped. Everything below is therefore stored
    // the natural way round and swapped once on the way out - getting this
    // backwards makes the vendor ID read 0xEC10 and the part look absent.
    //
    // POWER GATING. The codec's own LDO1 is off until PZ4 is driven high, and
    // nothing in RCM does that, so a real console answers here only after the
    // payload enables it. Gating on PZ4 keeps that behaviour observable
    // instead of handing out an identity the hardware would not have given.
    if (!on_i2c5 && i2c_slave_addr == 0x1C) {
      uint32_t pz4_out = mmio_regs.count(GPIO_BASE + 0x624)
                             ? mmio_regs[GPIO_BASE + 0x624] : 0;
      if (!(pz4_out & (1u << 4)))
        return 0;              // LDO1 still off: the part is silent
      auto be16 = [](uint16_t v) -> uint16_t {
        return (uint16_t)((v << 8) | (v >> 8));
      };
      switch (i2c_reg_addr) {
      case 0xFE: return be16(0x10EC);  // vendor ID, Realtek
      case 0xFF: return be16(0x6231);  // device ID, the rt5640 driver's probe
      case 0x00: return be16(0x0002);  // device id field, read-only here
      default:
        // Everything else reads back what the init table wrote. The payload
        // verifies several of its own writes (0xFA MCLK_DET, 0x73 ADDA_CLK,
        // the power and mixer registers), and a codec that always read 0
        // would report every one of them as not having stuck.
        return be16((uint16_t)codec_reg_get(i2c_reg_addr));
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
      max77620_regs_init(state);
      switch (i2c_reg_addr) {
      case 0x15: { // ONOFFSTAT
        // bit 2 EN0  - power button
        // bit 1 ACOK - charger present. Measured 0x02 on a real Mariko sitting
        //              on a charger; reporting 0 here made the PMIC disagree
        //              with the BQ24193 and tripped payload VBUS<->ACOK
        //              cross-checks that are meant to catch a broken ACOK trace.
        uint32_t v = state->btn_power.load() ? (1u << 2) : 0;
        if (state->chg_vbus_stat.load() != 0 || state->chg_power_good.load())
          v |= (1u << 1);
        return v;
      }
      // CID3 is a whole byte (0x5B on a real console); payloads print its low
      // nibble as "max77620 v%d". Masking here would turn 0x5B into 0x0B.
      case 0x5B: return state->pmic_silicon_rev.load();
      case 0x5C: return state->pmic_otp.load();                // CID4: 0x35 Erista, 0x53 Mariko
      case 0x5D: return state->pmic_es_rev.load(); // CID5: ES version (0x81)
      // Everything else comes out of the modelled register file, which holds
      // the values max77620_config_default() programs plus anything the
      // payload has written since. Returning 0 here (the old behaviour) made
      // every rail decode as 0.600 V and made writes look like they had no
      // effect ("not settable").
      default:   return max77620_regs[i2c_reg_addr];
      }
    }
    // MAX77621 CPU/GPU regulator (slave 0x1B/0x1C on I2C_5, Erista only).
    // Hekate reads CHIPID1 (reg 0x04) and prints the byte verbatim.
    if (on_i2c5 && (i2c_slave_addr == 0x1B || i2c_slave_addr == 0x1C)) {
      // Erista part: don't answer on a Mariko-configured console, otherwise a
      // payload sees both the MAX77621 and the MAX77812 present at once.
      if (state->pmic_otp.load() == 0x53) return 0;
      if (i2c_reg_addr == 0x04) return state->cpu_pmic_version.load();
      // VOUT (0x00) / VOUT_DVS (0x01): bit 7 = enable, bits 6:0 = 606.25 mV +
      // N * 6.25 mV. Report the rails enabled at a sane 1.0 V idle point.
      if (i2c_reg_addr == 0x00 || i2c_reg_addr == 0x01)
        return 0x80 | (uint8_t)((1000000u - 606250u) / 6250u);
      return 0;
    }
    // MAX77812 multi-phase buck (Mariko / Lite / OLED), I2C_5 @ 0x33 for the
    // PHASE211 retail variant (0x31 is the PHASE31 dev-kit part, left NAK'd).
    // Replaces the dual MAX77621 on Erista. Rails: M1 = GPU, M3 = DRAM,
    // M4 = CPU; vout_mv = 250 + N * 5.
    if (on_i2c5 && i2c_slave_addr == 0x33) {
      if (state->pmic_otp.load() != 0x53) return 0; // Mariko-family only
      auto vout = [](uint32_t mv) -> uint8_t {
        return (uint8_t)(((mv - 250u) / 5u) & 0xFF);
      };
      switch (i2c_reg_addr) {
      case 0x14: return 0x05; // VERSION: QS silicon (ES2 = 0x04)
      case 0x05: return 0x00; // TOPSYS_STAT: no thermal / OV / UV fault
      case 0x22: return 0x00; // BUCK_STAT: no per-rail fault latch
      // EN_CTRL: in RCM the CPU/GPU rails are still dormant (HOS brings them
      // up), so report all phases disabled - that is the true cold state.
      case 0x06: return 0x00;
      // Measured on a real Mariko while dormant in RCM.
      case 0x23: return vout(650);  // M1 GPU
      case 0x25: return vout(600);  // M3 DRAM (VDD2)
      case 0x26: return vout(600);  // M4 CPU
      default:   return 0;
      }
    }
    // MAX77620 RTC (slave 0x68 on I2C_5). Separate slave address from the PMIC
    // core. Registers 0x07..0x0D hold SEC/MIN/HOUR/WEEKDAY/MONTH/YEAR/DAY.
    //
    // Values are BCD unless CONTROL.BCD_MODE is clear, and the WEEKDAY
    // register is a *bitmask* (bit N set = day N), not an ordinal - returning
    // 0 there is what produced the "weekday 8" / "2000-00-00" nonsense.
    // YEAR counts from 2000. We serve a fixed, deterministic wall time and
    // advance the seconds from the emulated clock so repeated reads move
    // forward (an RTC that never ticks is itself a fault a payload may flag).
    if (on_i2c5 && i2c_slave_addr == 0x68) {
      if (i2c_reg_addr == 0x03) return 0x00; // CONTROL: binary mode, 24h
      if (i2c_reg_addr == 0x04 || i2c_reg_addr == 0x05) return 0x00; // UPDATE0/1: idle
      if (i2c_reg_addr >= 0x07 && i2c_reg_addr <= 0x0D) {
        // 2026-01-15 12:34:00 + emulated uptime, in binary (BCD_MODE off).
        uint32_t secs = 0u + (uint32_t)(state->emu_usec / 1000000ULL);
        uint32_t sec = (0 + secs) % 60;
        uint32_t min = (34 + ((0 + secs) / 60)) % 60;
        uint32_t hour = (12 + ((34 * 60 + secs) / 3600)) % 24;
        switch (i2c_reg_addr) {
        case 0x07: return sec;
        case 0x08: return min;
        case 0x09: return hour;        // bit 6 would be PM in 12h mode
        case 0x0A: return 1u << 4;     // WEEKDAY bitmask: Thursday
        case 0x0B: return 1;           // MONTH  (1-12)
        case 0x0C: return 26;          // YEAR   (offset from 2000)
        case 0x0D: return 15;          // DAY    (1-31)
        }
      }
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

// Read one slave register through the same model the normal (CMD_DATA1) path
// uses. The device models key off the i2c_slave_addr / i2c_reg_addr globals,
// so we borrow them for the call and put them back - that keeps a single
// source of truth for device behaviour instead of a second copy for
// packet-mode transfers.
static uint32_t i2c_device_reg_read(EmuState *state, bool on_i2c5, uint8_t slave,
                                    uint8_t reg) {
  uint8_t saved_slave = i2c_slave_addr;
  uint8_t saved_reg = i2c_reg_addr;
  i2c_slave_addr = slave;
  i2c_reg_addr = reg;
  uint32_t v = i2c_read(state, (on_i2c5 ? I2C5_BASE : I2C1_BASE) + 0x0C);
  i2c_slave_addr = saved_slave;
  i2c_reg_addr = saved_reg;
  return v;
}

void i2c_write(EmuState *state, uint64_t addr, uint32_t val) {
  bool on_i2c5 = (addr >= I2C5_BASE);
  uint32_t base = on_i2c5 ? I2C5_BASE : I2C1_BASE;
  uint32_t offset = (uint32_t)(addr - base);

  PacketState &pkt = on_i2c5 ? pkt_i2c5 : pkt_i2c1;

  switch (offset) {
  case 0x00: { // I2C_CNFG — this is where a normal-mode transfer actually runs.
    // Layout (bdk soc/i2c.c): bits 3:1 = size-1, bit 6 = 0 write / 1 read,
    // bit 9 = NORMAL_MODE_GO. A register READ is set up by first *writing*
    // one byte (the register address), so only a transfer of 2+ bytes in
    // write direction is a genuine register write. Committing on CMD_DATA1
    // instead would clobber every register with 0 on each read setup.
    i2c_cnfg_reg[on_i2c5 ? 1 : 0] = val;
    bool is_write = (val & (1u << 6)) == 0;
    uint32_t size = ((val >> 1) & 7) + 1;
    if ((val & (1u << 9)) && is_write && size >= 2 && on_i2c5 &&
        i2c_slave_addr == MAX77620_I2C_ADDR) {
      max77620_regs_init(state);
      max77620_regs[i2c_cmd_data1 & 0xFF] = (uint8_t)((i2c_cmd_data1 >> 8) & 0xFF);
    }
    // ALC5639 codec: a 16-bit register write arrives as three bytes packed
    // into CMD_DATA1 - [reg, value MSB, value LSB] - because the part is
    // big-endian on the wire while bdk packs the buffer little-endian.
    if ((val & (1u << 9)) && is_write && size >= 3 && !on_i2c5 &&
        i2c_slave_addr == 0x1C) {
      uint8_t  reg = (uint8_t)(i2c_cmd_data1 & 0xFF);
      uint16_t v   = (uint16_t)((((i2c_cmd_data1 >> 8) & 0xFF) << 8) |
                                ((i2c_cmd_data1 >> 16) & 0xFF));
      if (reg == 0x00) {
        // MX-00h is the software reset: a WRITE clears the whole register
        // file (a READ just returns the device id). The payload opens its
        // init sequence with it, so honouring it keeps the model's state
        // machine in step with the part's.
        codec_regs.clear();
      } else {
        codec_regs[reg] = v;
      }
    }
    break;
  }
  case I2C_CMD_ADDR0:
    i2c_slave_addr = (val >> 1) & 0x7F;
    break;
  case I2C_CMD_DATA1:
    i2c_reg_addr = val & 0xFF;
    i2c_cmd_data1 = val;
    // Hekate's i2c_send_byte goes through _i2c_send_normal which packs
    // [reg, value, ...] into CMD_DATA1 (low byte = reg, next byte = value).
    // Catch ONOFFCNFG1 writes to MAX77620 here so the emulator reacts to
    // the payload's POWER_OFF / SFT_RST same as real hardware.
    if (on_i2c5 && i2c_slave_addr == MAX77620_I2C_ADDR && i2c_reg_addr == 0x41) {
      uint8_t v = (val >> 8) & 0xFF;
      if (v & 0x02) {                         // ONOFFCNFG1_PWR_OFF
        printf("[emu] MAX77620 PWR_OFF received - exiting\n");
        fflush(stdout);
        state->running = false;
      } else if (v & 0x80) {                  // ONOFFCNFG1_SFT_RST
        printf("[emu] MAX77620 SFT_RST received - rebooting payload\n");
        fflush(stdout);
        state->reboot_requested = true;
      }
    }
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
    } else if (!pkt.is_read && idx == 4) {
      // Second payload byte = value being written. Watch for PMIC commands
      // that mean "shut the SoC down" or "reset" so the emulator can react
      // the same way real hardware would.
      if (on_i2c5 && pkt.dev_addr == 0x3C && pkt.reg_addr == 0x41) {
        uint8_t v = val & 0xFF;
        if (v & 0x02) {                         // ONOFFCNFG1_PWR_OFF
          printf("[emu] MAX77620 PWR_OFF received - exiting\n");
          fflush(stdout);
          state->running = false;
        } else if (v & 0x80) {                  // ONOFFCNFG1_SFT_RST
          printf("[emu] MAX77620 SFT_RST received - rebooting payload\n");
          fflush(stdout);
          state->reboot_requested = true;
        }
      }
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

// Forward decls — these handlers live further down this file but are reached
// from misc_read's catch-all routing for DSI accesses.
uint32_t dsi_read(EmuState *state, uint64_t addr);
void     dsi_write(EmuState *state, uint64_t addr, uint32_t val);

// KFUSE_KEYADDR (0x7000FC88): word index into the 144-word HDCP key block,
// bit 16 = auto-increment on each KFUSE_KEYS read. File scope so both the read
// and the write path share one cursor.
static uint32_t g_kfuse_keyaddr = 0;

uint32_t misc_read(EmuState *state, uint64_t addr) {
  // PINMUX (APB_MISC pad config range) — every write lands in the global
  // mmio_regs map at line 1726, so we just hand it back. Returning 0 here
  // (the old behavior) silently dropped the muxed function bits, which
  // broke any probe that read back PINMUX_AUX_* to confirm a pin's mode.
  if (addr >= PINMUX_BASE && addr < PINMUX_BASE + PINMUX_SIZE) {
    return mmio_regs.count(addr) ? mmio_regs[addr] : pinmux_reset_default(addr);
  }
  // APB_MISC_GP_HIDREV - hardware revision.
  // Bits 11:8 = chip ID (0x21 for both T210 / T210B01),
  // Bits  7:4 = major rev (1 = Erista T210, 2 = Mariko T210B01),
  // Bits  3:0 = minor.
  // Hekate's hw_get_chip_id() does `(HIDREV >> 4) & 0xF` and compares against
  // GP_HIDREV_MAJOR_T210B01 (=2) to decide h_cfg.t210b01.
  // Measured on a real Mariko: 0x00012127 (chip 0x21, major 2, minor 1).
  // The old 0x20 / 0x10 carried only the major nibble, so the chip-ID byte
  // read back as 0x00 and payloads printed "Chip ID : 0x00".
  // Both measured on real consoles: Mariko 0x00012127 (major 2, minor 1),
  // Erista 0x00022117 (major 1, minor 2).
  if (addr == APB_MISC_BASE + 0x804)
    return state->is_mariko.load() ? 0x00012127u : 0x00022117u;

  // UART
  if (addr >= 0x70006000 && addr < 0x70006500) {
    uint32_t offset = 0;
    int port = uart_port_of(addr, &offset);
    if (port < 0)  // the gaps between the five register blocks
      return mmio_regs.count(addr) ? mmio_regs[addr] : 0;
    UartPort &up = uart_ports[port];
    // Anything on UART-D can observe the radio, so settle its state machine
    // (POR completion, RTS_N) before answering.
    if (port == UART_D)
      bt_uart_sync(state);

    if (offset == 0x14) {
      // LSR: THRE | TMTY (0x60) always ready -- bdk's uart_wait_xfer spins on
      // TMTY with no timeout, so a modelled TX-busy state would deadlock the
      // payload -- plus RDR (bit 0) when this port's receive FIFO has bytes,
      // and any break/framing error the line itself is generating.
      uint32_t lsr = 0x60;
      if (!state->uart_rx_fifo[port].empty())
        lsr |= 0x01;
      lsr |= bt_line_lsr_bits(state, port);
      return lsr;
    }
    if (offset == 0x18) {
      // MSR. Bit 6 (RI) reads high on a Tegra UART whether or not anything is
      // wired to that pin: it is the 0x40 floor all four reference consoles
      // report. Bit 4 is the peer's RTS_N arriving on our CTS input. Bits 3:0
      // are "changed since you last looked" -- sticky, and cleared BY this
      // read, which is why the payload sees 0x4F once right after its
      // loopback self-test and 0x40 on every quiet read afterwards.
      uint32_t msr = 0x40 | (up.cts ? 0x10 : 0) | up.msr_delta;
      up.msr_delta = 0;
      return msr;
    }
    // With DLAB set, 0x00 and 0x04 are the divisor latches, NOT RBR/IER.
    // Honouring that matters twice over: a payload reading back its own baud
    // divisor gets the real value, and - more importantly - reading the
    // divisor no longer POPS a byte off the receive FIFO. That silently ate
    // host bytes, which is corruption waiting to happen on a sideload.
    uint32_t base = uart_bases[port];
    bool dlab = (mmio_regs.count(base + 0x0C) ? mmio_regs[base + 0x0C] : 0) & 0x80;

    if (offset == 0x00 && dlab)
      return up.divisor & 0xFF;         // DLL
    if (offset == 0x04 && dlab)
      return (up.divisor >> 8) & 0xFF;  // DLM

    if (offset == 0x00) {
      // RBR: pop the next queued byte; empty FIFO returns 0 (the payload
      // should have checked LSR.RDR first).
      if (!state->uart_rx_fifo[port].empty()) {
        uint8_t b = state->uart_rx_fifo[port].front();
        state->uart_rx_fifo[port].pop_front();
        return b;
      }
      return 0;
    }
    // Every other register (LCR, IER, IIR, MCR, IRDA_CSR, ASR...) reads back
    // what the payload wrote. bdk's uart_init programs LCR and the divisor
    // latches, and a payload that reports its own UART config - baud divisor,
    // word length, DLAB state - got zeros before this, so it decoded as
    // "0 baud, word=5".
    return mmio_regs.count(addr) ? mmio_regs[addr] : 0;
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
    // MC_EMEM_CFG (0x50) holds the external memory size in MB. Payloads that
    // report "RAM size" read it here; leaving it 0 makes a healthy console
    // look like it has no DRAM at all. Switch ships 4 GB across all models
    // (Erista/Mariko/Lite/OLED).
    if (offset == 0x50)
      return 4096;
    return 0;
  }
  // PWM controller (LCD backlight on PWM0, optional fan on PWM1). Same
  // story as PINMUX above — the writes are captured by the global cache
  // already; return whatever was last written so the payload can read
  // back its own duty cycle / enable bit.
  if (addr >= PWM_BASE && addr < PWM_BASE + 0x1000) {
    return mmio_regs.count(addr) ? mmio_regs[addr] : 0;
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
  // timeout (bdk/soc/kfuse.c), so DONE|CRCPASS has to come back.
  //
  // KFUSE_KEYS (0x8C) streams the 144-word HDCP key block, with KFUSE_KEYADDR
  // (0x88) holding the word index and bit 16 requesting auto-increment. A
  // diagnostic payload reads all 144 words and reports how many are blank, so
  // returning zeros looks like a wholly unprogrammed (i.e. faulty) key block.
  // Emit deterministic non-zero pseudo-data instead - real per-unit HDCP keys
  // aren't something we can or should reproduce, but "programmed" is the
  // honest state for a healthy emulated console.
  if (addr >= 0x7000FC00 && addr < 0x7000FD00) {
    uint32_t offset = (uint32_t)(addr - 0x7000FC00);
    if (offset == 0x80) // KFUSE_STATE
      // DONE | CRCPASS | CURBLOCK=48, as measured on real silicon (0x00030030).
      return (1u << 16) | (1u << 17) | 48u;
    if (offset == 0x88) // KFUSE_KEYADDR
      return g_kfuse_keyaddr;
    if (offset == 0x8C) { // KFUSE_KEYS
      uint32_t idx = g_kfuse_keyaddr & 0xFF;
      if (g_kfuse_keyaddr & (1u << 16))
        g_kfuse_keyaddr = (g_kfuse_keyaddr & ~0xFFU) | ((idx + 1) & 0xFF);
      // Cheap deterministic mix - stable across runs, never zero.
      uint32_t w = idx * 0x9E3779B9u + 0xA5A5A5A5u;
      w ^= w >> 15;
      w *= 0x85EBCA6Bu;
      w ^= w >> 13;
      return w ? w : 0xDEADBEEFu;
    }
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
    else if (offset == 0x28) {
      // HOSTCTL. sdmmc_get_bus_width() reads this register directly to decide
      // 1 / 4 / 8-bit (SDHCI_CTRL_4BITBUS = BIT(1), SDHCI_CTRL_8BITBUS =
      // BIT(5)). It had no read handler at all, so every payload saw a 1-bit
      // bus even after a successful 4-bit SD / 8-bit eMMC negotiation - and an
      // eMMC HS400-vs-bus-width cross-check would call that a fault.
      result = (base == SDMMC4_BASE) ? state->sdmmc4_hostctl
                                     : state->sdmmc_hostctl;
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
  // KFUSE_KEYADDR - sets the read cursor into the HDCP key block.
  if (addr == 0x7000FC88) {
    g_kfuse_keyaddr = val;
    return;
  }
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
      // 0x3E, not 0x1E: the old mask dropped SDHCI_CTRL_8BITBUS (BIT(5)), so
      // an eMMC 8-bit bus could never be represented.
      hostctl = val & 0x3E;
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
          // Capability/health bytes a diagnostic payload reports on. Values
          // measured from the eMMC in a real Mariko; left at 0 these read as
          // "feature not supported", which looks like a reduced-firmware
          // replacement part.
          ext_csd[503] = 0x01; // HPI_FEATURES: supported, CMD13 variant
          ext_csd[231] = 0x55; // SEC_FEATURE_SUPPORT: secure erase/trim/sanitize
          ext_csd[232] = 0x02; // TRIM_MULT
          ext_csd[229] = 0x11; // SEC_TRIM_MULT (measured)
          ext_csd[162] = 0x01; // RST_N_FUNCTION: permanently enabled
          ext_csd[267] = 0x01; // PRE_EOL_INFO: normal
          ext_csd[268] = 0x01; // DEVICE_LIFE_TIME_EST_TYP_A: 0-10%
          ext_csd[269] = 0x01; // DEVICE_LIFE_TIME_EST_TYP_B: 0-10%

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
          // SD SCR: byte 0 low nibble = SD_SPEC (2 => v2.00), byte 1 low
          // nibble = SD_BUS_WIDTHS (5 => 1-bit + 4-bit). This is what makes
          // hekate switch the bus to 4-bit; without it the init falls back to
          // 1-bit HS25 and hwtest flags "1-bit (dirty slot?)".
          uint8_t scr[8] = {0x02, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
          // Tegra drives this small read over SDMA with the destination in
          // register 0x58 (captured here as adma_addr), NOT the SDHCI-standard
          // sysad (0x00), which Tegra leaves unused. Matches the EXT_CSD path.
          uint64_t dma_addr = adma_addr ? adma_addr : sysad;
          if (dma_addr)
            uc_mem_write(uc, dma_addr, scr, 8);
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
              // No rawnand image loaded: serve a synthesized valid GPT so the
              // [eMMC GPT] probe sees "EFI PART" with good CRCs instead of
              // "signature missing". Everything outside LBA 0..2 reads as zero.
              if (state->emmc_gpp_fds.empty()) {
                if (is_read) {
                  size_t glen = 0;
                  const uint8_t *gpt = emmc_synth_gpt(&glen);
                  std::vector<uint8_t> io_buf(len, 0);
                  for (size_t o = 0; o < len; o++) {
                    uint64_t abs = current_off + o;
                    if (abs < glen) io_buf[o] = gpt[abs];
                  }
                  uc_mem_write(uc, dma_addr, io_buf.data(), len);
                }
                return;
              }
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

// ---- power partitions and I/O-pad deep power down --------------------------
//
// PWRGATE_STATUS is a live bitmap of ungated partitions, toggled one at a
// time through PWRGATE_TOGGLE. Unmodelled it read 0 forever, so bdk's
// pmc_domain_pwrgate_set() burned its full 5000-iteration retry budget and
// then reported failure for any partition a payload tried to bring up.
//
// IO_DPD_REQ/STATUS park pad groups in deep power down. Both reset to 0 (no
// pad parked), and writes carry a 2-bit command in bits 31:30: 1 = DPD OFF
// (wake the named pads), 2 = DPD ON (park them).
static uint32_t pmc_pwrgate_status = 0;
static uint32_t pmc_io_dpd_status[2] = {0, 0};

uint32_t pmc_read(EmuState *state, uint64_t addr) {
  (void)state;
  uint32_t offset = (uint32_t)(addr - PMC_BASE);

  switch (offset) {
  case 0x38:
    return pmc_pwrgate_status; // APBDEV_PMC_PWRGATE_STATUS
  case 0x30:
    return 0; // APBDEV_PMC_PWRGATE_TOGGLE - START always already clear
  case 0x1BC:
    return pmc_io_dpd_status[0];  // APBDEV_PMC_IO_DPD_STATUS
  case 0x1C4:
    return pmc_io_dpd_status[1];  // APBDEV_PMC_IO_DPD2_STATUS
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
  case 0x00:
    // APBDEV_PMC_CNTRL — bit 4 = MAIN_RST. Hekate writes this from
    // power_set_state(REBOOT_RCM); we mirror it as a soft reboot of the
    // emulated payload so the user can iterate without restarting rcm_emu.
    if (val & (1u << 4)) {
      printf("[emu] PMC MAIN_RST written - rebooting payload\n");
      fflush(stdout);
      state->reboot_requested = true;
    }
    break;
  case 0x30: {
    // APBDEV_PMC_PWRGATE_TOGGLE: bits 4:0 select a partition, bit 8 starts
    // the toggle. The transition is instantaneous here, so START reads back
    // clear and the caller's poll on PWRGATE_STATUS succeeds immediately.
    if (val & (1u << 8)) {
      uint32_t part = val & 0x1F;
      pmc_pwrgate_status ^= (1u << part);
      if (part == 3) // POWER_RAIL_PCIE
        pcie_set_powergate((pmc_pwrgate_status & (1u << 3)) != 0);
    }
    break;
  }
  case 0x1B8:
  case 0x1C0: {
    int bank = (offset == 0x1C0) ? 1 : 0;
    uint32_t code = val >> 30, mask = val & 0x3FFFFFFF;
    if (code == 1)
      pmc_io_dpd_status[bank] &= ~mask;   // DPD OFF: wake these pads
    else if (code == 2)
      pmc_io_dpd_status[bank] |= mask;    // DPD ON: park them
    break;
  }
  case 0x50:
    pmc_scratch0 = val;
    break;
  case 0x120:
    pmc_scratch37 = val;
    break;
  }
}

// ==================== Flow Controller ====================

// FLOW_CTLR_HALT_COP_EVENTS bit layout (bdk soc/t210.h):
//   bits 31:29  HALT_MODE  (2 = WAITEVENT)
//   bit  28     HALT_JTAG
//   bit  25     HALT_USEC  - timed wake-up sources; the low 8 bits carry
//   bit  24     HALT_MSEC    the delay count (HALT_MAX_CNT = 0xFF)
//   bit  23     HALT_SEC
static constexpr uint32_t HALT_SEC = 1u << 23;
static constexpr uint32_t HALT_MSEC = 1u << 24;
static constexpr uint32_t HALT_USEC = 1u << 25;
static constexpr uint32_t HALT_TIMED = HALT_SEC | HALT_MSEC | HALT_USEC;

// FLOW_CTLR_RAM_REPAIR (0x40): bit 0 = REQ, bit 1 = STS.
// ccplex_boot_cpu0() requests RAM repair for the fast cluster and then spins
// on STS with no timeout (bdk soc/ccplex.c). Reads used to fall through to 0,
// so anything that brings up the CCPLEX - e.g. a payload running its memory
// test on the A57s - hung there forever. Latch REQ and report the repair as
// complete straight away.
static constexpr uint32_t RAM_REPAIR_REQ = 1u << 0;
static constexpr uint32_t RAM_REPAIR_STS = 1u << 1;
static uint32_t flow_ram_repair = 0;

static uint32_t flow_read(EmuState *state, uint64_t addr) {
  (void)state;
  uint32_t offset = (uint32_t)(addr - 0x60007000);
  if (offset == 0x40)
    return flow_ram_repair | (flow_ram_repair & RAM_REPAIR_REQ ? RAM_REPAIR_STS : 0);
  return mmio_regs.count(addr) ? mmio_regs[addr] : 0;
}

static void flow_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - 0x60007000);
  if (offset == 0x40) {
    flow_ram_repair = val;
    return;
  }
  if (offset == 0x04) {
    // Two very different things get written here and they must not be
    // conflated:
    //
    //  1. bpmp_usleep() / bpmp_msleep() park the BPMP with a TIMER event
    //     source (HALT_USEC / HALT_MSEC / HALT_SEC) plus a delay count, to
    //     sleep with the core clock-gated. The timer event wakes the core
    //     and execution continues. This is a routine sleep and happens all
    //     over the BDK - treating it as terminal killed any payload that
    //     slept this way (e.g. a 200 ms bpmp_msleep = 0x410000C8).
    //
    //  2. bpmp_halt() writes WAITEVENT | JTAG with no timer source and is
    //     followed by `while(true);`. That one really is "payload done"
    //     (power-off / reboot / fatal), so we exit cleanly.
    if (val & HALT_TIMED) {
      // Advance the emulated microsecond counter by the requested delay so
      // TIMERUS-based delta loops observe the time actually passing, then
      // let the CPU run on.
      uint32_t delay = val & 0xFF;
      uint64_t us = (val & HALT_USEC)   ? (uint64_t)delay
                    : (val & HALT_MSEC) ? (uint64_t)delay * 1000ULL
                                        : (uint64_t)delay * 1000000ULL;
      state->emu_usec += us;
      state->bpmp_slept_us += us; // feeds the ACTMON BPMP-load model
      return;
    }
    printf("[flow] BPMP HALT/WaitEvent (val=0x%08X), shutting down emulator\n",
           val);
    state->running = false;
    uc_emu_stop(state->uc);
  } else {
    printf("[flow] W: 0x%02X = 0x%08X\n", offset, val);
  }
}

// ==================== Clock/Reset ====================

// ---- RST_DEVICES_U / CLK_OUT_ENB_U -----------------------------------------
//
// bdk never writes these two directly: clock_enable() goes through the SET and
// CLR aliases (RST_DEV_U_SET 0x310 / _CLR 0x314, CLK_ENB_U_SET 0x330 / _CLR
// 0x334). With clk_rst_write a no-op and both reads hardcoded, enabling a _U
// peripheral had no observable effect at all -- clock_enable_uart(UART_D) ran
// to completion and the payload's own "is this port clocked?" gate then read
// CLK_U_UARTD clear and aborted. Both seeds are the values measured on a real
// console at that point in the sweep.
//
// Note the previous CLK_OUT_ENB_U value force-set bit 15 with a comment about
// keeping SDMMC4 alive; bit 15 of the _U register is DTV (deprecated). SDMMC4
// is CLK_L bit 15, which the hardcoded _L value 0x9802D1B0 already carries, so
// dropping the forced bit costs the storage model nothing and lets _U match
// hardware exactly.
static uint32_t car_rst_u = 0x828EC5F8;
static uint32_t car_enb_u = 0x01F00200;

uint32_t clk_rst_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - CLK_RST_BASE);
  // PLL_BASE registers (per Hekate bdk/soc/clock.h): each PLL has an _BASE
  // register where bit 30 = ENABLE and bit 27 = LOCK. After enabling a PLL
  // the boot code polls bit 27 until set. Real silicon locks within ~1ms; we
  // simulate "always locked + always enabled" so the polls return immediately.
  // Offsets: PLLC=0x80, PLLM=0x90, PLLP=0xA0, PLLA=0xB0, PLLU=0xC0, PLLD=0xD0,
  //          PLLX=0xE0, PLLE=0xE8, PLLD2=0x4B8, PLLREFE=0x4C4.
  bool mariko = state && state->pmic_otp.load() == 0x53;

  // On a real console in RCM most PLLs are DOWN - measured on a Mariko, only
  // PLLP (the BPMP source) and PLLD are enabled and locked; PLLC/M/A/U/X/D2/
  // DP/RE all read disabled. Reporting everything as locked made a payload's
  // PLL page pure fiction.
  //
  // The lock bit still has to follow an ENABLE the payload writes itself,
  // otherwise a clock_enable_pll* poll loop would spin forever - so a written
  // ENABLE always comes back with LOCK set.
  //   PLL       Mariko            Erista
  //   PLLC 0x80  down              down
  //   PLLM 0x90  down              ENABLED, not locked (DRAM runs off PLLP)
  //   PLLP 0xA0  enabled + locked  enabled + locked
  //   PLLA 0xB0  down              down
  //   PLLU 0xC0  down              enabled + locked
  //   PLLD 0xD0  enabled + locked  enabled + locked
  //   PLLX/D2/DP/RE               down on both
  switch (offset) {
  case 0x80: case 0xB0: case 0xE0: case 0xE8:
  case 0x4B8: case 0x4C4: {
    uint32_t w = mmio_regs.count(addr) ? mmio_regs[addr] : 0;
    if (w & (1u << 30))
      return w | (1u << 27);          // payload brought it up -> locked
    return w;                          // measured: down on both
  }
  case 0x90: { // PLLM
    uint32_t w = mmio_regs.count(addr) ? mmio_regs[addr] : 0;
    if (w & (1u << 30))
      return w | (1u << 27);
    // Erista leaves PLLM enabled but unlocked at this point; Mariko has it off.
    return mariko ? 0u : (1u << 30);
  }
  case 0xC0: { // PLLU
    uint32_t w = mmio_regs.count(addr) ? mmio_regs[addr] : 0;
    if (w & (1u << 30))
      return w | (1u << 27);
    return mariko ? 0u : ((1u << 30) | (1u << 27));
  }
  case 0xA0: // PLLP_BASE - same value on both
    return 0x48115408u;
  case 0xD0: // PLLD_BASE - up on both
    return (1u << 30) | (1u << 27);
  }

  // Informational clock registers, measured on a real Mariko. These read back
  // as 0 otherwise, which makes the whole clock page look dead.
  {
    switch (offset) {
    case 0xA4:  return 0x00000003; // PLLP_OUTA
    case 0x68:  return 0x00005C00; // PLLP_OUTB
    // SCLK_BURST differs slightly by generation (measured).
    case 0x28:  return mariko ? 0x20003333u : 0x20003330u;
    case 0x2C:  return 0x80000000; // SUPER_SCLK_DIVIDER
    case 0x30:  return 0x00000002; // CLK_SYSTEM_RATE
    case 0x14:  return 0x030180C1; // CLK_OUT_ENB_H
    case 0x280: return 0x23024780; // CLK_OUT_ENB_X
    // _L carries the SDMMC clock-enable bits the storage model depends on:
    // SDMMC1 is bit 14 and SDMMC4 is bit 15, and the measured value has both.
    case 0x10:  return 0x9802D1B0;
    // _U and its reset counterpart are live, so a payload that enables a _U
    // peripheral (UART-D, I2C3, SDMMC3, ...) can see that it worked.
    case 0x0C:  return car_rst_u;  // RST_DEVICES_U
    case 0x18:  return car_enb_u;  // CLK_OUT_ENB_U
    // CLK_SOURCE_UARTD. clock_uart_use_src_div() programs PLLP_OUT0 with
    // CLK_SRC_DIV(2) here (0x00000002) and sets UART_SRC_CLK_DIV_EN for the
    // 1M/3M rates; the payload reads it back to confirm the port's source.
    case 0x1C0: return mmio_regs.count(addr) ? mmio_regs[addr] : 0;
    }
  }

  // ---- PTO (Peripheral Test Output) clock counter ----
  //
  // bdk's clock_get_dev_freq() measures a clock by selecting it with
  // PTO_CLK_CNT_CNTL (0x60), counting its edges over a 16-tick 32768 Hz
  // window, then reading PTO_CLK_CNT_STATUS (0x64): bit 31 = BUSY, bits 23:0
  // = the count, and freq_khz = cnt * 32768 / 16 / 1000 (i.e. cnt * 2.048).
  //
  // Unmodelled, STATUS read 0 and every rate on a payload's clock page came
  // out "(idle / not clocked)". Return counts that decode to the rates a real
  // Mariko reports in RCM; anything not measured stays 0, which is the honest
  // "not running" answer (CCLK_G really is idle - the A57s are off - and
  // PLLP_OBS is not routed).
  if (offset == 0x64) {
    uint32_t cntl = mmio_regs.count(CLK_RST_BASE + 0x60)
                        ? mmio_regs[CLK_RST_BASE + 0x60] : 0;
    uint32_t src = (cntl >> 14) & 0x1FF;   // PTO_SRC_SEL
    uint32_t khz = 0;
    switch (src) {
    case 0x1C: khz = mariko ? 407971 : 407980; break; // SCLK / BPMP
    case 0x24: khz = mariko ? 203991 : 204001; break; // EMC (DRAM)
    case 0x23: khz = mariko ? 199673 : 199677; break; // SDMMC4 (eMMC)
    case 0x20: khz = mariko ? 199671 : 199694; break; // SDMMC1 (SD)
    case 0x12: khz = 0; break;   // CCLK_G - A57 cluster is off in RCM
    case 0x43: khz = 0; break;   // PLLP_OBS - not routed
    default:   khz = 0; break;
    }
    // Invert bdk's maths; BUSY is always clear because we answer instantly.
    return (uint32_t)(((uint64_t)khz * 1000ull * 16ull) / 32768ull) & 0xFFFFFF;
  }

  // ---- OSC_FREQ_DET: crystal frequency measurement ----
  //
  // clock_get_osc_freq() does NOT read OSC_CTRL - it triggers this counter
  // (0x58), waits for BUSY in the status register (0x5C) to clear, and looks
  // the count up in a table. bdk asks for a 2-period 32768 Hz window, so a
  // 38.4 MHz crystal gives 38400000 * 2 / 32768 = 2343 counts, inside the
  // table's 2268..2418 bucket for 38400 kHz. Unmodelled this returned 0 and
  // every payload printed "OSC : 0.000 MHz".
  if (offset == 0x5C)
    return 2343;  // -> 38.4 MHz, BUSY clear (we answer instantly)

  if (offset == 0x50)
    return (4 << 28); // OSC_CTRL, informational
  if (offset == 0x04)
    return 0; // RST_DEVICES_L (none in reset)

  // PLLE and PLLREFE report lock in their _MISC registers, not in _BASE bit
  // 27 like the PLLs handled above, so the PCIe model answers for them.
  {
    uint32_t v = mmio_regs.count(addr) ? mmio_regs[addr] : 0;
    if (pcie_car_read(offset, &v))
      return v;
  }
  (void)state;
  return 0;
}

void clk_rst_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  uint32_t offset = (uint32_t)(addr - CLK_RST_BASE);
  // Only the _U pair is live so far; everything else stays a no-op, and the
  // generic mmio_regs cache in the write hook still serves the CLK_SOURCE_*
  // read-backs. bdk reaches these through the SET/CLR aliases exclusively.
  switch (offset) {
  case 0x00C: car_rst_u  =  val; break; // RST_DEVICES_U, direct write
  case 0x018: car_enb_u  =  val; break; // CLK_OUT_ENB_U, direct write
  case 0x310: car_rst_u |=  val; break; // RST_DEV_U_SET
  case 0x314: car_rst_u &= ~val; break; // RST_DEV_U_CLR
  case 0x330: car_enb_u |=  val; break; // CLK_ENB_U_SET
  case 0x334: car_enb_u &= ~val; break; // CLK_ENB_U_CLR
  default: break;
  }
  // The PCIe root complex depends on PLLE, PLLREFE and the PCIE/AFI/
  // PCIEXCLK/UPHY/padctl reset+enable bits, so it shadows the same writes.
  pcie_car_write(offset, val);
}

// ==================== Fuse ====================

// Latched value from the last FUSE_ADDR write. Hekate's fuse_read(addr) walks
// the raw fuse macroarray via FUSE_ADDR + FUSE_CTRL=READ + FUSE_RDATA, used by
// the "Dump fuses" Nyx button to write fuse_array_raw_*.bin to SD.
static uint32_t g_fuse_ctrl_addr = 0;

uint32_t fuse_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - FUSE_BASE);
  switch (offset) {
  case 0x00:
    // FUSE_CTRL: bits 20:16 = status (4 = IDLE), bit 30 = sense data ready.
    // Returning IDLE unconditionally lets fuse_wait_idle() exit; otherwise
    // Dump Fuses spins forever after issuing a READ command.
    return (1u << 30) | (4u << 16);
  case 0x08:
    // FUSE_RDATA: would return the raw macroarray word at FUSE_ADDR. We
    // don't model the raw array (only the cached fuse register file at
    // offset 0x100+), so return zeros here. The dump file ends up zero-
    // filled but the operation completes successfully.
    (void)g_fuse_ctrl_addr;
    return 0;
  }
  if (offset >= EmuState::FUSE_WORDS * 4) return 0;
  return state->fuse_at(offset).load();
}

void fuse_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  uint32_t offset = (uint32_t)(addr - FUSE_BASE);
  if (offset == 0x04) g_fuse_ctrl_addr = val; // FUSE_ADDR
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

// ==================== APE audio hub: I2S, ADMAIF, ADMA ====================
//
// hwtest's speaker probe drives a real tone through this path, so all three
// blocks needed a model. They were previously stubs returning 0, which made
// the probe print nonsense - I2S_TIMING reading back 0 turns
// "64 bclk/frame, fs 23437 Hz" into "2 bclk/frame, fs 750000 Hz" - and the
// ADMAIF page was not dispatched at all, so a quarter of a million PIO
// sample writes each fell through to the write-chain's logging arm.
//
// Write-back is the right default here: every register hwtest reads back is
// one it wrote itself (I2S_CTRL, I2S_TIMING, the CIF and FIFO_CTRL words),
// and reporting what was programmed is exactly what the real block does.

static uint32_t i2s_read(EmuState *state, uint64_t addr) {
  (void)state;
  // Read back what the payload programmed. mmio_regs is populated by the
  // write hook before dispatch, so this needs no state of its own.
  return mmio_regs.count(addr) ? mmio_regs[addr] : 0;
}

static void i2s_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  (void)addr;
  (void)val;
  // The write hook already cached the value in mmio_regs; nothing else to do.
}

// ADMAIF + AXBAR. Two offsets carry behaviour, the rest is write-back:
//
//   +0x32C  TX channel 0 FIFO_WRITE - the PIO sample port. Accept and drop.
//           This MUST be silent: it is written ~224k times per run.
//   +0x744  TX ACIF FIFO_FULL - the flag the sample loop spins on. Bit 0
//           must stay clear or hwtest burns 200,001 reads per sample before
//           giving up, i.e. ~45 billion reads for one melody.
//
// Soft-reset registers read back 0 because the payload polls them until they
// self-clear (bdk's usual "write 1, wait for 0" handshake).
static uint32_t admaif_read(EmuState *state, uint64_t addr) {
  (void)state;
  uint32_t offset = (uint32_t)(addr - ADMAIF_BASE);
  switch (offset) {
  case 0x704:            // global soft reset - completed
  case 0x744:            // TX ACIF FIFO_FULL - never full, so PIO never spins
    return 0;
  default:
    return mmio_regs.count(addr) ? mmio_regs[addr] : 0;
  }
}

static void admaif_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  (void)val;
  uint32_t offset = (uint32_t)(addr - ADMAIF_BASE);
  if (offset == 0x32C) {
    // Sample sink. Deliberately does nothing and logs nothing.
    return;
  }
  // Everything else is left in mmio_regs by the write hook.
}

// ADMA. hwtest resets the block and sets GLOBAL_CMD, then feeds the FIFO by
// PIO and never starts a channel, so only the soft-reset read-back matters.
static uint32_t adma_read(EmuState *state, uint64_t addr) {
  (void)state;
  uint32_t offset = (uint32_t)(addr - ADMA_BASE);
  if (offset == 0xC04)   // global soft reset - completed
    return 0;
  return mmio_regs.count(addr) ? mmio_regs[addr] : 0;
}

static void adma_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  (void)addr;
  (void)val;
}

// ==================== ACTMON (Activity Monitor) ====================
//
// ACTMON_BASE = 0x6000C800, i.e. inside the SYSREG page, so it is dispatched
// from sysreg_read/sysreg_write below.
//
// Layout (bdk soc/actmon.c):
//   0x00 GLB_STATUS, 0x04 GLB_PERIOD_CTRL
//   device blocks at 0x80 + dev*0x40, fields:
//     +0x00 ctrl   +0x04 upper_wmark  +0x08 lower_wmark  +0x0C init_avg
//     +0x10 avg_upper +0x14 avg_lower +0x18 count_weight +0x1C count
//     +0x20 avg_count +0x24 intr_status +0x28 ctrl2
//
// bdk derives load as count * 100 / (ACTMON_FREQ / (PERIOD_MS * WEIGHT)),
// so a fully-busy sample period reads
//   19200000/1000 * 20 * 5 = 1,920,000  ->  100.0 %
//
// We model the count from real emulated behaviour rather than a constant: the
// BPMP is "active" whenever it is executing and "idle" while parked in a
// FLOW_CTLR timed halt (bpmp_usleep / bpmp_msleep, tracked in bpmp_slept_us).
// A payload that measures load while spinning therefore sees a high number,
// and one that measures while sleeping sees a low one - which is exactly what
// an ACTMON test is checking for.
static constexpr uint32_t ACTMON_OFF_IN_SYSREG = 0x800;
static constexpr uint32_t ACTMON_FULL_COUNT = 1920000; // 100.0 % for one period
static constexpr int ACTMON_NDEV = 7;

struct ActmonDev {
  uint32_t ctrl = 0;
  uint32_t init_avg = 0;
  uint32_t count_weight = 0;
  uint32_t regs[16] = {0};   // catch-all for the rest of the block
  uint64_t last_us = 0;      // window start for the next count sample
  uint64_t last_slept = 0;
  uint32_t count = 0;
  uint32_t avg_count = 0;
  bool seeded = false;
};
static ActmonDev actmon_dev[ACTMON_NDEV];
static uint32_t actmon_glb_period = 0;

// Recompute count/avg_count for `d` from the activity in the window that has
// elapsed since the last sample. Windows shorter than 200 us reuse the last
// value so back-to-back reads stay coherent.
static void actmon_sample(EmuState *state, ActmonDev &d) {
  uint64_t now = state->emu_usec;
  uint64_t slept = state->bpmp_slept_us;
  if (!d.seeded) {
    d.last_us = now;
    d.last_slept = slept;
    d.seeded = true;
    return;
  }
  uint64_t window = now - d.last_us;
  if (window < 200)
    return;
  uint64_t win_slept = slept - d.last_slept;
  if (win_slept > window)
    win_slept = window;
  uint64_t active = window - win_slept;

  uint32_t c = (uint32_t)((active * ACTMON_FULL_COUNT) / window);
  d.count = c;
  // avg_count trails count with a simple IIR (bdk asks for a 128-sample
  // average via K_VAL; an exponential decay is a fair stand-in).
  d.avg_count = d.avg_count ? (uint32_t)((d.avg_count * 3 + c) / 4) : c;
  d.last_us = now;
  d.last_slept = slept;
}

static uint32_t actmon_read(EmuState *state, uint32_t off) {
  if (off == 0x00) { // GLB_STATUS - report the monitors that are enabled
    uint32_t st = 0;
    static const uint32_t act_bit[ACTMON_NDEV] = {
        1u << 15, 1u << 14, 1u << 13, 1u << 12, 1u << 10, 1u << 9, 1u << 8};
    for (int i = 0; i < ACTMON_NDEV; i++)
      if (actmon_dev[i].ctrl & (1u << 31))
        st |= act_bit[i];
    return st;
  }
  if (off == 0x04)
    return actmon_glb_period;

  if (off >= 0x80 && off < 0x80 + ACTMON_NDEV * 0x40) {
    int dev = (int)((off - 0x80) / 0x40);
    uint32_t f = (off - 0x80) % 0x40;
    ActmonDev &d = actmon_dev[dev];
    switch (f) {
    case 0x00: return d.ctrl;
    case 0x0C: return d.init_avg;
    case 0x18: return d.count_weight;
    case 0x1C:
      if (d.ctrl & (1u << 31)) actmon_sample(state, d);
      return d.count;
    case 0x20:
      if (d.ctrl & (1u << 31)) actmon_sample(state, d);
      return d.avg_count;
    default: return d.regs[(f / 4) & 15];
    }
  }
  return 0;
}

static void actmon_write(EmuState *state, uint32_t off, uint32_t val) {
  if (off == 0x04) {
    actmon_glb_period = val;
    return;
  }
  if (off >= 0x80 && off < 0x80 + ACTMON_NDEV * 0x40) {
    int dev = (int)((off - 0x80) / 0x40);
    uint32_t f = (off - 0x80) % 0x40;
    ActmonDev &d = actmon_dev[dev];
    switch (f) {
    case 0x00:
      d.ctrl = val;
      if (val & (1u << 31)) {
        // Enabling the monitor starts a fresh sample window.
        d.seeded = false;
        d.count = 0;
        d.avg_count = 0;
        actmon_sample(state, d);
      }
      break;
    case 0x0C: d.init_avg = val; break;
    case 0x18: d.count_weight = val; break;
    default: d.regs[(f / 4) & 15] = val; break;
    }
    return;
  }
}

static uint32_t sysreg_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - SYSREG_BASE);
  if (offset >= ACTMON_OFF_IN_SYSREG)
    return actmon_read(state, offset - ACTMON_OFF_IN_SYSREG);
  return 0;
}

static void sysreg_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - SYSREG_BASE);
  if (offset >= ACTMON_OFF_IN_SYSREG) {
    actmon_write(state, offset - ACTMON_OFF_IN_SYSREG, val);
    return;
  }
  printf("[sysreg] W: 0x%08X = 0x%08X\n", offset, val);
}

// ==================== SOC_THERM (on-die thermal sensors) ====================
//
// The Tegra's internal thermal sensors, as opposed to the TMP451 board sensor
// on I2C. Register layout follows the T210 TRM / Linux tegra-soctherm:
//
//   per-sensor block, 0x20 apart:
//     +0x00 CONFIG0, +0x04 CONFIG1, +0x08 CONFIG2
//     +0x0C STATUS0
//     +0x10 STATUS1  -> bit 31 TEMP_VALID, bits 15:0 current temp
//   sensor bases: CPU0 0xC0, CPU1 0xE0, CPU2 0x100, CPU3 0x120,
//                 MEM0 0x140, MEM1 0x160, GPU 0x180, PLLX 0x1A0
//
//   aggregated readback:
//     0x1C8 SENSOR_TEMP1 : CPU  bits 31:16 | GPU  bits 15:0
//     0x1CC SENSOR_TEMP2 : MEM  bits 31:16 | PLLX bits 15:0
//
// Temperature encoding (both places): bits 15:8 = integer °C, bit 7 = +0.5 °C,
// bit 0 = negate. We drive every sensor from the same emulated SoC die
// temperature the TMP451 model uses, so the existing "SoC temp" slider in the
// hardware-config window moves these too. Real silicon shows a few degrees of
// spread between sensors; we add a small fixed per-sensor offset so a payload
// that cross-checks them doesn't see suspiciously identical values.
static uint16_t soctherm_encode_temp(int temp_c10) {
  bool neg = temp_c10 < 0;
  int a = neg ? -temp_c10 : temp_c10;
  uint16_t v = (uint16_t)((a / 10) << 8);
  if ((a % 10) >= 5)
    v |= (1u << 7); // +0.5 °C
  if (neg)
    v |= 1u;
  return v;
}

// Per-sensor offset in °C*10, indexed by sensor slot (CPU0..3, MEM0/1, GPU,
// PLLX). Small and deterministic - just enough to look like real silicon.
static const int kSocThermOffsets[8] = {0, 3, -2, 5, -5, -4, 8, 2};

static int soctherm_sensor_c10(EmuState *state, int idx) {
  return (int)state->soc_temp_c10.load() + kSocThermOffsets[idx & 7];
}

static uint32_t soc_therm_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - SOC_THERM_BASE);

  // Per-sensor STATUS1 (current reading + valid bit).
  if (offset >= 0xC0 && offset < 0x1C0 && ((offset - 0xC0) % 0x20) == 0x10) {
    int idx = (int)((offset - 0xC0) / 0x20);
    return (1u << 31) | soctherm_encode_temp(soctherm_sensor_c10(state, idx));
  }

  switch (offset) {
  case 0x1C8: // SENSOR_TEMP1: CPU | GPU
    return ((uint32_t)soctherm_encode_temp(soctherm_sensor_c10(state, 0)) << 16) |
           soctherm_encode_temp(soctherm_sensor_c10(state, 6));
  case 0x1CC: // SENSOR_TEMP2: MEM | PLLX
    return ((uint32_t)soctherm_encode_temp(soctherm_sensor_c10(state, 4)) << 16) |
           soctherm_encode_temp(soctherm_sensor_c10(state, 7));
  default:
    // CONFIG/STATUS0 and the THERMCTL/THROT blocks read back whatever was
    // written; anything never written reads 0.
    if (mmio_regs.count(addr))
      return mmio_regs[addr];
    return 0;
  }
}

static void soc_therm_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  // Accept and remember configuration writes (sensor enables, THERMTRIP /
  // THROT programming, CTMON setup) so read-back-after-write behaves.
  mmio_regs[addr] = val;
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
    } else if ((address >= PCIE_BLOCK_BASE &&
                address < PCIE_BLOCK_BASE + PCIE_BLOCK_SIZE) ||
               (address >= PCIE_CS_BASE &&
                address < PCIE_CS_BASE + PCIE_CS_MODEL_SIZE) ||
               (address >= PCIE_MEM_BASE &&
                address < PCIE_MEM_BASE + PCIE_MEM_MODEL_SIZE)) {
      result = pcie_read(state, address);
    } else if (address >= XUSB_PADCTL_BASE &&
               address < XUSB_PADCTL_BASE + XUSB_PADCTL_SIZE) {
      result = padctl_read(state, address);
    } else if (address >= MSELECT_BASE &&
               address < MSELECT_BASE + MSELECT_SIZE) {
      result = mselect_read(state, address);
    } else if (address >= VIC_BASE && address < VIC_BASE + VIC_SIZE) {
      result = vic_read(state, address);
    } else if (address >= SYSREG_BASE && address < SYSREG_BASE + SYSREG_SIZE) {
      result = sysreg_read(state, address);
    } else if (address >= BPMP_CACHE_BASE &&
               address < BPMP_CACHE_BASE + BPMP_CACHE_SIZE) {
      result = bpmp_cache_read(state, address);
    } else if (address >= SOC_THERM_BASE &&
               address < SOC_THERM_BASE + SOC_THERM_SIZE) {
      result = soc_therm_read(state, address);
    } else if (address >= 0x60007000 && address < 0x60007000 + 0x1000) {
      result = flow_read(state, address);
    } else if (address >= I2S_BASE && address < I2S_BASE + I2S_SIZE) {
      result = i2s_read(state, address);
    } else if (address >= ADMAIF_BASE && address < ADMAIF_BASE + ADMAIF_SIZE) {
      result = admaif_read(state, address);
    } else if (address >= ADMA_BASE && address < ADMA_BASE + ADMA_SIZE) {
      result = adma_read(state, address);
    } else if (address >= RTC_BASE && address < RTC_BASE + RTC_SIZE) {
      result = rtc_read(state, address);
    } else if (address >= I2C2_BASE && address < I2C2_BASE + I2C2_SIZE) {
      // I²C2 / BH1730 ambient light sensor (slave 0x29).
      result = i2c2_read(state, address);
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
    } else if ((address >= PCIE_BLOCK_BASE &&
                address < PCIE_BLOCK_BASE + PCIE_BLOCK_SIZE) ||
               (address >= PCIE_CS_BASE &&
                address < PCIE_CS_BASE + PCIE_CS_MODEL_SIZE) ||
               (address >= PCIE_MEM_BASE &&
                address < PCIE_MEM_BASE + PCIE_MEM_MODEL_SIZE)) {
      pcie_write(state, address, val);
    } else if (address >= XUSB_PADCTL_BASE &&
               address < XUSB_PADCTL_BASE + XUSB_PADCTL_SIZE) {
      padctl_write(state, address, val);
    } else if (address >= MSELECT_BASE &&
               address < MSELECT_BASE + MSELECT_SIZE) {
      mselect_write(state, address, val);
    } else if (address >= SDMMC1_BASE && address < SDMMC1_BASE + 0x200) {
      misc_write(uc, state, address, value, size);
    } else if (address >= SDMMC4_BASE && address < SDMMC4_BASE + 0x200) {
      misc_write(uc, state, address, value, size);
    } else if (address >= 0x70006000 && address < 0x70006500) {
      uint32_t offset = 0;
      int port = uart_port_of(address, &offset);
      // Note offset 0 is the divisor latch LSB - not the transmit register -
      // whenever DLAB is set, so writing the baud divisor must not be
      // mistaken for a character to print. Every register (including this
      // one) was already cached above; the divisor is additionally kept in
      // UartPort because a later THR write overwrites that cache entry.
      if (port >= 0) {
        UartPort &up = uart_ports[port];
        uint32_t ubase = uart_bases[port];
        bool dlab = (mmio_regs.count(ubase + 0x0C) ? mmio_regs[ubase + 0x0C] : 0) & 0x80;

        if (offset == 0x00 && dlab) {
          up.divisor = (uint16_t)((up.divisor & 0xFF00) | (val & 0xFF));
        } else if (offset == 0x04 && dlab) {
          up.divisor = (uint16_t)((up.divisor & 0x00FF) | ((val & 0xFF) << 8));
        } else if (offset == 0x08) {
          // FCR. RX_CLR is honoured only on the BT port, where the receive
          // FIFO is fed by an emulated device and dropping it is exactly what
          // the hardware does. On the console ports that FIFO models a human
          // at a terminal, and bdk's uart_init() issues an unconditional
          // RX_CLR - which would silently eat scripted keystrokes.
          if (port == UART_D && (val & 0x02))
            state->uart_rx_fifo[port].clear();
        } else if (offset == 0x10) {
          uint8_t old_mcr = up.mcr;
          up.mcr = (uint8_t)val;
          // Entering or leaving 16550 internal loopback swings all four modem
          // inputs at once - MCR[3:0] are tied onto MSR[7:4] while it is on -
          // so every delta bit latches. That is what makes the payload's
          // first MSR read after its loopback self-test 0x4F, not 0x40.
          if ((old_mcr ^ up.mcr) & 0x10)
            up.msr_delta |= 0x0F;
        }

        if (offset == 0 && !dlab) {
          uint8_t b = (uint8_t)val;

          if (up.mcr & 0x10) {
            // Internal loopback: the byte is tied straight back to this
            // port's own receiver inside the controller and never reaches a
            // pad. It must NOT be handed to whatever is on the far end, or
            // the BT chip's H4 parser desyncs on the A5 5A 00 FF test
            // pattern - and it must not reach the TX log either, since it is
            // not something the payload actually transmitted.
            state->uart_rx_fifo[port].push_back(b);
          } else {
            // UART-D is the Bluetooth radio's H4 transport.
            if (port == UART_D)
              bt_chip_rx(state, b);

            // Append every byte to the per-port TX log — the console window
            // renders this as scrolling text. Trim from the front if it grows
            // past 64 KB so ImGui rendering stays responsive.
            std::string &log = state->uart_tx_log[port];
            if (b == '\n' || (b >= 0x20 && b < 0x7F))
              log.push_back((char)b);
            if (log.size() > 64 * 1024)
              log.erase(0, log.size() - 48 * 1024);

            // Mirror to host stdout in line-buffered form so existing
            // `grep '[uart]'` workflows still work.
            static char uart_line[EmuState::N_UARTS][512];
            static size_t uart_len[EmuState::N_UARTS] = {0};
            auto flush_uart_line = [&](int p) {
              if (uart_len[p] > 0) {
                uart_line[p][uart_len[p]] = 0;
                printf("[uart%c] %s\n", 'A' + (char)p, uart_line[p]);
                fflush(stdout);
                uart_len[p] = 0;
              }
            };
            if (b == '\n' || b == '\r') {
              flush_uart_line(port);
            } else if (b >= 0x20 && b < 0x7F) {
              if (uart_len[port] + 1 >= sizeof(uart_line[port])) flush_uart_line(port);
              uart_line[port][uart_len[port]++] = (char)b;
            }
          }
        }
      }
    } else if (address >= VIC_BASE && address < VIC_BASE + VIC_SIZE) {
      vic_write(state, address, val);
    } else if (address >= SYSREG_BASE && address < SYSREG_BASE + SYSREG_SIZE) {
      sysreg_write(state, address, val);
    } else if (address >= BPMP_CACHE_BASE &&
               address < BPMP_CACHE_BASE + BPMP_CACHE_SIZE) {
      bpmp_cache_write(state, address, val);
    } else if (address >= SOC_THERM_BASE &&
               address < SOC_THERM_BASE + SOC_THERM_SIZE) {
      soc_therm_write(state, address, val);
    } else if (address >= I2S_BASE && address < I2S_BASE + I2S_SIZE) {
      i2s_write(state, address, val);
    } else if (address >= ADMAIF_BASE && address < ADMAIF_BASE + ADMAIF_SIZE) {
      admaif_write(state, address, val);
    } else if (address >= ADMA_BASE && address < ADMA_BASE + ADMA_SIZE) {
      adma_write(state, address, val);
    } else if (address >= RTC_BASE && address < RTC_BASE + RTC_SIZE) {
      rtc_write(state, address, val);
    } else if (address >= I2C2_BASE && address < I2C2_BASE + I2C2_SIZE) {
      i2c2_write(state, address, val);
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
            (address >= RTC_BASE && address < RTC_BASE + RTC_SIZE))) {
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
      {I2C2_BASE, I2C2_SIZE},
      {SOC_THERM_BASE, SOC_THERM_SIZE},
      {MIPI_CAL_BASE, MIPI_CAL_SIZE},
      {SOR_BASE, SOR_SIZE},
      {SDMMC1_BASE, SDMMC_SIZE},
      {SDMMC4_BASE, SDMMC_SIZE},
      {APB_MISC_BASE, APB_MISC_SIZE},
      {HOST1X_BASE, HOST1X_SIZE},
      {VIC_BASE, VIC_SIZE},
      {SYSREG_BASE, SYSREG_SIZE},
      {I2S_BASE, I2S_SIZE},
      {ADMAIF_BASE, ADMAIF_SIZE},
      {ADMA_BASE, ADMA_SIZE},
      {SE_BASE, SE_SIZE},
      {TSEC_BASE, TSEC_SIZE},
      {SYSCTR0_BASE, SYSCTR0_SIZE},
      {BPMP_CACHE_BASE, BPMP_CACHE_SIZE}, // BPMP_CACHE
      {TZRAM_BASE, TZRAM_SIZE},           // TZRAM
      {XUSB_PADCTL_BASE, XUSB_PADCTL_SIZE},
      {MSELECT_BASE, MSELECT_SIZE},
      // PCIe. These are the only mapped regions below 0x50000000, so they
      // also need their own hooks (see below) - the main MMIO hook pair
      // covers 0x50000000..0x7FFFFFFF and would never see them.
      {PCIE_BLOCK_BASE, PCIE_BLOCK_SIZE},
      {PCIE_CS_BASE, PCIE_CS_MODEL_SIZE},
      {PCIE_MEM_BASE, PCIE_MEM_MODEL_SIZE},
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

  // T210 puts the PCIe root complex at the BOTTOM of the address map, below
  // every other peripheral, so it needs a second hook pair. The range is a
  // filter, not a mapping - only the three small regions mapped above ever
  // resolve inside it.
  static uc_hook h_pcie_read, h_pcie_write;
  uc_hook_add(uc, &h_pcie_read, UC_HOOK_MEM_READ, (void *)hook_mmio_read,
              state, PCIE_BLOCK_BASE, PCIE_MEM_BASE + PCIE_MEM_MODEL_SIZE - 1);
  uc_hook_add(uc, &h_pcie_write, UC_HOOK_MEM_WRITE, (void *)hook_mmio_write,
              state, PCIE_BLOCK_BASE, PCIE_MEM_BASE + PCIE_MEM_MODEL_SIZE - 1);

  pcie_reset(state);

  uc_hook_add(uc, &h_unmapped,
              UC_HOOK_MEM_READ_UNMAPPED | UC_HOOK_MEM_WRITE_UNMAPPED,
              (void *)hook_unmapped, state, 1, 0);

  uc_hook_add(uc, &h_code, UC_HOOK_CODE, (void *)hook_code, state, 1, 0);
}
