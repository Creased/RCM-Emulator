#include "mmio.h"
#include "block_linear.h"
#include "trace.h"
#include "../emu_state.h"
#include "../platform.h"
#include "i2c3.h"
#include "memory_map.h"
#include "bpmp.h"
#include "ccplex.h"
#include "pcie.h"
#include "regcache.h"
#include "se_engine.h"
#include "tegra_bl.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>
#include <map>
#include <memory>
#include <unistd.h>
#include <vector>
#include <SDL2/SDL.h>

// pread/pwrite are POSIX and MinGW does not have them, so the SD/eMMC image
// access below would not compile for Windows. Seek-then-read is equivalent
// here: the emulated storage is touched only from the CPU thread, so the
// atomicity real pread() buys against a shared file offset is not in play.
// Kept next to the includes rather than in a header because these four call
// sites are the only users in the tree.
#ifdef _WIN32
#include <io.h>
static inline ssize_t pread(int fd, void *buf, size_t n, long long off) {
  if (_lseeki64(fd, off, SEEK_SET) < 0)
    return -1;
  return _read(fd, buf, (unsigned int)n);
}
static inline ssize_t pwrite(int fd, const void *buf, size_t n, long long off) {
  if (_lseeki64(fd, off, SEEK_SET) < 0)
    return -1;
  return _write(fd, buf, (unsigned int)n);
}
#endif

#define BIT(n) (1U << (n))

static RegCache mmio_regs;

// A payload asked for a reset. Stop the BPMP where it is instead of letting
// it run to the end of its batch: bdk's power_set_state() follows the reset
// write with bpmp_halt(), which must not get the chance to end the run.
static void request_reboot(EmuState *state, bool power_cycle) {
  if (power_cycle)
    state->reboot_cold = true;
  state->reboot_requested = true;
  if (g_bus_master == BUS_BPMP)
    uc_emu_stop(state->uc);
}

BusMaster g_bus_master = BUS_BPMP;

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
// reading 0, the honest "not modelled" answer (most pads reset to non-zero).
struct PinmuxDefault { uint16_t off; uint32_t val; };
static const PinmuxDefault pinmux_defaults[] = {
    // PH5 / BT_HOST_WAKE: E_INPUT | PARKED | TRISTATE | PULL_DOWN. Measured
    // as 0x0074 at payload entry on all four reference consoles.
    {0x1C8, 0x00000074},
    // PEX_L1_RST_N / PEX_L1_CLKREQ_N: the root-port-1 reset and clock-request
    // pads. TRM reset values, and what hwtest's Wi-Fi probe reads on a real
    // console before it unparks them: PE1 function, PARKED (bit 5) set, so the
    // pad does not drive until software clears it.
    {0x044, 0x00000460},
    {0x048, 0x00000470},
    // Pads the wireless probes sample before (or without) configuring them.
    // TRM 9.15 reset values: E_INPUT | PARKED | TRISTATE | PULL_DOWN (0x74),
    // UART2_RX pulled up instead (0x78).
    {0x0F8, 0x00000078},   // UART2_RX
    {0x118, 0x00000074},   // UART4_RX (UART-D, the BT transport)
    {0x1B8, 0x00000074},   // WIFI_RST
    {0x1C0, 0x00000074},   // AP_WAKE_BT
    {0x1C4, 0x00000074},   // BT_RST
    {0x1CC, 0x00000074},   // AP_WAKE_NFC
    {0x250, 0x00000074},   // GPIO_PH6
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
  uint8_t  ier       = 0;      // IER, which shares +0x04 with DLM
  uint8_t  fcr       = 0;      // FCR, write-only; IIR 7:6 reports its bit 0
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
  // `now` can step back a little when CPU0 takes the bus (it trails the
  // BPMP by up to a slice), so compare rather than subtract.
  if (bt_chip.phase == BT_PHASE_POR &&
      state->emu_usec >= bt_chip.por_us + BT_POR_US)
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
  return mmio_regs.get(addr);
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
    TRACE("[gpio] R: Port X IN = 0x%02X\n", val);
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
    uint32_t out = mmio_regs.get(GPIO_BASE + 0x624);
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
    uint32_t csr = mmio_regs.get(PWM_BASE + PWM_CSR_1_OFF);
    bool ch_en   = (csr & (1u << 31)) != 0;
    bool abs_off = (csr & (1u << 24)) != 0;
    uint32_t inv_duty = (csr >> 16) & 0xFF;   // inverted: 236 = ~0%
    bool spinning = ch_en && !abs_off && inv_duty < 236;
    uint32_t v = (spinning && ((state->emu_usec / 2186ull) & 1ull)) ? (1u << 7) : 0;
    // PS3 is the gamecard slot's card detect: active low, pulled up. No
    // cartridge is emulated, so an undriven PS3 reads high ("slot empty") -
    // it read 0, "cartridge seated", before.
    uint32_t oe = mmio_regs.get(GPIO_BASE + 0x418);
    if (!(oe & (1u << 3)))
      v |= 1u << 3;
    // Pins the payload drives itself read back what it drives.
    v = (v & ~oe) | (mmio_regs.get(GPIO_BASE + 0x428) & oe & 0x7F);
    return v;
  }

  // Port H IN. Resolved per pin (see gpio_h_in) instead of mirroring OUT,
  // because this is the port the Bluetooth probe measures by RELEASING pads:
  // PH5/BT_HOST_WAKE and PH3/BT_DEV_WAKE are read while the Tegra is
  // deliberately not driving them, and mirroring OUT there answers 0 to every
  // question regardless of what is on the board.
  if (offset == 0x13C) {
    uint32_t v = gpio_h_in(state);
    TRACE("[gpio] R: Port H IN = 0x%02X\n", v);
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
      uint32_t v = mmio_regs.get(addr);
      TRACE("[gpio] R: offset 0x%X (CNF/OE/OUT cache) = 0x%08X\n", offset, v);
      return v;
    }
    if (bank_off < 0x40) {
      // IN: mirror of the matching OUT one slot earlier.
      uint64_t out_addr = addr - 0x10;
      uint32_t v = mmio_regs.get(out_addr);
      TRACE("[gpio] R: offset 0x%X (IN, mirror of OUT @ 0x%X) = 0x%08X\n",
             offset, (uint32_t)(offset - 0x10), v);
      return v;
    }
    // MSK_CNF 0x80 / MSK_OE 0x90 / MSK_OUT 0xA0: aliases that read back the
    // plain register they write through, 0x80 lower.
    if (bank_off >= 0x80 && bank_off < 0xB0) {
      uint32_t v = mmio_regs.get(addr - 0x80);
      TRACE("[gpio] R: offset 0x%X (masked alias of 0x%X) = 0x%08X\n", offset,
             (uint32_t)(offset - 0x80), v);
      return v;
    }
  }

  TRACE("[gpio] R: offset 0x%X = 0\n", offset);
  return 0;
}

// The SD card loses power with PE4 (SDMMC section below).
static void sd_card_gpio_update();

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
    uint32_t cur   = mmio_regs.get(plain);
    mmio_regs[plain] = (cur & ~mask) | (val & mask);
    TRACE("[gpio] W: offset 0x%X masked -> 0x%X = 0x%08X\n", offset,
           (uint32_t)(offset - 0x80), mmio_regs[plain]);
  } else {
    TRACE("[gpio] W: offset 0x%X = 0x%08X\n", offset, val);
  }

  // Bank 1 holds ports E..H: port H carries BT_REG_ON and port E the SD
  // card's supply (PE4), so re-sample both after anything that could have
  // moved them.
  if (offset >= 0x100 && offset < 0x200) {
    gpio_h_update(state);
    sd_card_gpio_update();
  }
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

// ---- CCPLEX CPU rail regulators on I2C_5 ----
//
// Erista: two MAX77621 bucks, CPU at 0x1B and GPU at 0x1C. Mariko: one
// MAX77812 at 0x33 (PHASE211) whose M4 phase is the CPU rail. These used to
// be fixed read-only answers, which was fine while nothing but an identity
// read ever reached them - but bdk's ccplex_boot_cpu0() turns the CPU rail on
// through them with read-modify-writes, and whether that rail is up decides
// whether CPU0 can run at all. So they are register files now, seeded with
// the values the fixed answers used to give.
//
// MAX77621 VOUT/VOUT_DVS: bit 7 = enable, bits 6:0 = 606.25 mV + N*6.25 mV.
static const uint8_t kMax77621Seed[2][8] = {
    {0x80 | 63, 0x80 | 63, 0, 0, 0, 0, 0, 0},  // CPU, 1.000 V
    {0x80 | 63, 0x80 | 63, 0, 0, 0, 0, 0, 0},  // GPU, 1.000 V
};
static uint8_t max77621_regs[2][8] = {
    {0x80 | 63, 0x80 | 63, 0, 0, 0, 0, 0, 0},
    {0x80 | 63, 0x80 | 63, 0, 0, 0, 0, 0, 0},
};
static uint8_t max77812_regs[256];
static bool max77812_ready = false;

static void max77812_regs_init() {
  if (max77812_ready)
    return;
  max77812_ready = true;
  auto vout = [](uint32_t mv) -> uint8_t {
    return (uint8_t)(((mv - 250u) / 5u) & 0xFF);
  };
  memset(max77812_regs, 0, sizeof(max77812_regs));
  max77812_regs[0x14] = 0x05;      // VERSION: QS silicon (ES2 = 0x04)
  max77812_regs[0x05] = 0x00;      // TOPSYS_STAT: no thermal / OV / UV fault
  max77812_regs[0x22] = 0x00;      // BUCK_STAT: no per-rail fault latch
  // EN_CTRL: in RCM the CPU/GPU rails are still dormant (HOS brings them up),
  // so all phases start disabled - that is the true cold state.
  max77812_regs[0x06] = 0x00;
  // Measured on a real Mariko while dormant in RCM.
  max77812_regs[0x23] = vout(650); // M1 GPU
  max77812_regs[0x25] = vout(600); // M3 DRAM (VDD2)
  max77812_regs[0x26] = vout(600); // M4 CPU
}

static void max77620_regs_init(EmuState *state);
static uint8_t max77620_regs[256];

bool cpu_rail_on(EmuState *state) {
  if (state->pmic_otp.load() == 0x53) {
    max77812_regs_init();
    return (max77812_regs[0x06] >> 6) & 1;        // EN_CTRL.EN_M4 (CPU)
  }
  // The MAX77621's EN pin is MAX77620 GPIO5 (bdk _ccplex_enable_power_t210
  // configures it push-pull, output high, before touching the buck), and the
  // buck's own output enable is VOUT bit 7. Both have to be on.
  max77620_regs_init(state);
  uint8_t gpio5 = max77620_regs[0x3B];
  bool en_pin = !(max77620_regs[0x40] & (1u << 5)) &&   // plain GPIO, not AME
                !(gpio5 & (1u << 1)) &&                  // DIR = output
                (gpio5 & (1u << 3));                     // output high
  return en_pin && (max77621_regs[0][0] & 0x80);
}

// Which slaves actually answer, per bus. This is the emulator's answer to
// "is that chip fitted?", so it must list exactly what is modelled below and
// nothing else - a bus scan is a real diagnostic and an emulator that ACKs
// every address turns it into noise.
//
// Deliberately absent, because they are absent on the board this models:
//   I2C_1 0x1A  TC94B15WBG headphone amp - not fitted on Erista; hwtest
//               reports "not fitted on this board" and that is correct.
// I2C_STATUS CMD1_STAT (3:0): 1 = SL1_NOACK_FOR_BYTE1, i.e. nobody answered
// the address byte (TRM 35.x). bdk tests the whole nibble.
#define I2C_STATUS_NOACK (1u << 0)

static bool i2c_slave_present(EmuState *state, bool on_i2c5, uint8_t addr) {
  if (on_i2c5) {
    bool mariko = state && state->pmic_otp.load() == 0x53;
    switch (addr) {
    case 0x1B:                    // MAX77621 CPU DC-DC  (Erista)
    case 0x1C:                    // MAX77621 GPU DC-DC  (Erista)
      return !mariko;
    case 0x33:                    // MAX77812 CPU/GPU/DRAM buck (Mariko family)
      return mariko;
    case MAX77620_I2C_ADDR:       // 0x3C PMIC
    case 0x68:                    // MAX77620 RTC sub-block
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
    return (mmio_regs.get(GPIO_BASE + 0x624)) & (1u << 4);
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
static bool max77620_regs_ready = false;
static uint8_t max77620_seed_otp = 0;

static void max77620_regs_init(EmuState *state) {
  // Seeded for one console generation; switching it in the config window
  // re-seeds on the next access instead of keeping the other one's rails.
  uint8_t otp = state ? state->pmic_otp.load() : 0;
  if (max77620_regs_ready && otp == max77620_seed_otp)
    return;
  max77620_regs_ready = true;
  max77620_seed_otp = otp;

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
    // Power mode in CFG 7:6: NORMAL for a rail that is up, DISABLE for one
    // that is down. CFG2 bit 3 (POK), which max77620_regulator_get_status()
    // reads, is derived from it on every read (max77620_read).
    max77620_regs[l.volt_reg] = (uint8_t)(code | ((l.on ? 3u : 0u) << 6));
    max77620_regs[l.volt_reg + 1] = BIT(2);
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
    int      hdr_idx       = 0;   // 0 PROT, 1 size, 2 header, 3 payload
    uint32_t tx_seen       = 0;   // payload bytes of this packet so far
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
      uint32_t pz4_out = mmio_regs.get(GPIO_BASE + 0x624);
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
      default:
        // LDOn_CFG2 (0x24, 0x26 .. 0x34): POK (bit 3) follows the rail, i.e.
        // the power mode in LDOn_CFG, so a rail the payload switches on
        // reports good and one it switches off stops doing so.
        if (i2c_reg_addr >= 0x24 && i2c_reg_addr <= 0x34 && !(i2c_reg_addr & 1)) {
          bool up = (max77620_regs[i2c_reg_addr - 1] >> 6) != 0;
          return (max77620_regs[i2c_reg_addr] & ~BIT(3)) | (up ? BIT(3) : 0);
        }
        return max77620_regs[i2c_reg_addr];
      }
    }
    // MAX77621 CPU/GPU regulator (slave 0x1B/0x1C on I2C_5, Erista only).
    // Hekate reads CHIPID1 (reg 0x04) and prints the byte verbatim.
    if (on_i2c5 && (i2c_slave_addr == 0x1B || i2c_slave_addr == 0x1C)) {
      // Erista part: don't answer on a Mariko-configured console, otherwise a
      // payload sees both the MAX77621 and the MAX77812 present at once.
      if (state->pmic_otp.load() == 0x53) return 0;
      if (i2c_reg_addr == 0x04) return state->cpu_pmic_version.load();
      // VOUT (0x00) / VOUT_DVS (0x01) / CONTROL1/2 read back what was last
      // written; see max77621_regs.
      if (i2c_reg_addr < 8)
        return max77621_regs[i2c_slave_addr - 0x1B][i2c_reg_addr];
      return 0;
    }
    // MAX77812 multi-phase buck (Mariko / Lite / OLED), I2C_5 @ 0x33 for the
    // PHASE211 retail variant (0x31 is the PHASE31 dev-kit part, left NAK'd).
    // Replaces the dual MAX77621 on Erista. Rails: M1 = GPU, M3 = DRAM,
    // M4 = CPU; vout_mv = 250 + N * 5.
    if (on_i2c5 && i2c_slave_addr == 0x33) {
      if (state->pmic_otp.load() != 0x53) return 0; // Mariko-family only
      max77812_regs_init();
      return max77812_regs[i2c_reg_addr];
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
      if (i2c_reg_addr == 0x03) return 0x03; // CONTROL: BIN_FORMAT | 24H
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

// One register byte written through a packet-mode transfer: the same
// effect on the modelled chips as a normal-mode [reg, value] write.
static void i2c_packet_reg_write(EmuState *state, bool on_i2c5, uint8_t dev,
                                 uint8_t reg, uint8_t v) {
  if (!on_i2c5)
    return;
  bool mariko = state->pmic_otp.load() == 0x53;
  if (dev == MAX77620_I2C_ADDR) {
    max77620_regs_init(state);
    max77620_regs[reg] = v;
    if (reg == 0x41 && (v & 0x02)) {            // ONOFFCNFG1_PWR_OFF
      printf("[emu] MAX77620 PWR_OFF received - exiting\n");
      fflush(stdout);
      state->running = false;
    } else if (reg == 0x41 && (v & 0x80)) {     // ONOFFCNFG1_SFT_RST
      printf("[emu] MAX77620 SFT_RST received - rebooting payload\n");
      fflush(stdout);
      request_reboot(state, true);
    }
    if (reg == 0x3B || reg == 0x40)
      ccplex_rail_changed(state);
  } else if (!mariko && (dev == 0x1B || dev == 0x1C) && reg < 8 &&
             reg != 0x04 && reg != 0x05) {
    max77621_regs[dev - 0x1B][reg] = v;
    ccplex_rail_changed(state);
  } else if (mariko && dev == 0x33 && reg != 0x14) {
    max77812_regs_init();
    max77812_regs[reg] = v;
    ccplex_rail_changed(state);
  }
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
    // CPU/GPU rail bucks. Same wire format; only the chip this SoC
    // generation is fitted with takes the write (the other one NAKs).
    if ((val & (1u << 9)) && is_write && size >= 2 && on_i2c5) {
      uint8_t reg = (uint8_t)(i2c_cmd_data1 & 0xFF);
      uint8_t v   = (uint8_t)((i2c_cmd_data1 >> 8) & 0xFF);
      bool mariko = state->pmic_otp.load() == 0x53;
      if (!mariko && (i2c_slave_addr == 0x1B || i2c_slave_addr == 0x1C) &&
          reg < 8 && reg != 0x04 && reg != 0x05) {        // CHIPID is RO
        max77621_regs[i2c_slave_addr - 0x1B][reg] = v;
        ccplex_rail_changed(state);
      } else if (mariko && i2c_slave_addr == 0x33 && reg != 0x14) {
        max77812_regs_init();
        max77812_regs[reg] = v;
        ccplex_rail_changed(state);
      }
    }
    if ((val & (1u << 9)) && is_write && size >= 2 && on_i2c5 &&
        i2c_slave_addr == MAX77620_I2C_ADDR &&
        ((i2c_cmd_data1 & 0xFF) == 0x3B || (i2c_cmd_data1 & 0xFF) == 0x40))
      ccplex_rail_changed(state);   // GPIO5 / AME_GPIO: the MAX77621 EN pin
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
        request_reboot(state, true);
      }
    }
    break;
  case 0x50: {        // I2C_TX_FIFO  (packet-mode dispatch)
    // The word stream bdk's _i2c_send_packet / i2c_xfer_packet push:
    //   PROT_I2C, size-1, header (addr << 1 | READ/REP_START...), then the
    //   payload packed four bytes to a word, little-endian.
    // A combined write+read sends its second packet straight after the
    // first, with no FIFO flush, so a packet ends when its payload has been
    // seen. The PROT marker is only looked for at a packet boundary: as a
    // data word, 0x10 is just a register number or a length.
    switch (pkt.hdr_idx) {
    case 0:
      if (val == I2C_PACKET_PROT_I2C)
        pkt.hdr_idx = 1;
      break;
    case 1:
      pkt.payload_size = (val & 0xFFF) + 1;
      pkt.hdr_idx = 2;
      break;
    case 2:
      pkt.dev_addr = (val >> 1) & 0x7F;
      pkt.is_read  = (val & I2C_HEADER_READ) != 0;
      i2c_slave_addr = pkt.dev_addr; // mirror for any cross-path lookups
      pkt.tx_seen = 0;
      if (pkt.is_read) {
        packet_populate_rx(state, on_i2c5, pkt);
        pkt.hdr_idx = 0;
      } else {
        pkt.hdr_idx = 3;
      }
      break;
    default:
      for (int k = 0; k < 4 && pkt.tx_seen < pkt.payload_size; k++) {
        uint8_t b = (uint8_t)(val >> (8 * k));
        if (pkt.tx_seen == 0)
          pkt.reg_addr = b;   // first payload byte: the register address
        else
          i2c_packet_reg_write(state, on_i2c5, pkt.dev_addr,
                               (uint8_t)(pkt.reg_addr + pkt.tx_seen - 1), b);
        pkt.tx_seen++;
      }
      if (pkt.tx_seen >= pkt.payload_size)
        pkt.hdr_idx = 0;
      break;
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
//
// The display controller's windows (TRM 24.10-24.11) are modelled as one
// register file per window, A-D. The CPU reaches them either through the
// indirect window pages (0x700-0x7FF, 0x800-0x83F), which write every window
// DC_CMD_DISPLAY_WINDOW_HEADER selects and read back the lowest selected one,
// or through a window's own direct range (A: 0xB80/0xBC0, B: 0xD80/0xDC0,
// C: 0xF80/0xFC0). Writes go to the assembly copy. WIN_x_ACT_REQ in
// DC_CMD_STATE_CONTROL promotes window x to the active copy that the scan-out
// (display/sdl_display.cpp) decodes, so a half-programmed window never shows.
// (The hardware stages this twice, UPDATE then ACT_REQ; every payload writes
// both back to back, so one step is enough.)
//
// The rest of the DC is plain read-back storage, except the two words that
// payloads poll: STATE_CONTROL (requests complete at once) and INT_STATUS
// (a frame has always just ended).

static constexpr uint32_t kDcCmdIntStatus     = 0x037;
static constexpr uint32_t kDcCmdStateControl  = 0x041;
static constexpr uint32_t kDcCmdWindowHeader  = 0x042;
static constexpr uint32_t kDcIntFrameEndVBlank = (1u << 1) | (1u << 2);

// Resolve a DC word index to the windows it addresses and the register-file
// slot. False for anything that is not a window register.
static bool dc_window_reg(const EmuState *state, uint32_t index,
                          uint32_t *mask, uint32_t *slot) {
  if (index >= 0x700 && index < 0x800) {
    *mask = (state->dc_window_header >> 4) & 0xF;
    *slot = index - 0x700;
    return true;
  }
  if (index >= 0x800 && index < 0x840) {
    *mask = (state->dc_window_header >> 4) & 0xF;
    *slot = 0x100 + (index - 0x800);
    return true;
  }
  static const uint32_t kDirect[3] = {0xB80, 0xD80, 0xF80};
  for (uint32_t w = 0; w < 3; w++) {
    uint32_t base = kDirect[w];
    if (index >= base && index < base + 0x80) {
      *mask = 1u << w;
      *slot = index < base + 0x40 ? index - base : 0x100 + (index - base - 0x40);
      return true;
    }
  }
  return false;
}

static void dc_reset(EmuState *state) {
  memset(state->dc_win, 0, sizeof(state->dc_win));
  memset(state->dc_win_active, 0, sizeof(state->dc_win_active));
  for (int w = 0; w < 4; w++)
    state->dc_win[w][dcwin::BLEND_LAYER] =
        state->dc_win_active[w][dcwin::BLEND_LAYER] = dcwin::BLEND_LAYER_RESET;
  state->dc_window_header = 0;
  state->dc_programmed = false;
  state->vic_out_addr = 0;
  state->vic_out_xform = 0;
}

// One line per window whose picture changed (not for moves: bdk slides its
// log window in 16 pixels at a time).
static void dc_log_window(int w, const uint32_t *r) {
  if (!(r[dcwin::OPTIONS] & dcwin::WIN_ENABLE)) {
    printf("[display] window %c off\n", 'A' + w);
    return;
  }
  static const char *const kKind[4] = {"pitch", "tiled", "block", "?"};
  uint32_t kind = r[dcwin::SURFACE_KIND];
  printf("[display] window %c: %ux%u @ 0x%08X, %s", 'A' + w,
         r[dcwin::SIZE] & 0x1FFF, (r[dcwin::SIZE] >> 16) & 0x1FFF,
         r[dcwin::START_ADDR], kKind[kind & 3]);
  if ((kind & 3) == 2)
    printf(" (%u-GOB blocks)", 1u << ((kind >> 4) & 7));
  printf(", stride %u, depth 0x%X, options 0x%X\n",
         r[dcwin::LINE_STRIDE] & 0xFFFF, r[dcwin::COLOR_DEPTH] & 0x7F,
         r[dcwin::OPTIONS]);
}

uint32_t display_read(EmuState *state, uint64_t addr) {
  uint32_t index = (uint32_t)(addr - DISPLAY_A_BASE) / 4;
  uint32_t mask, slot;
  if (dc_window_reg(state, index, &mask, &slot)) {
    for (int w = 0; w < 4; w++)
      if (mask & (1u << w))
        return state->dc_win[w][slot];
    return 0;
  }
  switch (index) {
  case kDcCmdIntStatus:    return kDcIntFrameEndVBlank;
  case kDcCmdStateControl: return 0;
  case kDcCmdWindowHeader: return state->dc_window_header;
  default:                 return mmio_regs.get(addr);
  }
}

void display_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t index = (uint32_t)(addr - DISPLAY_A_BASE) / 4;
  uint32_t mask, slot;
  if (dc_window_reg(state, index, &mask, &slot)) {
    for (int w = 0; w < 4; w++)
      if (mask & (1u << w))
        state->dc_win[w][slot] = val;
    return;
  }
  if (index == kDcCmdWindowHeader) {
    state->dc_window_header = val;
  } else if (index == kDcCmdStateControl) {
    uint32_t req = (val >> 1) & 0xF; // WIN_A..D_ACT_REQ
    for (int w = 0; w < 4; w++) {
      if (!(req & (1u << w)))
        continue;
      uint32_t *act = state->dc_win_active[w];
      const uint32_t *asm_ = state->dc_win[w];
      static const uint32_t kShown[] = {
          dcwin::OPTIONS, dcwin::COLOR_DEPTH, dcwin::SIZE, dcwin::LINE_STRIDE,
          dcwin::START_ADDR, dcwin::SURFACE_KIND};
      bool changed = !state->dc_programmed;
      for (uint32_t r : kShown)
        changed |= act[r] != asm_[r];
      memcpy(act, asm_, sizeof(state->dc_win_active[w]));
      if (changed)
        dc_log_window(w, act);
    }
    if (req) {
      state->dc_programmed = true;
      state->display_dirty = true;
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

// ---- SD host controller registers beyond the command model ---------------
//
// A byte-exact shadow of each controller's 512-byte register block, for what
// the command/data model below does not own: POWER_CONTROL, CLOCK_CONTROL,
// SW_RESET, the interrupt enables, HOST_CONTROL2 and the Tegra vendor block
// at 0x100..0x1FF (TRM 32.9.2). These all read 0 before, whatever was
// written - and bdk decides the eMMC bus mode from them: POWER_CONTROL
// reading "off" made _mmc_storage_enable_highspeed fall back to HS52, so the
// eMMC ran at 51 MHz while claiming HS400. HS200 then needs the tuning
// handshake, HS400 the tuned tap and a DLL calibration; all three are here.
struct SdhciRegs {
  uint8_t  b[0x200];
  // Hardware tuning (TRM 32.9.2.14): iteration count since EXECUTE_TUNING
  // was set, the 256-bit per-iteration pass map, the best window found.
  uint32_t tune_i;
  uint32_t tune_map[8];
  uint8_t  win_first, win_last;
  // A tuning block sent with the card clock stopped, delivered when it runs.
  bool     tune_pending;
};
static SdhciRegs sdhci_regs[2];   // [0] SDMMC1, [1] SDMMC4

// Trimmer taps at which the emulated eMMC's tuning block samples cleanly.
// At 200 MHz a unit interval is ~70 taps; a healthy part passes over most
// of one eye. Deterministic, so every run tunes to the same tap.
static constexpr uint32_t kTunePassFirst = 26, kTunePassLast = 82;

static SdhciRegs &sdhci_of(uint32_t base) {
  return sdhci_regs[base == SDMMC4_BASE ? 1 : 0];
}
static uint32_t sdhci_get32(const SdhciRegs &r, uint32_t off) {
  return (uint32_t)r.b[off] | ((uint32_t)r.b[off + 1] << 8) |
         ((uint32_t)r.b[off + 2] << 16) | ((uint32_t)r.b[off + 3] << 24);
}
static void sdhci_put32(SdhciRegs &r, uint32_t off, uint32_t v) {
  for (int i = 0; i < 4; i++)
    r.b[off + i] = (uint8_t)(v >> (8 * i));
}

// Power-on state: the standard registers clear, the vendor block at its
// TRM reset values (the SDMMC2/4 set, which carries the DLL registers).
static void sdhci_reset_regs(SdhciRegs &r) {
  memset(&r, 0, sizeof(r));
  static const struct { uint16_t off; uint32_t val; } kReset[] = {
      {0x100, 0x0000D02D}, {0x104, 0x38600002}, {0x10C, 0x00000007},
      {0x11C, 0x00000C80}, {0x120, 0xFFFF0098}, {0x1AC, 0x00000015},
      {0x1B0, 0x16083504}, {0x1B4, 0x60000000}, {0x1B8, 0x20208780},
      {0x1BC, 0x10000000}, {0x1C0, 0x74020090}, {0x1C4, 0x00000023},
      {0x1D0, 0x0000000F}, {0x1D4, 0x00201000}, {0x1D8, 0x00100804},
      {0x1DC, 0x00000800}, {0x1E0, 0x88000001}, {0x1E4, 0x00010000},
      {0x1F8, 0x00000032},
  };
  for (const auto &e : kReset)
    sdhci_put32(r, e.off, e.val);
}

static bool sdhci_shadowed(uint32_t off) {
  return (off >= 0x28 && off < 0x40) || (off >= 0x100 && off < 0x200);
}

static uint8_t sdhci_byte(EmuState *s, uint32_t base, uint32_t off) {
  if (off >= 0x200)
    return 0;
  SdhciRegs &r = sdhci_of(base);
  bool e = base == SDMMC4_BASE;
  uint32_t nor = e ? s->sdmmc4_norintsts : s->sdmmc_norintsts;
  uint32_t err = e ? s->sdmmc4_errintsts : s->sdmmc_errintsts;
  switch (off) {
  case 0x28:
    return e ? s->sdmmc4_hostctl : s->sdmmc_hostctl;
  case 0x2C: {
    // CLOCK_CONTROL: the internal clock is stable as soon as it is enabled.
    uint8_t c = r.b[0x2C] & ~0x02;
    return (c & 0x01) ? (uint8_t)(c | 0x02) : c;
  }
  case 0x2F:
    return 0;                     // SW_RESET: completes at once
  case 0x30: case 0x31:
    // NORMAL_INT_STATUS bit 15, ERR_INTERRUPT, is the OR of every error
    // bit (SDHCI 2.2.18) - what BDK's _sdmmc_check_mask_interrupt tests.
    return (uint8_t)((nor | (err ? 0x8000u : 0u)) >> (8 * (off - 0x30)));
  case 0x32: case 0x33:
    return (uint8_t)(err >> (8 * (off - 0x32)));
  case 0x3C: case 0x3D:
    return 0;                     // AUTO_CMD_ERROR_STATUS
  case 0x1B3:
    return r.b[off] & 0x7F;       // DLLCAL_CFG.CALIBRATE self-clears
  case 0x1BF:
    return r.b[off] & 0x7F;       // DLLCAL_CFG_STA.ACTIVE: calibration done
  case 0x1C8: case 0x1C9: case 0x1CA: case 0x1CB: {
    // VENDOR_TUNING_STATUS0: the pass map word TUNING_WORD_SEL selects.
    uint32_t w = r.tune_map[r.b[0x1C0] & 7];
    return (uint8_t)(w >> (8 * (off - 0x1C8)));
  }
  case 0x1CC: case 0x1CE: return r.win_first;   // start, after / before fine
  case 0x1CD: case 0x1CF: return r.win_last;    // end,   after / before fine
  case 0x1E7:
    return r.b[off] & 0x7F;       // AUTO_CAL_CONFIG.START self-clears
  case 0x1EC:
    return 0x01;                  // AUTO_CAL_STATUS: done, pull-up code 1
  case 0x1ED: case 0x1EE: case 0x1EF:
    return 0;                     // ...ACTIVE (bit 31) clear
  default:
    return r.b[off];
  }
}

static uint32_t sdhci_read(EmuState *s, uint32_t base, uint32_t off) {
  uint32_t v = 0;
  for (int i = 3; i >= 0; i--)
    v = (v << 8) | sdhci_byte(s, base, off + (uint32_t)i);
  return v;
}

// SW_RESET (0x2F). RESET_ALL puts the standard registers back to power-on
// and leaves the vendor block's trims alone; RESET_CMD / RESET_DATA clear
// the status their line owns (SDHCI 2.2.12).
static void sdhci_sw_reset(EmuState *s, uint32_t base, uint8_t v) {
  bool e = base == SDMMC4_BASE;
  uint32_t &nor = e ? s->sdmmc4_norintsts : s->sdmmc_norintsts;
  uint32_t &err = e ? s->sdmmc4_errintsts : s->sdmmc_errintsts;
  if (v & 0x01) {
    SdhciRegs &r = sdhci_of(base);
    uint8_t vendor[0x100];
    memcpy(vendor, r.b + 0x100, sizeof(vendor));  // vendor block survives
    sdhci_reset_regs(r);
    memcpy(r.b + 0x100, vendor, sizeof(vendor));
    nor = err = 0;
    (e ? s->sdmmc4_hostctl : s->sdmmc_hostctl) = 0;
    return;
  }
  if (v & 0x02) {                 // CMD line: command complete, CMD errors
    nor &= ~0x0001u;
    err &= ~0x000Fu;
  }
  if (v & 0x04) {                 // DAT line: transfer/DMA/buffer, DAT errors
    nor &= ~0x003Eu;
    err &= ~0x0270u;
  }
}

// A tuning block arriving: BUFFER_READ_READY.
static void sdhci_deliver_tuning_block(EmuState *s, uint32_t base) {
  (base == SDMMC4_BASE ? s->sdmmc4_norintsts : s->sdmmc_norintsts) |= 0x0020;
}

static void sdhci_write(EmuState *s, uint32_t base, uint32_t offset,
                        uint64_t value, int size) {
  SdhciRegs &r = sdhci_of(base);
  for (int i = 0; i < size; i++) {
    uint32_t off = offset + (uint32_t)i;
    if (off >= 0x200)
      break;
    uint8_t v = (uint8_t)(value >> (8 * i));
    switch (off) {
    case 0x28:                                  // HOST_CONTROL: command model
    case 0x30: case 0x31: case 0x32: case 0x33: // W1C status: command model
    case 0x3C: case 0x3D:                       // read-only
    case 0x1BC: case 0x1BD: case 0x1BE: case 0x1BF:
    case 0x1C8: case 0x1C9: case 0x1CA: case 0x1CB:
    case 0x1CC: case 0x1CD: case 0x1CE: case 0x1CF:
    case 0x1EC: case 0x1ED: case 0x1EE: case 0x1EF:
      break;
    case 0x2C: {
      bool was_on = r.b[0x2C] & 0x04;
      r.b[0x2C] = v;
      // Tegra sends a tuning command with SD_CLK stopped, resets CMD/DAT,
      // then restarts the clock: the block comes in now.
      if (!was_on && (v & 0x04) && r.tune_pending) {
        r.tune_pending = false;
        sdhci_deliver_tuning_block(s, base);
      }
      break;
    }
    case 0x2F:
      sdhci_sw_reset(s, base, v);
      break;
    case 0x3E: {
      // HOST_CONTROL2 low byte. EXECUTE_TUNING (bit 6) belongs to the
      // hardware once set: writing 1 starts tuning - clearing the sampling
      // clock select and the pass map - and only the tuning circuit clears
      // it again. SAMPLING_CLOCK_SELECT (bit 7) takes what is written.
      uint8_t cur = r.b[0x3E];
      uint8_t nv = (uint8_t)((v & ~0x40) | (cur & 0x40));
      if ((v & 0x40) && !(cur & 0x40)) {
        nv = (uint8_t)((nv | 0x40) & ~0x80);
        r.tune_i = 0;
        memset(r.tune_map, 0, sizeof(r.tune_map));
        r.win_first = r.win_last = 0;
      }
      r.b[0x3E] = nv;
      break;
    }
    default:
      r.b[off] = v;
      break;
    }
  }
}

// SEND_TUNING_BLOCK (CMD19 SD, CMD21 eMMC HS200) with the Tegra tuning
// circuit (TRM 32.9.2.14). With EXECUTE_TUNING set each block is one
// iteration at the next trimmer tap - START_TAP_VAL plus 2^STEP_SIZE per
// iteration when TAP_VAL_UPDATED_BY_HW is set, the programmed tap
// otherwise - and SAMPLING_CLOCK_SELECT reports whether it sampled
// cleanly. After NUM_TUNING_ITERATIONS blocks the circuit clears
// EXECUTE_TUNING; in hardware-tap mode it then sets the final tap
// first_pass + (last_pass - first_pass) * (MUL_M + 1) / 2^DIV_N and
// leaves SAMPLING_CLOCK_SELECT set if any tap passed.
static void sdhci_tuning_command(EmuState *s, uint32_t base) {
  SdhciRegs &r = sdhci_of(base);
  if (r.b[0x3E] & 0x40) {
    static const uint16_t kTries[8] = {40, 64, 128, 192, 256, 256, 256, 256};
    uint32_t tun0 = sdhci_get32(r, 0x1C0), tun1 = sdhci_get32(r, 0x1C4);
    bool hw_tap = tun0 & (1u << 17);
    uint32_t tries = kTries[(tun0 >> 13) & 7];
    bool sdr50 = (r.b[0x3E] & 0x07) == 2;
    uint32_t step = 1u << (sdr50 ? (tun1 & 7) : ((tun1 >> 4) & 7));
    uint32_t clk = sdhci_get32(r, 0x100);
    uint32_t i = r.tune_i;
    uint32_t tap = hw_tap ? ((((tun0 >> 18) & 0xFF) + i * step) & 0xFF)
                          : ((clk >> 16) & 0xFF);
    bool pass = tap >= kTunePassFirst && tap <= kTunePassLast;
    if (i < 256 && pass)
      r.tune_map[i / 32] |= 1u << (i % 32);
    if (hw_tap)
      sdhci_put32(r, 0x100, (clk & ~0x00FF0000u) | (tap << 16));
    r.b[0x3E] = (uint8_t)((r.b[0x3E] & ~0x80) | (pass ? 0x80 : 0));
    r.tune_i = i + 1;
    if (r.tune_i >= tries) {
      r.b[0x3E] &= ~0x40;                     // tuning complete
      if (hw_tap) {
        int first = -1, last = -1;
        uint32_t start = (tun0 >> 18) & 0xFF;
        for (uint32_t k = 0; k < tries && k < 256; k++) {
          if (!(r.tune_map[k / 32] & (1u << (k % 32))))
            continue;
          uint32_t t = (start + k * step) & 0xFF;
          if (first < 0)
            first = (int)t;
          last = (int)t;
        }
        if (first >= 0) {
          uint32_t m = (tun0 >> 6) & 0x7F, n = (tun0 >> 3) & 7;
          uint32_t fin = (uint32_t)first +
                         (((uint32_t)(last - first) * (m + 1)) >> n);
          if (fin > (uint32_t)last)
            fin = (uint32_t)last;
          clk = sdhci_get32(r, 0x100);
          sdhci_put32(r, 0x100, (clk & ~0x00FF0000u) | ((fin & 0xFF) << 16));
          r.win_first = (uint8_t)first;
          r.win_last = (uint8_t)last;
          r.b[0x3E] |= 0x80;
          printf("[sdmmc] SDMMC%c tuned: %u iterations, pass window taps "
                 "%d..%d, tap %u\n", base == SDMMC4_BASE ? '4' : '1',
                 tries, first, last, fin & 0xFF);
        } else {
          r.b[0x3E] &= ~0x80;
          printf("[sdmmc] SDMMC%c tuning failed: no tap passed\n",
                 base == SDMMC4_BASE ? '4' : '1');
        }
        fflush(stdout);
      }
    }
  }
  if (r.b[0x2C] & 0x04)
    sdhci_deliver_tuning_block(s, base);
  else
    r.tune_pending = true;
}

// ---- The SD card's signalling voltage and switch functions ----------------
//
// A UHS-I card (SD Physical Layer 3.01). ACMD41 with S18R set answers S18A
// while the card still signals at 3.3 V; CMD11 then moves it to 1.8 V, where
// it stays until it loses power - bdk's supply GPIO (PE4) going low, or a
// reset. A card re-initialised without a power cycle answers S18A = 0 but
// keeps offering its UHS bus speeds, which is how hosts tell. CMD6
// (SWITCH_FUNC) reports and selects one function per group: bus speed
// (group 1: SDR12 and SDR25/HS, plus SDR50, SDR104 and DDR50 at 1.8 V),
// driver strength (3: type B, plus A, C and D at 1.8 V) and power limit
// (4: 0.72 W, plus 1.44 W at 1.8 V). CMD0 puts the functions back to their
// defaults but not the signalling voltage.
struct SdCard {
  bool    s18a_offered = false; // the last ACMD41 answered S18A
  bool    v18 = false;          // signalling at 1.8 V
  bool    bus4 = false;         // ACMD6 set a 4-bit bus
  uint8_t func[6] = {};         // current function, groups 1..6
  bool    supplied = false;     // PE4 driving the card's VDD
};
static SdCard sd_card;

static uint16_t sd_func_support(int group) {
  static const uint16_t k33[6] = {0x8003, 0x8001, 0x8001, 0x8001, 0x8001, 0x8001};
  static const uint16_t k18[6] = {0x801F, 0x8001, 0x800F, 0x8003, 0x8001, 0x8001};
  return (sd_card.v18 ? k18 : k33)[group];
}

// The 64-byte CMD6 status (SD 4.3.10.4). arg bit 31: 0 check, 1 switch;
// bits 4g+3..4g: the function wanted in group g+1, 0xF for "no change".
static void sd_switch_func(uint32_t arg, uint8_t st[64]) {
  memset(st, 0, 64);
  st[1] = 100; // mA the selected functions draw; bdk refuses over 800
  uint8_t res[6];
  for (int g = 0; g < 6; g++) {
    uint16_t sup = sd_func_support(g);
    st[2 + (5 - g) * 2] = (uint8_t)(sup >> 8);
    st[3 + (5 - g) * 2] = (uint8_t)sup;
    uint32_t want = (arg >> (4 * g)) & 0xF;
    res[g] = want == 0xF ? sd_card.func[g]
           : ((sup >> want) & 1) ? (uint8_t)want : 0xF;
  }
  if (arg >> 31)
    for (int g = 0; g < 6; g++)
      if (res[g] != 0xF)
        sd_card.func[g] = res[g];
  st[14] = (uint8_t)(res[5] << 4 | res[4]);
  st[15] = (uint8_t)(res[3] << 4 | res[2]);
  st[16] = (uint8_t)(res[1] << 4 | res[0]);
  st[17] = 1; // data structure version 1: busy status valid (none busy)
}

// The 64-byte SD status (ACMD13, SD 4.10.2): the bus width ACMD6 set, and
// the card's grades - a Class 10, U1, V10 UHS-I card with 4 MB AUs.
static void sd_status(uint8_t ss[64]) {
  memset(ss, 0, 64);
  ss[0]  = sd_card.bus4 ? 0x80 : 0x00; // DAT_BUS_WIDTH
  ss[8]  = 4;                          // SPEED_CLASS: Class 10
  ss[10] = 9 << 4;                     // AU_SIZE: 4 MB
  ss[14] = 1 << 4 | 9;                 // UHS_SPEED_GRADE U1, UHS_AU_SIZE 4 MB
  ss[15] = 10;                         // VIDEO_SPEED_CLASS: V10
}

// The SD card's CSD, version 2.0 (SD 5.3.3): an SDHC/SDXC card as big as
// the image (32 GiB with none), command classes 0, 2, 4, 5, 7, 8 and 10 -
// class 8, application commands, is what makes bdk read the SD status.
// Laid out as the R2 response registers carry it, CSD[127:8]: bdk shifts
// each word left by 8 to put the stripped CRC back.
static void sd_csd(EmuState *state, uint32_t r[4]) {
  uint64_t bytes = state->sd_fd >= 0 ? (uint64_t)file_size64(state->sd_fd)
                                     : 32ull << 30;
  uint64_t units = bytes / (512u << 10);
  uint32_t c_size = units ? (uint32_t)std::min<uint64_t>(units - 1, 0x3FFFFF)
                          : 0;
  unsigned __int128 c = 0;
  auto put = [&](unsigned hi, unsigned lo, uint64_t v) {
    c |= (unsigned __int128)(v & ((1ull << (hi - lo + 1)) - 1)) << lo;
  };
  put(127, 126, 1);     // CSD_STRUCTURE: version 2.0
  put(119, 112, 0x0E);  // TAAC: 1 ms
  put(103, 96, 0x32);   // TRAN_SPEED: 25 MHz
  put(95, 84, 0x5B5);   // CCC
  put(83, 80, 9);       // READ_BL_LEN: 512
  put(69, 48, c_size);  // C_SIZE: (C_SIZE + 1) x 512 KiB
  put(46, 46, 1);       // ERASE_BLK_EN
  put(45, 39, 0x7F);    // SECTOR_SIZE
  put(28, 26, 2);       // R2W_FACTOR
  put(25, 22, 9);       // WRITE_BL_LEN: 512
  put(0, 0, 1);         // always 1
  for (int i = 0; i < 4; i++)
    r[i] = (uint32_t)(c >> (8 + 32 * i));
}

// Port E sits in GPIO bank 1: CNF 0x100, OE 0x110, OUT 0x120. PE4 high
// powers the card; falling, the card loses its state.
static void sd_card_gpio_update() {
  uint32_t m = 1u << 4;
  bool on = (mmio_regs.get(GPIO_BASE + 0x100) & m) &&
            (mmio_regs.get(GPIO_BASE + 0x110) & m) &&
            (mmio_regs.get(GPIO_BASE + 0x120) & m);
  if (sd_card.supplied && !on)
    sd_card = SdCard();
  sd_card.supplied = on;
}

// ---- The eMMC's EXT_CSD -----------------------------------------------------
//
// The properties segment is fixed; the fields CMD6 SWITCH writes (bus
// width, HS_TIMING, partition access, ...) keep what was written until the
// card is reset, so a CMD8 after the bus-mode switches reads them back.
static uint8_t g_ext_csd[512];
static bool g_ext_csd_ready = false;

static uint8_t *emmc_ext_csd() {
  if (g_ext_csd_ready)
    return g_ext_csd;
  g_ext_csd_ready = true;
  uint8_t *x = g_ext_csd;
  memset(x, 0, 512);
  x[192] = 7;          // EXT_CSD_REV: eMMC v5.0
  x[196] = 0x57;       // CARD_TYPE: HS400_1.8V | HS200_1.8V | DDR_1.8V | HS_52 | HS_26
  // 32GB worth of sectors (BDK only uses sec_cnt for sanity, not for read
  // addressing).
  uint32_t sec_cnt = 64 * 1024 * 1024;
  x[212] = sec_cnt & 0xFF;
  x[213] = (sec_cnt >> 8) & 0xFF;
  x[214] = (sec_cnt >> 16) & 0xFF;
  x[215] = (sec_cnt >> 24) & 0xFF;
  // BOOT0/BOOT1 and RPMB are 4 MiB each on the Switch's eMMC: 32 x 128 KiB.
  x[226] = 32;         // BOOT_SIZE_MULT
  x[168] = 32;         // RPMB_SIZE_MULT
  // Capability/health bytes a diagnostic payload reports on. Values
  // measured from the eMMC in a real Mariko; left at 0 these read as
  // "feature not supported", which looks like a reduced-firmware
  // replacement part.
  x[503] = 0x01;       // HPI_FEATURES: supported, CMD13 variant
  x[231] = 0x55;       // SEC_FEATURE_SUPPORT: secure erase/trim/sanitize
  x[232] = 0x02;       // TRIM_MULT
  x[229] = 0x11;       // SEC_TRIM_MULT (measured)
  x[162] = 0x01;       // RST_N_FUNCTION: permanently enabled
  x[267] = 0x01;       // PRE_EOL_INFO: normal
  x[268] = 0x01;       // DEVICE_LIFE_TIME_EST_TYP_A: 0-10%
  x[269] = 0x01;       // DEVICE_LIFE_TIME_EST_TYP_B: 0-10%
  return x;
}

// CMD0 / power-on: back to 1-bit legacy timing, in the user area.
static void emmc_ext_csd_card_reset() {
  uint8_t *x = emmc_ext_csd();
  x[183] = 0;          // BUS_WIDTH
  x[185] = 0;          // HS_TIMING
  x[179] &= ~0x07;     // PARTITION_ACCESS
}

// CMD6 SWITCH, access modes 1 = set bits, 2 = clear bits, 3 = write byte,
// to the fields a host may change.
static void emmc_ext_csd_switch(uint32_t arg) {
  uint32_t access = (arg >> 24) & 3, index = (arg >> 16) & 0xFF;
  uint8_t value = (uint8_t)(arg >> 8);
  switch (index) {
  case 33:  // CACHE_CTRL
  case 34:  // POWER_OFF_NOTIFICATION
  case 161: // HPI_MGMT
  case 163: // BKOPS_EN
  case 175: // ERASE_GROUP_DEF
  case 177: // BOOT_BUS_CONDITIONS
  case 179: // PARTITION_CONFIG
  case 183: // BUS_WIDTH
  case 185: // HS_TIMING
  case 187: // POWER_CLASS
    break;
  default:
    return;
  }
  uint8_t *x = emmc_ext_csd();
  if (access == 1)
    x[index] |= value;
  else if (access == 2)
    x[index] &= (uint8_t)~value;
  else if (access == 3)
    x[index] = value;
}

uint32_t misc_read(EmuState *state, uint64_t addr) {
  // PINMUX (APB_MISC pad config range) — every write lands in the global
  // mmio_regs map at line 1726, so we just hand it back. Returning 0 here
  // (the old behavior) silently dropped the muxed function bits, which
  // broke any probe that read back PINMUX_AUX_* to confirm a pin's mode.
  if (addr >= PINMUX_BASE && addr < PINMUX_BASE + PINMUX_SIZE) {
    return mmio_regs.get(addr, pinmux_reset_default(addr));
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
  // The rest of APB_MISC (pad control, GP_* configuration, PINMUX_GLOBAL):
  // R/W registers that read back what was written. They read 0 before even
  // though every write was cached, so a read-modify-write lost its bits.
  if (addr >= APB_MISC_BASE && addr < PINMUX_BASE)
    return mmio_regs.get(addr);

  // UART
  if (addr >= 0x70006000 && addr < 0x70006500) {
    uint32_t offset = 0;
    int port = uart_port_of(addr, &offset);
    if (port < 0)  // the gaps between the five register blocks
      return mmio_regs.get(addr);
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
      // In internal loopback the modem inputs come from this port's own
      // outputs instead (TRM 36.x): RTS -> CTS, DTR -> DSR, OUT1 -> RI,
      // OUT2 -> DCD.
      if (up.mcr & 0x10)
        msr = up.msr_delta | ((up.mcr & 0x02) ? 0x10 : 0) |
              ((up.mcr & 0x01) ? 0x20 : 0) | ((up.mcr & 0x04) ? 0x40 : 0) |
              ((up.mcr & 0x08) ? 0x80 : 0);
      up.msr_delta = 0;
      return msr;
    }
    // With DLAB set, 0x00 and 0x04 are the divisor latches, NOT RBR/IER.
    // Honouring that matters twice over: a payload reading back its own baud
    // divisor gets the real value, and - more importantly - reading the
    // divisor no longer POPS a byte off the receive FIFO. That silently ate
    // host bytes, which is corruption waiting to happen on a sideload.
    uint32_t base = uart_bases[port];
    bool dlab = (mmio_regs.get(base + 0x0C)) & 0x80;

    if (offset == 0x00 && dlab)
      return up.divisor & 0xFF;         // DLL
    if (offset == 0x04 && dlab)
      return (up.divisor >> 8) & 0xFF;  // DLM
    if (offset == 0x04)
      return up.ier;
    if (offset == 0x08) {
      // IIR, read-only (FCR is the write-only register at the same offset):
      // 7:6 set while the FIFOs are enabled, then the highest-priority
      // pending interrupt - received data, then THR empty - or 1 for none.
      uint32_t iir = (up.fcr & 0x01) ? 0xC0 : 0x00;
      if ((up.ier & 0x01) && !state->uart_rx_fifo[port].empty())
        return iir | 0x04;
      if (up.ier & 0x02)
        return iir | 0x02;
      return iir | 0x01;
    }

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
    return mmio_regs.get(addr);
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
    // The rest are configuration registers (arbitration, latency allowance,
    // carveouts, the EMEM address map sdram_init programs from the BCT):
    // they read back what was written.
    return mmio_regs.get(addr);
  }
  // PWM controller (LCD backlight on PWM0, optional fan on PWM1). Same
  // story as PINMUX above — the writes are captured by the global cache
  // already; return whatever was last written so the payload can read
  // back its own duty cycle / enable bit.
  if (addr >= PWM_BASE && addr < PWM_BASE + 0x1000) {
    return mmio_regs.get(addr);
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
    uint16_t blksize =
        (base == SDMMC4_BASE) ? state->sdmmc4_blksize : state->sdmmc_blksize;
    uint16_t blkcnt =
        (base == SDMMC4_BASE) ? state->sdmmc4_blkcnt : state->sdmmc_blkcnt;
    uint16_t trnmod =
        (base == SDMMC4_BASE) ? state->sdmmc4_trnmod : state->sdmmc_trnmod;

    uint32_t result = 0;
    if (sdhci_shadowed(offset))
      result = sdhci_read(state, base, offset);
    else if (offset == 0x00)
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
    else if (offset == 0x40)
      result =
          0x376CD08C; // Full Tegra capabilities (64-bit, SDMA, ADMA2, etc.)
    else if (offset == 0x44)
      result = 0x10002F73; // CAP1
    else if (offset >= 0x10 && offset <= 0x1C)
      result = rsp[(offset - 0x10) / 4];
    else if (offset == 0xFE)
      result = 0x0303; // SDHCI Version 4.0

    TRACE("[sdmmc%c] R: 0x%02lX = 0x%08X (PC=0x%llX)\n",
           (base == SDMMC4_BASE) ? '4' : '1', (unsigned long)offset,
           result, (unsigned long long)state->insn_count);
    return result;
  }
  return 0;
}

// Where a single-buffer data phase (EXT_CSD, CMD6 status, ACMD13, SCR) lands.
// Tegra's SDMA takes its system address from 0x58 (bdk _sdmmc_dma_init
// writes it there, SD Host Controller v4 style), falling back to the
// standard 0x00 SDMA register. Under ADMA2 it is the first descriptor's
// buffer: 32-bit address in an 8-byte descriptor, 64-bit in a 12-byte one.
static uint64_t sdmmc_dma_target(uc_engine *uc, uint8_t hostctl,
                                 uint64_t adma_addr, uint32_t sysad) {
  uint32_t dmasel = hostctl & 0x18;
  if (dmasel == 0x10 || dmasel == 0x18) {
    uint8_t d[12] = {0};
    if (uc_mem_read(uc, adma_addr, d, dmasel == 0x18 ? 12 : 8) != UC_ERR_OK)
      return 0;
    uint64_t a = (uint64_t)d[4] | ((uint64_t)d[5] << 8) |
                 ((uint64_t)d[6] << 16) | ((uint64_t)d[7] << 24);
    if (dmasel == 0x18)
      a |= ((uint64_t)d[8] | ((uint64_t)d[9] << 8) | ((uint64_t)d[10] << 16) |
            ((uint64_t)d[11] << 24)) << 32;
    return a;
  }
  return adma_addr ? adma_addr : sysad;
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

    TRACE("[sdmmc%c] W: 0x%02X = 0x%08X (size %d)\n",
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

    if (sdhci_shadowed(offset))
      sdhci_write(state, base, offset, value, size);
    if (offset == 0x00)
      sysad = val;
    if (offset == 0x28)
      // 0x3E, not 0x1E: the old mask dropped SDHCI_CTRL_8BITBUS (BIT(5)), so
      // an eMMC 8-bit bus could never be represented.
      hostctl = val & 0x3E;
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
        TRACE("[sdmmc] W: offset 0x%X = 0x%08X (PC=0x%08llX)\n", offset, val,
               (unsigned long long)state->insn_count);
        if (offset == 0x0E || (offset == 0x0C && size == 4)) {
          TRACE("[sdmmc] CMD%d: arg=0x%08X, blkcnt=%d, trnmod=0x%04X\n", cmd,
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
        // GO_IDLE_STATE: the eMMC comes back in its user area (PARTITION_
        // ACCESS = 0). Leaving BOOT0 selected made the next init read the
        // GPT - and write user data - into the boot partition's image.
        if (base == SDMMC4_BASE) {
          state->emmc_partition = 0;
          emmc_ext_csd_card_reset();
        } else {
          // Functions back to default; the signalling voltage stays.
          memset(sd_card.func, 0, sizeof(sd_card.func));
          sd_card.s18a_offered = false;
          sd_card.bus4 = false;
        }
        break;
      case 11: // SD VOLTAGE_SWITCH (R1, card in READY)
        if (base == SDMMC1_BASE && sd_card.s18a_offered) {
          sd_card.v18 = true;
          sd_card.s18a_offered = false;
          rsp[0] = r1_base | (1 << 9);
        } else {
          rsp[0] = r1_base | (1 << 9) | (1u << 22); // ILLEGAL_COMMAND
        }
        break;
      case 19: // SD SEND_TUNING_BLOCK
      case 21: // eMMC SEND_TUNING_BLOCK_HS200
        sdhci_tuning_command(state, base);
        rsp[0] = r1_base | (4 << 9) | (1u << 8);
        break;
      case 8:
        rsp[0] = (base == SDMMC4_BASE) ? 0x00000900 : arg;
        // eMMC SEND_EXT_CSD: 512-byte data transfer to DMA dst. Without this,
        // sdmmc_storage_init_mmc times out waiting for transfer complete and
        // bails with storage->initialized = 0 — which silently breaks all
        // later sdmmc_storage_read calls (returns 0 without issuing CMD18).
        if (base == SDMMC4_BASE) {
          const uint8_t *ext_csd = emmc_ext_csd();

          uint64_t dma_addr = 0;
          // Tegra's SDMMC uses register 0x58 as the SDMA system-address
          // register (Hekate writes the destination buffer pointer directly
          // there in sdmmc_driver.c::_sdmmc_dma_init). adma_addr in our state
          // captures that write, so it IS the destination, not a descriptor
          // pointer. Fall back to sysad (SDHCI standard 0x00) if 0x58 wasn't
          // programmed.
          dma_addr = sdmmc_dma_target(uc, hostctl, adma_addr, sysad);
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
        if (base == SDMMC1_BASE) {
          sd_csd(state, rsp);
          break;
        }
        rsp[0] = 0x400E0032;
        rsp[1] = 0x5B590000;
        rsp[2] = 0x00007F80;
        rsp[3] = 0x16504000; // bits 18-21 = 0100 → mmca_vsn = 4
        break;
      case 13: { // SEND_STATUS / ACMD13 (SD_STATUS)
        bool is_acmd = (base == SDMMC1_BASE) ? state->last_cmd_was_55
                                             : state->last_cmd4_was_55;
        if (is_acmd) {
          uint8_t ss[64];
          sd_status(ss);
          uint64_t dma_addr = sdmmc_dma_target(uc, hostctl, adma_addr, sysad);
          if (dma_addr)
            uc_mem_write(uc, dma_addr, ss, 64);
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
          // ACMD6: SET_BUS_WIDTH (arg 2 = 4-bit)
          if (base == SDMMC1_BASE)
            sd_card.bus4 = (arg & 3) == 2;
          rsp[0] = r1_base | (4 << 9);
        } else if (base == SDMMC1_BASE) {
          // CMD6: SWITCH_FUNC (SD)
          uint8_t status[64];
          sd_switch_func(arg, status);

          uint64_t dma_addr = sdmmc_dma_target(uc, hostctl, adma_addr, sysad);
          if (dma_addr)
            uc_mem_write(uc, dma_addr, status, 64);
          norintsts |= 0x0002; // Transfer Complete
          rsp[0] = r1_base | (4 << 9);
        } else if (base == SDMMC4_BASE) {
          // CMD6: SWITCH (eMMC)
          emmc_ext_csd_switch(arg);
          if (((arg >> 16) & 0xFF) == 179) { // PARTITION_CONFIG
            state->emmc_partition = emmc_ext_csd()[179] & 0x7;
            TRACE("[sdmmc] eMMC Partition Switch: %u\n",
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
      case 41: // SD_SEND_OP_COND: powered up, CCS, 2.7-3.6 V; S18A per S18R
        sd_card.s18a_offered = base == SDMMC1_BASE && (arg & (1u << 24)) &&
                               !sd_card.v18;
        rsp[0] = 0xC0FF8000 | (sd_card.s18a_offered ? (1u << 24) : 0);
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
          // 1-bit HS25 and hwtest flags "1-bit (dirty slot?)". Byte 2 bit 7,
          // SD_SPEC3: Physical Layer 3.0x, as a UHS-I card is.
          uint8_t scr[8] = {0x02, 0x35, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00};
          // Tegra drives this small read over SDMA with the destination in
          // register 0x58 (captured here as adma_addr), NOT the SDHCI-standard
          // sysad (0x00), which Tegra leaves unused. Matches the EXT_CSD path.
          uint64_t dma_addr = sdmmc_dma_target(uc, hostctl, adma_addr, sysad);
          if (dma_addr)
            uc_mem_write(uc, dma_addr, scr, 8);
          norintsts |= 0x0002;
        } else {
          TRACE("[sdmmc] ERROR: Storage command %d on base 0x%llX but fd is "
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

        {   // fd == -1: no image behind it - do_io serves it as blank
          uint64_t sector = arg;
          uint64_t file_off = sector * 512;
          uint16_t bcnt =
              (blkcnt == 0 && (cmd == 18 || cmd == 25)) ? 1 : blkcnt;
          if (bcnt == 0)
            bcnt = 1;
          if (cmd == 17 || cmd == 18) {
            TRACE("[sdmmc] %s READ CMD%d: Sector = %llu, Count = %u (part=%u)\n",
                   (base == SDMMC1_BASE) ? "SD" : "eMMC", cmd,
                   (unsigned long long)sector, bcnt,
                   (base == SDMMC1_BASE) ? 0 : state->emmc_partition);
          }
          size_t xfer_len = bcnt * 512;

          // A write that did not land, or a DMA address nothing backs, ends
          // the command with a data error instead of a silent "complete".
          bool io_err = false;
          // Read `len` bytes at `off` of one image file into buf. Anything
          // past its end reads as blank, as on a card larger than its dump.
          auto file_read = [&](int f, uint64_t off, uint8_t *buf, size_t len) {
            ssize_t res = pread(f, buf, len, (off_t)off);
            if (res < 0)
              res = 0;
            if ((size_t)res < len)
              TRACE("[sdmmc] short read at 0x%llX: %zd of %zu bytes\n",
                    (unsigned long long)off, res, len);
          };
          auto file_write = [&](int f, uint64_t off, const uint8_t *buf,
                                size_t len) {
            ssize_t res = pwrite(f, buf, len, (off_t)off);
            if (res != (ssize_t)len) {
              printf("[sdmmc] write of %zu bytes at 0x%llX failed (%zd)\n", len,
                     (unsigned long long)off, res);
              io_err = true;
            }
          };

          auto do_io = [&](int current_fd, uint64_t current_off,
                           uint64_t dma_addr, size_t len) {
            std::vector<uint8_t> io_buf(len, 0);
            if (!is_read && uc_mem_read(uc, dma_addr, io_buf.data(), len) != UC_ERR_OK) {
              io_err = true;
              return;
            }
            if (current_fd == -2) { // GPP Spanning
              // No rawnand image loaded: serve a synthesized valid GPT so the
              // [eMMC GPT] probe sees "EFI PART" with good CRCs instead of
              // "signature missing". Everything outside LBA 0..2 reads as zero.
              if (state->emmc_gpp_fds.empty()) {
                if (is_read) {
                  size_t glen = 0;
                  const uint8_t *gpt = emmc_synth_gpt(&glen);
                  for (size_t o = 0; o < len; o++) {
                    uint64_t abs = current_off + o;
                    if (abs < glen) io_buf[o] = gpt[abs];
                  }
                }
              } else {
                // Hekate-style rawnand splitting can use 2GB or 4GB chunks.
                // Stat the first part once and cache; assumes all but the
                // last part are the same size (true for both Hekate's
                // standard 4GB-FAT32-friendly splits and tools that use 2GB
                // chunks).
                static uint64_t part_size = 0;
                if (!part_size) {
                  int64_t sz = file_size64(state->emmc_gpp_fds[0]);
                  part_size = sz > 0 ? (uint64_t)sz : 4ULL * 1024 * 1024 * 1024;
                  TRACE("[sdmmc] GPP part size detected: %llu bytes\n",
                        (unsigned long long)part_size);
                }
                // A transfer can straddle two parts (a 2 GiB split falls in
                // the middle of SYSTEM): go part by part. Past the last part
                // the device is blank; writes there have nowhere to go.
                size_t done = 0;
                while (done < len) {
                  uint64_t off = current_off + done;
                  size_t idx = (size_t)(off / part_size);
                  uint64_t part_off = off % part_size;
                  size_t n = (size_t)std::min<uint64_t>(len - done, part_size - part_off);
                  if (idx < state->emmc_gpp_fds.size()) {
                    int real_fd = state->emmc_gpp_fds[idx];
                    if (is_read)
                      file_read(real_fd, part_off, io_buf.data() + done, n);
                    else
                      file_write(real_fd, part_off, io_buf.data() + done, n);
                  } else if (!is_read) {
                    printf("[sdmmc] eMMC write past the last rawnand part "
                           "(0x%llX) dropped\n", (unsigned long long)off);
                    io_err = true;
                  }
                  done += n;
                }
              }
            } else if (current_fd >= 0) {
              if (is_read)
                file_read(current_fd, current_off, io_buf.data(), len);
              else
                file_write(current_fd, current_off, io_buf.data(), len);
            }
            // No image behind this partition (BOOT1, a BOOT0 not given):
            // it reads as blank and swallows writes.
            if (is_read && uc_mem_write(uc, dma_addr, io_buf.data(), len) != UC_ERR_OK)
              io_err = true;
          };

          if (trnmod & 0x0001) {            // DMA Enabled (Bit 0 of TRNMOD)
            uint32_t dmasel = hostctl & 0x18;
            if (dmasel == 0x10 || dmasel == 0x18) {
              // ADMA2 (SD Host Controller spec 1.13): each descriptor is
              // attr (bit 0 Valid, bit 1 End, 5:4 Act: 10b transfer, 11b
              // link), a 16-bit length (0 = 64 KiB) and the address - 32
              // bits in an 8-byte descriptor, 64 in a 12-byte one. The walk
              // is bounded: a table with no End bit must not run through all
              // of DRAM.
              bool d64 = dmasel == 0x18;
              uint64_t desc_addr = adma_addr;
              for (int n = 0; n < 4096; n++) {
                uint8_t desc[12] = {0};
                if (uc_mem_read(uc, desc_addr, desc, d64 ? 12 : 8) != UC_ERR_OK) {
                  errintsts |= 1u << 9;   // ADMA error
                  break;
                }
                uint16_t attr = (uint16_t)(desc[0] | (desc[1] << 8));
                uint32_t dlen = (uint32_t)(desc[2] | (desc[3] << 8));
                if (!dlen)
                  dlen = 65536;
                uint64_t addr = (uint64_t)desc[4] | ((uint64_t)desc[5] << 8) |
                                ((uint64_t)desc[6] << 16) | ((uint64_t)desc[7] << 24);
                if (d64)
                  addr |= ((uint64_t)desc[8] | ((uint64_t)desc[9] << 8) |
                           ((uint64_t)desc[10] << 16) | ((uint64_t)desc[11] << 24)) << 32;
                if (!(attr & 0x01)) {     // not Valid: the engine stops, in error
                  errintsts |= 1u << 9;
                  break;
                }
                uint32_t act = (attr >> 4) & 3;
                if (act == 2) {           // transfer
                  do_io(fd, file_off, addr, dlen);
                  file_off += dlen;
                } else if (act == 3) {    // link to the next table
                  desc_addr = addr;
                  continue;
                }
                if (attr & 0x02)          // End
                  break;
                desc_addr += d64 ? 12 : 8;
              }
            } else { // SDMA
              do_io(fd, file_off, adma_addr, xfer_len);
            }
          } else { // PIO
            do_io(fd, file_off, sysad, xfer_len);
          }
          if (io_err)
            errintsts |= 1u << 4;         // data timeout
        }
        // R1 for the read/write command itself: TRAN, READY_FOR_DATA, no
        // error bits. bdk checks this cached response after every transfer;
        // it used to hold whatever the previous command had answered.
        rsp[0] = r1_base | (4 << 9) | (1u << 8);
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

// APBDEV_RTC (TRM 11.x). SECONDS counts once per second and is writable;
// reading MILLI_SECONDS snapshots SECONDS into SHADOW_SECONDS, which is how
// bdk's get_tmr_ms() reads both halves of one instant. The RTC is always-on:
// it keeps counting across a SoC reset (rtc_base_us carries the time over).
static uint64_t rtc_base_us = 0;       // RTC time at emu_usec == 0
static int64_t  rtc_sec_adjust = 0;    // SECONDS writes, as an offset
static uint32_t rtc_shadow_seconds = 0;

static uint32_t rtc_seconds(EmuState *state) {
  return (uint32_t)((int64_t)((rtc_base_us + state->emu_usec) / 1000000) +
                    rtc_sec_adjust);
}

uint32_t rtc_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - RTC_BASE);
  switch (offset) {
  case 0x08:
    return rtc_seconds(state); // APBDEV_RTC_SECONDS
  case 0x0C:
    return rtc_shadow_seconds; // APBDEV_RTC_SHADOW_SECONDS
  case 0x10:                   // APBDEV_RTC_MILLI_SECONDS
    rtc_shadow_seconds = rtc_seconds(state);
    return (uint32_t)(((rtc_base_us + state->emu_usec) / 1000) % 1000);
  default:
    return mmio_regs.get(addr);
  }
}

void rtc_write(EmuState *state, uint64_t addr, uint32_t val) {
  if ((uint32_t)(addr - RTC_BASE) == 0x08)
    rtc_sec_adjust += (int64_t)val - (int64_t)rtc_seconds(state);
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
// Bit 3 (PCIE) is already ungated at RCM entry - measured on both an Erista
// and a Mariko (hwtest's Wi-Fi probe reads PWRGATE_STATUS before touching it).
static uint32_t pmc_pwrgate_status = 1u << 3;
static uint32_t pmc_io_dpd_status[2] = {0, 0};
// Everything else in the PMC is plain read/write storage: the scratch
// registers payloads leave notes in, the secure scratch the SE deposits its
// context key in, pad and timing configuration.
static uint32_t pmc_regs[PMC_SIZE / 4];

bool pmc_partition_on(int part) {
  return part >= 0 && part < 32 && ((pmc_pwrgate_status >> part) & 1);
}

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
  case 0x2BC:
    // APBDEV_PMC_GLB_AMAP_CFG: which CCPLEX apertures decode as MMIO or DRAM
    // (TRM 12.6.172). Measured 0x00020000 in RCM; bits 1..3 clear means the
    // three PCIe apertures are MMIO for the CPU, which CPU0's accesses to
    // the root complex depend on.
    return 0x00020000;
  default:
    return offset < PMC_SIZE ? pmc_regs[offset / 4] : 0;
  }
}

void pmc_secure_scratch_write(unsigned n, uint32_t value) {
  static const uint16_t kOffset[8] = {0xB0, 0xB4, 0xB8, 0xBC,
                                      0xC0, 0xC4, 0x224, 0x228};
  if (n < 8)
    pmc_regs[kOffset[n] / 4] = value;
}

// ==================== PMC ====================

void pmc_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - PMC_BASE);
  if (offset < PMC_SIZE)
    pmc_regs[offset / 4] = val;
  switch (offset) {
  case 0x00:
    // APBDEV_PMC_CNTRL — bit 4 = MAIN_RST. Hekate writes this from
    // power_set_state(REBOOT_RCM); we mirror it as a soft reboot of the
    // emulated payload so the user can iterate without restarting rcm_emu.
    if (val & (1u << 4)) {
      printf("[emu] PMC MAIN_RST written - rebooting payload\n");
      fflush(stdout);
      request_reboot(state, false);
    }
    break;
  case 0x30: {
    // APBDEV_PMC_PWRGATE_TOGGLE: bits 4:0 select a partition, bit 8 starts
    // the toggle. The transition is instantaneous here, so START reads back
    // clear and the caller's poll on PWRGATE_STATUS succeeds immediately.
    if (val & (1u << 8)) {
      uint32_t part = val & 0x1F;
      pmc_pwrgate_status ^= (1u << part);
      bool on = (pmc_pwrgate_status >> part) & 1;
      if (part == 3) // POWER_RAIL_PCIE
        pcie_set_powergate(on);
      // CRAIL / CE0 / C0NC carry CPU0: gating one under a running core
      // takes the core down with it.
      ccplex_partition_changed(state, (int)part, on);
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

// FLOW_CTLR_RAM_REPAIR (0x40, TRM 17.2.9, reset 0x4 = BYPASS_EN). Software
// sets REQ (bit 0); hardware repairs every segment, sets STS (bit 1, read-
// only) and clears REQ again. ccplex_boot_cpu0() sets REQ and spins on STS
// with no timeout (bdk soc/ccplex.c), so the repair completes at once.
static constexpr uint32_t RAM_REPAIR_REQ = 1u << 0;
static constexpr uint32_t RAM_REPAIR_STS = 1u << 1;
static constexpr uint32_t RAM_REPAIR_RESET = 1u << 2;   // BYPASS_EN
static uint32_t flow_ram_repair = RAM_REPAIR_RESET;

// ---- 1 MHz timers TMR0..TMR9 ----------------------------------------------
//
// Only what a halted BPMP needs: bdk's timer_usleep() arms TMR8 and stops
// the core with FLOW_MODE_STOP_UNTIL_IRQ until it fires. PTV (TRM 8.7.1):
// bit 31 EN, bit 30 PER (periodic), 28:0 the count, in an n+1 scheme.
static const uint16_t kTmrPtv[10] = {0x88, 0x00, 0x08, 0x50, 0x58,
                                     0x60, 0x68, 0x70, 0x78, 0x80}; // TMR0..9
static uint64_t tmr_armed_us[10];
static constexpr uint32_t TMR_EN = 1u << 31, TMR_PER = 1u << 30;

static void tmr_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t off = (uint32_t)(addr - TMR_BASE);
  for (int t = 0; t < 10; t++)
    if (off == kTmrPtv[t] && (val & TMR_EN))
      tmr_armed_us[t] = state->emu_usec;
}

// The first armed timer to fire: its expiry time. One-shot timers disarm,
// periodic ones re-arm from their expiry.
static bool tmr_fire_next(EmuState *state, uint64_t *when) {
  int best = -1;
  uint64_t best_at = 0;
  for (int t = 0; t < 10; t++) {
    uint32_t ptv = mmio_regs.get(TMR_BASE + kTmrPtv[t]);
    if (!(ptv & TMR_EN))
      continue;
    uint64_t at = tmr_armed_us[t] + (ptv & 0x1FFFFFFF) + 1;
    if (best < 0 || at < best_at) {
      best = t;
      best_at = at;
    }
  }
  if (best < 0)
    return false;
  uint64_t addr = TMR_BASE + kTmrPtv[best];
  uint32_t ptv = mmio_regs.get(addr);
  if (ptv & TMR_PER)
    tmr_armed_us[best] = best_at;
  else
    mmio_regs[addr] = ptv & ~TMR_EN;
  *when = best_at > state->emu_usec ? best_at : state->emu_usec;
  return true;
}

static uint32_t flow_read(EmuState *state, uint64_t addr) {
  (void)state;
  uint32_t offset = (uint32_t)(addr - 0x60007000);
  if (offset == 0x40)
    return flow_ram_repair;
  return mmio_regs.get(addr);
}

static void flow_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - 0x60007000);
  if (offset == 0x40) {
    flow_ram_repair = (val & ~(RAM_REPAIR_REQ | RAM_REPAIR_STS)) |
                      (flow_ram_repair & RAM_REPAIR_STS);
    if (val & RAM_REPAIR_REQ)
      flow_ram_repair |= RAM_REPAIR_STS;
    return;
  }
  if (offset == 0x04) {
    // HALT_COP_EVENTS: MODE in 31:29 (TRM 17.2.2).
    //   0 NONE, 1 RUN_AND_INT       - no halt at all
    //   2/3 WAITEVENT(_AND_INT)     - stop until an event source fires
    //   4/5 STOP_UNTIL_IRQ(_AND_INT), 6 STOP_UNTIL_EVENT_AND_IRQ
    // Three different things arrive here:
    //
    //  1. bpmp_usleep() / bpmp_msleep() park the BPMP with a TIMER event
    //     source (HALT_USEC / HALT_MSEC / HALT_SEC) plus a delay count, to
    //     sleep with the core clock-gated. The timer event wakes the core
    //     and execution continues.
    //  2. timer_usleep() arms TMR8 and waits in STOP_UNTIL_IRQ for its
    //     interrupt, the only interrupt source modelled here.
    //  3. bpmp_halt() writes WAITEVENT | JTAG with no timer source and is
    //     followed by `while(true);`. That one really is "payload done"
    //     (power-off / reboot / fatal) - unless CPU0 is still running, as
    //     after fusee's or hekate's L4T handoff, which then carries on
    //     alone.
    uint32_t mode = val >> 29;
    if (mode <= 1)
      return;
    if (state->reboot_requested) {
      // power_set_state() halts right after writing MAIN_RST or SFT_RST:
      // the reset is on its way, so this is not the end of the run.
      if (g_bus_master == BUS_BPMP)
        uc_emu_stop(state->uc);
      return;
    }
    uint64_t wake = 0;
    if (val & HALT_TIMED) {
      // Advance the emulated microsecond counter by the requested delay so
      // TIMERUS-based delta loops observe the time actually passing, then
      // let the CPU run on.
      uint32_t delay = val & 0xFF;
      wake = state->emu_usec +
             ((val & HALT_USEC)   ? (uint64_t)delay
              : (val & HALT_MSEC) ? (uint64_t)delay * 1000ULL
                                  : (uint64_t)delay * 1000000ULL);
    } else if (mode >= 4 && !tmr_fire_next(state, &wake)) {
      wake = 0;
    }
    if (wake) {
      state->bpmp_slept_us += wake - state->emu_usec; // ACTMON BPMP load
      state->emu_usec = wake;
      bpmp_clock_jumped();
      return;
    }
    if (ccplex_cpu0_running()) {
      printf("[flow] BPMP halted for good (val=0x%08X); CPU0 runs on\n", val);
      fflush(stdout);
      state->bpmp_halted = true;
      bpmp_yield();
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

// ---- Reset and clock-enable banks (TRM 5.2) ---------------------------------
//
// RST_DEVICES_x and CLK_OUT_ENB_x for banks L, H, U, V, W, X and Y. Each is
// written directly or through its SET and CLR strobes, and all three read
// back the bank: "for reads, you can use either method to retrieve the
// reset/clock-enable state" (TRM 5.2.100). bdk relies on both halves -
// clock_enable() goes through the strobes, clock_sdmmc_is_active() reads
// RST_DEV_L_SET and CLK_ENB_L_SET, and display_init() tears the panel down
// first when CLK_OUT_ENB_L already has DISP1 on.
//
// The PCIe and CPU models keep reset banks V, W and Y and clock bank V,
// because their preconditions depend on them; the table only routes reads
// of those strobes to them. The rest start from the TRM reset values
// (unknown bits as 0), except bank U, which is what a real console read.
// The L, H and X clock banks used to be real-console readings too, but
// taken after a payload had brought the display up - so every payload
// found DISP1 already clocked at RCM entry and began with a panel teardown.
struct CarBank {
  uint16_t reg, set, clr;
  bool own;       // false: another model holds this bank
  uint32_t seed;
};
static const CarBank kCarBanks[] = {
    {0x004, 0x300, 0x304, true,  0x1CD3D2C8}, // RST_DEVICES_L
    {0x008, 0x308, 0x30C, true,  0x87D1F326}, // RST_DEVICES_H
    {0x00C, 0x310, 0x314, true,  0x828EC5F8}, // RST_DEVICES_U (measured)
    {0x358, 0x430, 0x434, false, 0},          // RST_DEVICES_V (pcie.cpp)
    {0x35C, 0x438, 0x43C, false, 0},          // RST_DEVICES_W (pcie.cpp)
    {0x28C, 0x290, 0x294, true,  0x01E42049}, // RST_DEVICES_X
    {0x2A4, 0x2A8, 0x2AC, false, 0},          // RST_DEVICES_Y (pcie.cpp)
    {0x010, 0x320, 0x324, true,  0x80000130}, // CLK_OUT_ENB_L
    {0x014, 0x328, 0x32C, true,  0x00000080}, // CLK_OUT_ENB_H
    {0x018, 0x330, 0x334, true,  0x01F00200}, // CLK_OUT_ENB_U (measured)
    {0x360, 0x440, 0x444, false, 0},          // CLK_OUT_ENB_V (ccplex.cpp)
    {0x364, 0x448, 0x44C, true,  0x402000FC}, // CLK_OUT_ENB_W
    {0x280, 0x284, 0x288, true,  0x23000780}, // CLK_OUT_ENB_X
    {0x298, 0x29C, 0x2A0, true,  0x00000300}, // CLK_OUT_ENB_Y
};
constexpr size_t kCarBankCount = sizeof(kCarBanks) / sizeof(kCarBanks[0]);

struct CarBanks {
  uint32_t v[kCarBankCount];
  CarBanks() {
    for (size_t i = 0; i < kCarBankCount; ++i)
      v[i] = kCarBanks[i].seed;
  }
};
static CarBanks car_banks;

static const CarBank *car_bank_of(uint32_t off) {
  for (const CarBank &b : kCarBanks)
    if (off == b.reg || off == b.set || off == b.clr)
      return &b;
  return nullptr;
}

// The EMC clock as CAR sets it (EMC section below).
static uint32_t emc_rate_khz();

uint32_t clk_rst_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr - CLK_RST_BASE);
  if (const CarBank *b = car_bank_of(offset)) {
    if (b->own)
      return car_banks.v[b - kCarBanks];
    if (offset != b->reg) // a strobe of a bank another model keeps
      return clk_rst_read(state, CLK_RST_BASE + b->reg);
  }
  // PLL_BASE registers (per Hekate bdk/soc/clock.h): each PLL has an _BASE
  // register where bit 30 = ENABLE and bit 27 = LOCK. After enabling a PLL
  // the boot code polls bit 27 until set. Real silicon locks within ~1ms;
  // here a PLL is locked as soon as it is enabled.
  // Offsets: PLLC=0x80, PLLM=0x90, PLLP=0xA0, PLLA=0xB0, PLLU=0xC0, PLLD=0xD0,
  //          PLLX=0xE0, PLLE=0xE8, PLLD2=0x4B8, PLLREFE=0x4C4, PLLC2=0x4E8,
  //          PLLC3=0x4FC, PLLDP=0x590, PLLC4=0x5A4, PLLMB=0x5E8, PLLA1=0x6A4.
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
  // PLLE_BASE (0xE8) and PLLREFE_BASE (0x4C4) are not in this list: bit 30
  // is PLLE's LOCK_OVERRIDE, bit 27 sits inside a divider field in both
  // (PLDIV_CML / KCP, TRM 5.2), and they report lock in their _MISC
  // registers - pcie_car_read() answers for those. Forcing bit 27 on here
  // handed back, and invited a read-modify-write to store, a divider the
  // payload never programmed.
  if (offset == 0xE8 || offset == 0x4C4)
    return mmio_regs.get(addr);
  switch (offset) {
  case 0x80: case 0xB0: case 0xE0: case 0x4B8: case 0x4E8: case 0x4FC:
  case 0x590: case 0x5A4: case 0x6A4: {
    // PLLC, PLLA, PLLX, PLLD2 (measured down on both), PLLC2, PLLC3,
    // PLLDP, PLLC4 and PLLA1: locked once the payload enables them (bit
    // 30). Bit 27 is LOCK or FREQ_LOCK on all of them; PLLA1 also has LOCK
    // in 26, and PLLDP and PLLC4 FREQLOCK in 28 and a LOCK_OVERRIDE in 24
    // (TRM 5.2). PLLC4 does not run in IDDQ (bit 18): bdk clears that,
    // enables it for the eMMC's HS200/HS400 clock and waits for the lock
    // with no timeout.
    uint32_t lock = 1u << 27;
    if (offset == 0x6A4)
      lock |= 1u << 26;
    if (offset == 0x590 || offset == 0x5A4)
      lock |= 1u << 28;
    uint32_t w = mmio_regs.get(addr) & ~lock;
    bool on = (w & (1u << 30)) && !(offset == 0x5A4 && (w & (1u << 18)));
    if ((offset == 0x590 || offset == 0x5A4) && (w & (1u << 24)))
      on = true;
    return on ? w | lock : w;
  }
  case 0x90: { // PLLM
    // Until the payload programs it: Erista leaves PLLM enabled but
    // unlocked at this point, Mariko has it off. Once written, it reads
    // back what was written - the dividers too, before ENABLE is set.
    if (!mmio_regs.count(addr))
      return mariko ? 0u : (1u << 30);
    uint32_t w = mmio_regs.get(addr);
    return (w & (1u << 30)) ? w | (1u << 27) : w;
  }
  case 0xC0: { // PLLU
    if (!mmio_regs.count(addr))
      return mariko ? 0u : ((1u << 30) | (1u << 27));
    uint32_t w = mmio_regs.get(addr);
    return (w & (1u << 30)) ? w | (1u << 27) : w;
  }
  case 0xA0: // PLLP_BASE - same value on both
    return 0x48115408u;
  case 0xD0: // PLLD_BASE - up on both
    return (1u << 30) | (1u << 27);
  case 0x5E8: { // PLLMB_BASE (TRM 5.2.230), reset 0x00002A02: down
    // Minerva moves the EMC onto PLLMB for a DRAM frequency change and
    // spins on PLLMB_LOCK after enabling it. LOCK and FREQ_LOCK (bits 27,
    // 26) are read-only: set once the PLL is enabled, or forced by
    // PLLMB_MISC1.LOCK_OVERRIDE.
    uint32_t w = mmio_regs.get(addr, 0x00002A02) & ~(3u << 26);
    uint32_t misc1 = mmio_regs.get(CLK_RST_BASE + 0x5EC, 0x00010000);
    if ((w & (1u << 30)) || (misc1 & (1u << 18)))
      w |= 3u << 26;
    return w;
  }
  case 0x52C: { // UTMIPLL_HW_PWRDN_CFG0 (TRM 5.2.202), reset 0x0000000F
    // UTMIPLL_LOCK (bit 31) and SEQ_STATE (27:26) are read-only. The USB
    // PLL locks unless software holds it in IDDQ (IDDQ_SWCTL with
    // IDDQ_OVERRIDE_VALUE, the reset state); bdk's usb code clears the
    // override, then clock_enable_utmipll() waits for the lock.
    uint32_t w = mmio_regs.get(addr, 0x0000000F) & ~(1u << 31 | 3u << 26);
    return (w & 3) == 3 ? w : w | (1u << 31);
  }
  case 0x5EC: // PLLMB_MISC1, reset 0x00010000 (EN_LCKDET)
    return mmio_regs.get(addr, 0x00010000);
  case 0x19C: // CLK_SOURCE_EMC, reset 0x60180000 (TRM 5.2.73)
    return mmio_regs.get(addr, 0x60180000);
  case 0x664: // CLK_SOURCE_EMC_DLL, reset 0x60000000
  case 0x724: // CLK_SOURCE_EMC_SAFE, reset 0x60000000
    return mmio_regs.get(addr, 0x60000000);
  }

  // Informational clock registers, measured on a real Mariko. These read back
  // as 0 otherwise, which makes the whole clock page look dead.
  {
    switch (offset) {
    case 0xA4:  return 0x00000003; // PLLP_OUTA
    case 0x68:  return 0x00005C00; // PLLE_SS_CNTL (TRM reset value)
    // SCLK_BURST differs slightly by generation (measured).
    case 0x28:  return mariko ? 0x20003333u : 0x20003330u;
    case 0x2C:  return 0x80000000; // SUPER_SCLK_DIVIDER
    case 0x30:  return 0x00000002; // CLK_SYSTEM_RATE
    // CLK_SOURCE_UARTD. clock_uart_use_src_div() programs PLLP_OUT0 with
    // CLK_SRC_DIV(2) here (0x00000002) and sets UART_SRC_CLK_DIV_EN for the
    // 1M/3M rates; the payload reads it back to confirm the port's source.
    case 0x1C0: return mmio_regs.get(addr);
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
    uint32_t cntl = mmio_regs.get(CLK_RST_BASE + 0x60);
    uint32_t src = (cntl >> 14) & 0x1FF;   // PTO_SRC_SEL
    uint32_t khz = 0;
    switch (src) {
    case 0x1C: khz = mariko ? 407971 : 407980; break; // SCLK / BPMP
    case 0x24: // EMC (DRAM): the modelled clock; measured at the boot clock
      khz = emc_rate_khz();
      if (khz == 204000)
        khz = mariko ? 203991 : 204001;
      break;
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

  // OSC_CTRL: 31:28 OSC_FREQ, 5 = 38.4 MHz, the Switch's crystal (TRM
  // 5.2.x, reset 0x500003f1; bdk hw_init() writes 0x50000071). This used to
  // answer 4, which decodes as 19.2 MHz and contradicted OSC_FREQ_DET.
  if (offset == 0x50)
    return mmio_regs.get(addr, 0x500003F1);

  // CPU complex clock/reset state (CLK_OUT_ENB_V, RST_CPUG_CMPLX, ...).
  {
    uint32_t v = 0;
    if (ccplex_car_read(offset, &v))
      return v;
  }

  // PLLE and PLLREFE report lock in their _MISC registers, not in _BASE bit
  // 27 like the PLLs handled above, so the PCIe model answers for them.
  {
    uint32_t v = mmio_regs.get(addr);
    if (pcie_car_read(offset, &v))
      return v;
  }
  // Everything else is a plain R/W register (the CLK_SOURCE_* dividers,
  // PLL MISC words, ...): it reads back what was written.
  return mmio_regs.get(addr);
}

// A CLK_SOURCE_EMC write is the CAR/EMC clock-change handshake (EMC below).
static void emc_clock_source_write(EmuState *state, uint32_t val);

void clk_rst_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  uint32_t offset = (uint32_t)(addr - CLK_RST_BASE);
  // The reset and clock-enable banks, and CLK_SOURCE_EMC, which starts an
  // EMC clock change. Everything else only lands in the write hook's
  // mmio_regs cache, which clk_rst_read hands back.
  if (const CarBank *b = car_bank_of(offset); b && b->own) {
    uint32_t &bank = car_banks.v[b - kCarBanks];
    if (offset == b->reg)      bank = val;
    else if (offset == b->set) bank |= val;
    else                       bank &= ~val;
  }
  if (offset == 0x19C)
    emc_clock_source_write(state, val);
  // The PCIe root complex depends on PLLE, PLLREFE and the PCIE/AFI/
  // PCIEXCLK/UPHY/padctl reset+enable bits, so it shadows the same writes.
  pcie_car_write(state, offset, val);
  // The CPU complex: CPU reset/clock enables, PLLX and CCLK.
  ccplex_car_write(state, offset, val);
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
  uint32_t v = state->fuse_at(offset).load();
  // FUSE_RESERVED_ODM28_B01 bit 0 tells bdk which MAX77812 is fitted: clear
  // means the PHASE31 part at I2C_5 0x31, set the PHASE211 retail part at
  // 0x33 (bdk power/max7762x.c _max77812_get_address). This model only
  // answers at 0x33, so a Mariko must read the bit set; left clear, every
  // bdk regulator call went to a NAKing 0x31 and the CPU rail could never
  // be enabled - which is exactly what booting CPU0 needs.
  if (offset == 0x240 && state->is_mariko.load())
    v |= 1u;
  return v;
}

void fuse_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  uint32_t offset = (uint32_t)(addr - FUSE_BASE);
  if (offset == 0x04) g_fuse_ctrl_addr = val; // FUSE_ADDR
}

// ==================== EMC ====================
//
// A register file - reads return what was written - with the controller's
// own behaviour on top (TRM 18.11):
//
//   Banks. EMC_BASE is the broadcast bank: a write there reaches both
//   channels. EMC0_BASE / EMC1_BASE address one channel each. mmio_regs
//   holds every bank's words (the write hook stores the raw address), and a
//   broadcast write is mirrored into both channel banks.
//
//   Mode registers. EMC_MRW and EMC_MRW2..15 send an MRW to the DRAM.
//   EMC_MRR sends an MRR, after which the channel's EMC_MRR holds the answer
//   (both x16 dies' bytes, bits 15:0) and its EMC_EMC_STATUS.MRR_DIVLD stays
//   set until that data is read. MR4 reports the normal-temperature refresh
//   rate, MR5-8 the configured DRAM ID, MR18/MR19 the DQS interval
//   oscillator count, and any other mode register what was last written.
//
//   Digital DLL. It locks as soon as it is (re)started: EMC_DIG_DLL_STATUS
//   reports DLL_LOCK and DLL_PRIV_UPDATED. The DLL code itself (DLL_OUT) is
//   not modelled and reads 0.
//
//   Clock change. A new source or divisor in CAR CLK_SOURCE_EMC starts the
//   CAR/EMC clock-change handshake: the EMC replays the writes queued in its
//   clock-change FIFO (EMC_CCFIFO_DATA, then EMC_CCFIFO_ADDR) and raises
//   CLKCHANGE_COMPLETE in EMC_INTSTATUS (sticky, write 1 to clear).
//
//   EMC_EMC_STATUS otherwise reports timing updates done, the DRAM active
//   (not powered down, not in self-refresh) and the state machines idle.
//
// bdk's sdram_read_mrx needs the MRR path. Minerva, hekate's DRAM training
// and frequency-switching module, needs the rest: it spins on DLL lock and
// on the DLL-enable bit it writes, waits for the clock-change handshake,
// and divides by the oscillator count.

static constexpr uint32_t EMC_INTSTATUS      = 0x000;
static constexpr uint32_t EMC_MRW            = 0x0E8;
static constexpr uint32_t EMC_MRR            = 0x0EC;
static constexpr uint32_t EMC_EMC_STATUS     = 0x2B4;
static constexpr uint32_t EMC_CFG_DIG_DLL    = 0x2BC;
static constexpr uint32_t EMC_DIG_DLL_STATUS = 0x2C4;
static constexpr uint32_t EMC_CCFIFO_ADDR    = 0x3E8;
static constexpr uint32_t EMC_CCFIFO_DATA    = 0x3EC;
static constexpr uint32_t EMC_CCFIFO_STATUS  = 0x3F0;
static constexpr uint32_t EMC_FBIO_CFG7      = 0x584;

static constexpr uint32_t EMC_INT_CLKCHANGE_COMPLETE = 1u << 4;
static constexpr uint32_t EMC_INT_MRR_DIVLD          = 1u << 5;
static constexpr uint32_t EMC_INT_CCFIFO_OVERFLOW    = 1u << 8;
static constexpr uint32_t EMC_STATUS_MRR_DIVLD       = 1u << 20;
// The TRM calls the clock-change FIFO 32 deep, but CCFIFO_COUNT is 7 bits
// and NVIDIA's own DVFS sequence (which Minerva ports) queues up to 65
// entries per change, so the model takes as many as the count can report.
static constexpr size_t   kEmcCcfifoDepth            = 127;

// CAR CLK_SOURCE_EMC (TRM 5.2.73). A clock change starts when
// EMC_2X_CLK_SRC, MC_EMC_SAME_FREQ or EMC_2X_CLK_DIVISOR changes, or on
// FORCE_CC_TRIGGER; rewriting the same value does nothing.
static constexpr uint32_t CAR_CLK_SOURCE_EMC       = 0x19C;
static constexpr uint32_t CAR_CLK_SOURCE_EMC_RESET = 0x60180000; // CLK_M
static constexpr uint32_t CAR_EMC_CC_FIELDS        = 0xE00100FF;
static constexpr uint32_t CAR_EMC_FORCE_CC_TRIGGER = 1u << 27;

// The LPDDR4's DQS-to-DQ delay, what the DQS interval oscillator measures.
// JEDEC allows 200-800 ps; this is mid-range.
static constexpr uint32_t kDramTdqs2dqPs = 400;

struct EmcState {
  uint8_t  mr[256] = {};          // mode registers, as last written by MRW
  uint32_t mrr_ma[2] = {5, 5};    // per channel: MA of the last MRR
  bool     mrr_valid[2] = {};     // per channel: MRR_DIVLD
  uint32_t intstatus = 0;
  uint32_t ccfifo_data = 0;
  std::vector<std::pair<uint32_t, uint32_t>> ccfifo; // (offset, data)
  uint32_t clk_source = CAR_CLK_SOURCE_EMC_RESET;    // CAR, as last applied
};
static EmcState emc;

static bool emc_is_mrw(uint32_t offset) {
  switch (offset) {
  case 0x0E8: case 0x134: case 0x138: case 0x13C: // MRW, MRW2-4
  case 0x4A0: case 0x4A4: case 0x4A8: case 0x4AC: // MRW5-8
  case 0x4B0: case 0x4B4: case 0x4B8: case 0x4BC: // MRW9-12
  case 0x4C0: case 0x4C4: case 0x4D0:             // MRW13-15
    return true;
  default:
    return false;
  }
}

// The EMC clock, from CAR. EMC_2X_CLK_SRC picks the input; the _UD inputs
// bypass the 7.1 EMC_2X_CLK_DIVISOR. The PLLs run at
// 38.4 MHz / DIVM * DIVN / (DIVP + 1), as Minerva's own table computes.
static uint32_t emc_rate_khz() {
  auto pll = [](uint32_t off) -> uint32_t {
    uint32_t b = mmio_regs.get(CLK_RST_BASE + off);
    uint32_t m = b & 0xFF, n = (b >> 8) & 0xFF, p = (b >> 20) & 0x1F;
    return m ? (uint32_t)(38400ull * n / m / (p + 1)) : 0;
  };
  uint32_t src = emc.clk_source, in = 0;
  bool ud = false;
  switch (src >> 29) {
  case 0: in = pll(0x090); break;             // PLLM_OUT0
  case 1: in = pll(0x080); break;             // PLLC_OUT0
  case 2: in = 408000; break;                 // PLLP_OUT0
  case 3: in = 38400; break;                  // CLK_M
  case 4: in = pll(0x090); ud = true; break;  // PLLM_UD
  case 5: in = pll(0x5E8); ud = true; break;  // PLLMB_UD
  case 6: in = pll(0x5E8); break;             // PLLMB_OUT0
  case 7: in = 408000; ud = true; break;      // PLLP_UD
  }
  return ud ? in : (uint32_t)(in * 2ull / ((src & 0xFF) + 2));
}

// MR18/MR19: the DQS interval oscillator count, run time / (2 x tDQS2DQ)
// (JEDEC LPDDR4). The run time is MR23's setting in clocks (16 x OP up to
// 63, then 2048, 4096 or 8192) at the current DRAM clock. MR23 = 0 means
// "stopped by MPC"; that is counted as the 2048-clock setting.
static uint32_t emc_dqs_osc_count() {
  uint32_t op = emc.mr[23];
  uint32_t clocks = op == 0 ? 2048 : op < 64 ? 16 * op
                  : op < 128 ? 2048 : op < 192 ? 4096 : 8192;
  uint32_t khz = emc_rate_khz();
  if (!khz)
    khz = 204000;
  uint64_t tck_ps = 1000000000ull / khz;
  uint64_t count = clocks * tck_ps / (2 * kDramTdqs2dqPs);
  return (uint32_t)std::min<uint64_t>(std::max<uint64_t>(count, 1), 0xFFFF);
}

static uint32_t emc_mr_value(EmuState *state, uint32_t ma) {
  uint32_t v;
  switch (ma) {
  case 4:  v = 0x03; break;                        // refresh 1x: normal temp
  case 5:  v = state->dram_vendor.load(); break;
  case 6:  v = state->dram_rev_id1.load(); break;
  case 7:  v = state->dram_rev_id2.load(); break;
  case 8:  v = state->dram_density.load(); break;
  case 18: v = emc_dqs_osc_count() & 0xFF; break;
  case 19: v = emc_dqs_osc_count() >> 8; break;
  default: v = emc.mr[ma & 0xFF]; break;
  }
  return (v & 0xFF) << 8 | (v & 0xFF);            // both dies answer
}

// Which bank an address is in: -1 broadcast, 0 / 1 a channel.
static int emc_bank(uint64_t addr) {
  return addr >= EMC1_BASE ? 1 : addr >= EMC0_BASE ? 0 : -1;
}

uint32_t emc_read(EmuState *state, uint64_t addr) {
  uint32_t offset = (uint32_t)(addr & 0xFFF);
  int ch = std::max(emc_bank(addr), 0);   // a broadcast read sees channel 0
  switch (offset) {
  case EMC_INTSTATUS:
    return emc.intstatus;
  case EMC_MRR:
    emc.mrr_valid[ch] = false;
    return emc_mr_value(state, emc.mrr_ma[ch]);
  case EMC_EMC_STATUS: {
    // ACPD/DSR FSMs idle (27:24), ZQ FSM idle (22), no outstanding
    // transactions (2), request FIFO empty (0).
    uint32_t v = (0xFu << 24) | (1u << 22) | (1u << 2) | (1u << 0);
    return emc.mrr_valid[ch] ? v | EMC_STATUS_MRR_DIVLD : v;
  }
  case EMC_CFG_DIG_DLL:
    // Bits 31, 30, 26, 4 and 1 are write-1 triggers; they read 0.
    return mmio_regs.get(addr) & ~0xC4000012u;
  case EMC_DIG_DLL_STATUS:
    return (1u << 17) | (1u << 15);        // DLL_PRIV_UPDATED | DLL_LOCK
  case EMC_CCFIFO_STATUS:
    return (uint32_t)emc.ccfifo.size();
  case EMC_FBIO_CFG7:
    return mmio_regs.get(addr, (1u << 1) | (1u << 2)); // both channels
  default:
    return mmio_regs.get(addr);
  }
}

void emc_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  uint32_t offset = (uint32_t)(addr & 0xFFF);
  int bank = emc_bank(addr);
  switch (offset) {
  case EMC_INTSTATUS:
    emc.intstatus &= ~val;                 // write 1 to clear
    return;
  case EMC_MRR:
    for (int c = 0; c < 2; c++) {
      if (bank < 0 || bank == c) {
        emc.mrr_ma[c] = (val >> 16) & 0xFF;
        emc.mrr_valid[c] = true;
      }
    }
    emc.intstatus |= EMC_INT_MRR_DIVLD;
    break;
  case EMC_CCFIFO_DATA:
    emc.ccfifo_data = val;
    break;
  case EMC_CCFIFO_ADDR:
    if (emc.ccfifo.size() < kEmcCcfifoDepth)
      emc.ccfifo.emplace_back(val & 0xFFFF, emc.ccfifo_data);
    else
      emc.intstatus |= EMC_INT_CCFIFO_OVERFLOW;
    break;
  default:
    if (emc_is_mrw(offset))
      emc.mr[(val >> 16) & 0xFF] = (uint8_t)val;
    break;
  }
  mmio_regs[addr] = val;
  if (bank < 0) {
    mmio_regs[EMC0_BASE + offset] = val;
    mmio_regs[EMC1_BASE + offset] = val;
  }
}

// CAR CLK_SOURCE_EMC was written (clk_rst_write).
static void emc_clock_source_write(EmuState *state, uint32_t val) {
  bool change = ((val ^ emc.clk_source) & CAR_EMC_CC_FIELDS) ||
                (val & CAR_EMC_FORCE_CC_TRIGGER);
  emc.clk_source = val;
  if (!change)
    return;
  std::vector<std::pair<uint32_t, uint32_t>> fifo;
  fifo.swap(emc.ccfifo);
  for (const auto &e : fifo)
    emc_write(state, EMC_BASE + (e.first & 0xFFF), e.second);
  emc.intstatus |= EMC_INT_CLKCHANGE_COMPLETE;
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

// ==================== VIC ====================
//
// bdk (vic.c) drives VIC through the Falcon private-register window:
// VIC(0x1000 + (priv >> 6)) = data. PVIC_FALCON_ADDR supplies the low bits
// for registers that need them; none of the ones modelled here do. The model
// keeps what a compose needs - the config struct's address (PRAMBASE), the
// slots' source surfaces and the target surface - and, on COMPOSE_START,
// reads the config struct from guest memory and draws the enabled slots
// into the target with the output flip/transpose. Surfaces are pitch or
// block linear (BlkKind 1, BlkHeight = log2 GOBs per block). That is what Nyx
// relies on: its 1280x720 GUI is turned into the panel's 720x1280 portrait
// framebuffer by a 270-degree VIC rotation (flip X, then transpose).
//
// Config struct layout (bdk vic_config_t; all fields little-endian u64
// bitfields):
//   0x10 OutputConfig         u64 0: FlipX b48, FlipY b49, Transpose b50
//                             u64 1: TargetRect L b0, R b16, T b32, B b48
//   0x20 OutputSurfaceConfig  u64 0: format b0, BlkKind b11, BlkHeight b15,
//                                    W-1 b32, H-1 b46
//   0x90 + n * 0xB0  slot n:  u64 0: SlotEnable b0
//                             u64 4/5: SourceRect L|R, T|B (16.16, b0/b32)
//                             u64 6: DestRect L b0, R b16, T b32, B b48
//        + 0x40 SlotSurfaceConfig: as OutputSurfaceConfig
// Rects are inclusive. The output surface size is given pre-transpose, so a
// transposed target is (H x W) pixels, which is how the DC reads it back.

static constexpr uint32_t kVicPramBase  = 0x1500; // VIC_SC_PRAMBASE      0x14000
static constexpr uint32_t kVicSfcBase0  = 0x150C; // VIC_SC_SFC0_BASE_LUMA 0x14300 + n * 0x100
static constexpr uint32_t kVicTarget    = 0x1880; // VIC_BL_TARGET_BASADR 0x22000
static constexpr uint32_t kVicCompose   = 0x1400; // VIC_FC_COMPOSE       0x10000

struct VicRegs {
  uint64_t cfg = 0;         // config struct address
  uint64_t sfc[8] = {};     // slot n luma surface
  uint64_t target = 0;      // output surface
};
static VicRegs vic;

static uint32_t vic_read(EmuState *state, uint64_t addr) {
  (void)state;
  (void)addr;
  return 0; // PVIC_FALCON_IDLESTATE: composes finish at once
}

// Bytes per pixel of a VIC pixel format (bdk vic.h), 0 if not modelled.
static uint32_t vic_bpp(uint32_t fmt) {
  if (fmt == 1)
    return 1; // L8
  if (fmt >= 21 && fmt <= 24)
    return 2; // X1B5G5R5 .. B5G5R5X1
  if (fmt >= 31 && fmt <= 38)
    return 4; // A8B8G8R8 .. R8G8B8X8
  return 0;
}

// A host pointer to [addr, addr + len) of emulated DRAM, or null.
static uint8_t *vic_dram(EmuState *state, uint64_t addr, uint64_t len) {
  if (!state->dram_low_ptr || addr < DRAM_BASE ||
      addr + len > DRAM_BASE + DRAM_WINDOW_SIZE)
    return nullptr;
  return state->dram_low_ptr + (addr - DRAM_BASE);
}

static void vic_compose(EmuState *state) {
  uint64_t c[0x610 / 8];
  if (!vic.cfg || uc_mem_read(state->uc, vic.cfg, c, sizeof(c)) != UC_ERR_OK)
    return;
  auto bits = [](uint64_t v, unsigned lo, unsigned n) {
    return (uint32_t)((v >> lo) & ((1ull << n) - 1));
  };

  bool flip_x = bits(c[2], 48, 1), flip_y = bits(c[2], 49, 1);
  bool transpose = bits(c[2], 50, 1);
  uint32_t tl = bits(c[3], 0, 14), tr = bits(c[3], 16, 14);
  uint32_t tt = bits(c[3], 32, 14), tb = bits(c[3], 48, 14);
  uint32_t ofmt = bits(c[4], 0, 7), okind = bits(c[4], 11, 4);
  uint32_t ogob = 1u << std::min(bits(c[4], 15, 4), 5u);
  uint32_t ow = bits(c[4], 32, 14) + 1, oh = bits(c[4], 46, 14) + 1;
  uint32_t bpp = vic_bpp(ofmt);

  static uint64_t logged = ~0ull;
  auto log_once = [&](const char *what) {
    uint64_t key = vic.target ^ ((uint64_t)ofmt << 40) ^ ((uint64_t)okind << 48);
    if (logged != key) {
      logged = key;
      printf("[vic] compose skipped: %s\n", what);
    }
  };
  // BlkKind 0 is pitch, 1 the Tegra 16Bx2 block-linear layout; BlkHeight is
  // log2 of the block height in GOBs.
  if (!bpp || okind > 1)
    return log_once("output surface format or layout not modelled");

  // The output as laid out in memory: a transposed target is oh x ow.
  uint32_t out_w = transpose ? oh : ow, out_h = transpose ? ow : oh;
  uint32_t out_pitch = out_w * bpp, out_gobs = (out_pitch + 63) / 64;
  uint64_t out_len = okind ? bl_size(out_gobs, ogob, out_h)
                           : (uint64_t)out_pitch * out_h;
  uint8_t *dst = vic_dram(state, vic.target, out_len);
  if (!dst)
    return log_once("target outside DRAM");
  if (tr >= ow)
    tr = ow - 1;
  if (tb >= oh)
    tb = oh - 1;

  for (int n = 0; n < 8; n++) {
    const uint64_t *sl = &c[(0x90 + n * 0xB0) / 8];
    if (!(sl[0] & 1) || !vic.sfc[n])
      continue;
    uint32_t sfmt = bits(sl[8], 0, 7), skind = bits(sl[8], 11, 4);
    uint32_t sgob = 1u << std::min(bits(sl[8], 15, 4), 5u);
    uint32_t sw = bits(sl[8], 32, 14) + 1, sh = bits(sl[8], 46, 14) + 1;
    if (skind > 1 || vic_bpp(sfmt) != bpp) {
      log_once("slot surface layout not modelled, or not the output's size "
               "of pixel");
      continue;
    }
    uint32_t src_pitch = sw * bpp, src_gobs = (src_pitch + 63) / 64;
    const uint8_t *src = vic_dram(state, vic.sfc[n],
                                  skind ? bl_size(src_gobs, sgob, sh)
                                        : (uint64_t)src_pitch * sh);
    if (!src)
      continue;
    uint32_t sl_l = bits(sl[4], 16, 14), sl_r = bits(sl[4], 48, 14);
    uint32_t sl_t = bits(sl[5], 16, 14), sl_b = bits(sl[5], 48, 14);
    uint32_t dl = bits(sl[6], 0, 14), dr = bits(sl[6], 16, 14);
    uint32_t dt = bits(sl[6], 32, 14), db = bits(sl[6], 48, 14);
    if (sl_r < sl_l || sl_b < sl_t || dr < dl || db < dt)
      continue;
    uint32_t x0 = std::max(dl, tl), x1 = std::min(dr, tr);
    uint32_t y0 = std::max(dt, tt), y1 = std::min(db, tb);
    if (x0 > x1 || y0 > y1)
      continue;
    // Nearest-neighbour scaling from the source rect onto the dest rect;
    // the column map is worked out once per compose, not per pixel.
    static std::vector<uint32_t> sxs;
    sxs.resize(x1 - x0 + 1);
    for (uint32_t tx = x0; tx <= x1; tx++)
      sxs[tx - x0] = sl_l + (uint32_t)((uint64_t)(tx - dl) *
                                       (sl_r - sl_l + 1) / (dr - dl + 1));
    for (uint32_t ty = y0; ty <= y1; ty++) {
      uint32_t sy = sl_t + (uint32_t)((uint64_t)(ty - dt) * (sl_b - sl_t + 1) /
                                      (db - dt + 1));
      if (sy >= sh)
        continue;
      uint32_t fy = flip_y ? oh - 1 - ty : ty;
      for (uint32_t tx = x0; tx <= x1; tx++) {
        uint32_t sx = sxs[tx - x0];
        if (sx >= sw)
          continue;
        uint32_t fx = flip_x ? ow - 1 - tx : tx;
        uint32_t col = transpose ? fy : fx, row = transpose ? fx : fy;
        uint64_t o = okind ? bl_offset(col * bpp, row, out_gobs, ogob)
                           : (uint64_t)row * out_pitch + (uint64_t)col * bpp;
        uint64_t i = skind ? bl_offset(sx * bpp, sy, src_gobs, sgob)
                           : (uint64_t)sy * src_pitch + (uint64_t)sx * bpp;
        memcpy(dst + o, src + i, bpp);
      }
    }
  }

  uint32_t xf = (flip_x ? dcwin::XF_FLIP_X : 0) |
                (flip_y ? dcwin::XF_FLIP_Y : 0) |
                (transpose ? dcwin::XF_TRANSPOSE : 0);
  if (state->vic_out_addr != vic.target || state->vic_out_xform != xf)
    printf("[vic] compose: %ux%u -> 0x%08llX%s%s%s\n", ow, oh,
           (unsigned long long)vic.target, flip_x ? ", flip X" : "",
           flip_y ? ", flip Y" : "", transpose ? ", transpose" : "");
  state->vic_out_addr = vic.target;
  state->vic_out_xform = xf;
  state->display_dirty = true;
}

static void vic_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - VIC_BASE);
  if (offset == kVicPramBase)
    vic.cfg = (uint64_t)val << 8;
  else if (offset >= kVicSfcBase0 && offset < kVicSfcBase0 + 8 * 4)
    vic.sfc[(offset - kVicSfcBase0) / 4] = (uint64_t)val << 8;
  else if (offset == kVicTarget)
    vic.target = (uint64_t)val << 8;
  else if (offset == kVicCompose && (val & 1))
    vic_compose(state);
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
  return mmio_regs.get(addr);
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
    return mmio_regs.get(addr);
  }
}

// Captured PIO audio. Every word the payload pushes into the FIFO is a whole
// stereo frame - low 16 bits left, high 16 bits right, per the CIF's UNPACK16
// - so the capture is already interleaved PCM and needs only a header.
//
// This exists because "the probe reached its verdict" and "the console made
// the right noise" are different claims, and only one of them can be checked
// by reading registers. Writing the samples out means the tone an emulated
// run produces can actually be listened to, and measured, the same way a
// line-in capture of the real console is.
static std::vector<int16_t> audio_pcm;
static bool audio_pcm_full = false;

// 4 M frames is ~3 minutes at this rate: far more than any test tone, and
// bounded so a runaway payload cannot eat memory.
static const size_t AUDIO_PCM_MAX_FRAMES = 4u * 1024u * 1024u;

// Playback on the host sound card.
//
// The take is played AFTER it is complete, not streamed while it is produced.
// That is deliberate. The emulator does not generate samples anywhere near a
// steady rate - a phrase arrives in a burst, then generation stalls entirely
// while the payload talks to the codec over I2C between phrases - so a live
// stream underruns wherever the producer falls behind the sound card, which
// is heard as flicker. Queueing the finished buffer in one go removes the
// producer from the timing path completely: SDL has every sample before the
// first one is played, so it cannot run dry.
//
// The cost is latency - the melody is heard once the payload has finished
// writing it - which for a test tone is a fair trade for hearing it cleanly.
//
// An absent or unusable device (headless CI, no sound server) is not an
// error: playback is skipped and the WAV is still written, so a run never
// fails for want of speakers.
static SDL_AudioDeviceID audio_dev = 0;

static uint32_t audio_rate_hz(void) {
  uint32_t timing = mmio_regs.count(I2S_BASE + 0xA4)
                        ? (mmio_regs[I2S_BASE + 0xA4] & 0x7FFu) : 31u;
  uint32_t frame = 2u * (timing + 1u);
  return frame ? (1500000u / frame) : 23437u;
}

static void audio_play_take(void) {
  if (audio_pcm.empty() || getenv("RCM_EMU_NO_AUDIO"))
    return;

  if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0) {
    printf("[audio] no audio subsystem (%s) - WAV only\n", SDL_GetError());
    return;
  }

  SDL_AudioSpec want{}, have{};
  want.freq     = (int)audio_rate_hz();
  want.format   = AUDIO_S16SYS;
  want.channels = 2;
  want.samples  = 4096;
  want.callback = nullptr;    // queue-driven, no callback

  // No ALLOW_ANY_CHANGE: SDL_QueueAudio does not convert, so a device opened
  // at a different rate or format would replay the take at the wrong pitch.
  // Better to fail and keep the WAV than to play something misleading.
  // One device at a time: a new take replaces the one still playing
  // instead of leaking a device per take.
  if (audio_dev) {
    SDL_CloseAudioDevice(audio_dev);
    audio_dev = 0;
  }
  audio_dev = SDL_OpenAudioDevice(nullptr, 0, &want, &have, 0);
  if (!audio_dev) {
    printf("[audio] no device (%s) - WAV only\n", SDL_GetError());
    return;
  }

  SDL_QueueAudio(audio_dev, audio_pcm.data(),
                 (Uint32)(audio_pcm.size() * sizeof(int16_t)));
  SDL_PauseAudioDevice(audio_dev, 0);
  printf("[audio] playing %.2f s on the host at %d Hz stereo\n",
         (double)(audio_pcm.size() / 2) / (double)have.freq, have.freq);
  fflush(stdout);

  // Deliberately does NOT wait for playback to finish. This runs on the
  // emulation thread, so blocking here freezes the whole emulator for the
  // length of the take - nearly ten seconds for hwtest's melody - during
  // which the window stops repainting and Windows paints it as hung. The
  // whole buffer is already queued, so SDL drains it in the background
  // while emulation carries on, and the device is left open for it.
}


static void audio_write_wav(void) {
  if (audio_pcm.empty())
    return;

  // The frame length the payload programmed decides the rate: I2S_TIMING
  // holds the per-channel bit count minus one, so a frame is 2 x (cnt + 1)
  // bit clocks and fs = bclk / that. hwtest drives BCLK at PLLA_OUT0 / 80 =
  // 1.5 MHz, which with its TIMING of 31 gives 23437.5 Hz - the real rate,
  // and the one its note table is computed against. Falling back to a
  // nominal 48 kHz would replay the melody at twice the intended pitch.
  uint32_t timing = mmio_regs.count(I2S_BASE + 0xA4)
                        ? (mmio_regs[I2S_BASE + 0xA4] & 0x7FFu) : 31u;
  uint32_t frame = 2u * (timing + 1u);
  uint32_t rate  = frame ? (1500000u / frame) : 23437u;

  const char *path = "last_audio.wav";
  FILE *f = fopen(path, "wb");
  if (!f) {
    printf("[audio] could not open %s\n", path);
    return;
  }

  uint32_t data_bytes = (uint32_t)(audio_pcm.size() * sizeof(int16_t));
  uint32_t byte_rate  = rate * 2u * 2u;   // stereo, 16-bit
  auto u32 = [&](uint32_t v) { fwrite(&v, 4, 1, f); };
  auto u16 = [&](uint16_t v) { fwrite(&v, 2, 1, f); };

  fwrite("RIFF", 1, 4, f); u32(36 + data_bytes); fwrite("WAVE", 1, 4, f);
  fwrite("fmt ", 1, 4, f); u32(16); u16(1); u16(2);
  u32(rate); u32(byte_rate); u16(4); u16(16);
  fwrite("data", 1, 4, f); u32(data_bytes);
  fwrite(audio_pcm.data(), 1, data_bytes, f);
  fclose(f);

  printf("[audio] wrote %s: %zu frames, %u Hz stereo%s\n", path,
         audio_pcm.size() / 2, rate,
         audio_pcm_full ? " (truncated)" : "");
  fflush(stdout);
  audio_play_take();          // buffer is complete: safe to play it straight
  audio_pcm.clear();
  audio_pcm_full = false;
}

static void admaif_write(EmuState *state, uint64_t addr, uint32_t val) {
  (void)state;
  uint32_t offset = (uint32_t)(addr - ADMAIF_BASE);
  if (offset == 0x32C) {
    // One packed word = one stereo frame. Record it rather than dropping it.
    if (audio_pcm.size() / 2 < AUDIO_PCM_MAX_FRAMES) {
      audio_pcm.push_back((int16_t)(val & 0xFFFF));          // left
      audio_pcm.push_back((int16_t)((val >> 16) & 0xFFFF));  // right
    } else {
      audio_pcm_full = true;
    }
    return;
  }
  if (offset == 0x300 && val == 0) {
    // TX channel disabled: the payload has finished feeding the port, so the
    // take is complete. Write it out and play it. Doing this here rather
    // than at exit means a run killed on its time budget still produces
    // both the file and the sound.
    audio_write_wav();
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
  return mmio_regs.get(addr);
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
// an ACTMON test is checking for. The COP monitor is the exception: it counts
// the halted time instead, as the hardware does.
static constexpr uint32_t ACTMON_OFF_IN_SYSREG = 0x800;
static constexpr uint32_t ACTMON_FULL_COUNT = 1920000; // 100.0 % for one period
static constexpr int ACTMON_NDEV = 7;
static constexpr int ACTMON_DEV_COP = 1;   // device order: CPU, COP, AHB, ...

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
static void actmon_sample(EmuState *state, ActmonDev &d, bool counts_halt) {
  uint64_t now = state->emu_usec;
  uint64_t slept = state->bpmp_slept_us;
  if (!d.seeded) {
    d.last_us = now;
    d.last_slept = slept;
    d.seeded = true;
    return;
  }
  // `now` can step back when CPU0 takes the bus (it trails the BPMP by up to
  // a slice); an unsigned difference would wrap into a huge window.
  if (now < d.last_us + 200)
    return;
  uint64_t window = now - d.last_us;
  uint64_t win_slept = slept - d.last_slept;
  if (win_slept > window)
    win_slept = window;
  uint64_t active = window - win_slept;
  // The BPMP (COP) monitor counts the cycles the BPMP-Lite spends halted,
  // not running (TRM 41.x: "counts the clock cycles when BPMP-Lite is in
  // halt state"); the others count activity.
  uint64_t counted = counts_halt ? win_slept : active;

  uint32_t c = (uint32_t)((counted * ACTMON_FULL_COUNT) / window);
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
      if (d.ctrl & (1u << 31)) actmon_sample(state, d, dev == ACTMON_DEV_COP);
      return d.count;
    case 0x20:
      if (d.ctrl & (1u << 31)) actmon_sample(state, d, dev == ACTMON_DEV_COP);
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
        actmon_sample(state, d, dev == ACTMON_DEV_COP);
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
  // SB (secure boot) block at +0x200: CSR and the CCPLEX reset vector.
  if (offset >= 0x200 && offset < 0x300)
    return ccplex_sb_read(offset - 0x200);
  return 0;
}

static void sysreg_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t offset = (uint32_t)(addr - SYSREG_BASE);
  if (offset >= ACTMON_OFF_IN_SYSREG) {
    actmon_write(state, offset - ACTMON_OFF_IN_SYSREG, val);
    return;
  }
  if (offset >= 0x200 && offset < 0x300) {
    ccplex_sb_write(state, offset - 0x200, val);
    return;
  }
  TRACE("[sysreg] W: 0x%08X = 0x%08X\n", offset, val);
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

// ==================== Bus dispatch ====================
//
// One dispatch serves every master. It used to live inside a pair of Unicorn
// UC_HOOK_MEM_READ / UC_HOOK_MEM_WRITE callbacks over plain RAM pages, with
// the read result smuggled back by uc_mem_write()-ing it into the page before
// the load completed. That had two costs well beyond the handlers themselves:
//
//  - Unicorn routes EVERY guest load and store through its slow path while
//    any memory hook exists, so IRAM and DRAM traffic paid for MMIO's hooks;
//  - each read did a uc_mem_write, i.e. a linear walk of Unicorn's mapped
//    region list, which is dozens of entries long here.
//
// The windows are now uc_mmio_map() regions: loads and stores to RAM run on
// the translator's fast path, and an MMIO access is one callback. The same
// callbacks serve CPU0's engine, which is what lets the two cores share one
// set of register models.

// I2C4 and I2C6: controllers with nothing modelled behind them. Every
// transaction completes at once with the address NACKed, so a bus scan finds
// an empty bus instead of a device at every address (a 0 STATUS reads as
// "ACK"). CNFG reads back without SEND, which the hardware clears when done.
static uint32_t i2c_empty_bus_read(uint64_t address) {
  switch ((uint32_t)(address & 0xFF)) {
  case 0x00: return mmio_regs.get(address) & ~(1u << 9);
  case 0x1C: return I2C_STATUS_NOACK;
  case 0x8C: return 0;   // CONFIG_LOAD self-clears
  default:   return mmio_regs.get(address);
  }
}

uint32_t mmio_bus_read(EmuState *state, uint64_t address, unsigned size) {
  (void)size;
  if (address >= TMR_BASE && address < TMR_BASE + TMR_SIZE) {
    // TIMERUS_CNTR_1US. The rest of the timer block reads 0. The BPMP's
    // reads go through the clock's poll pacing; CPU0 has its own clock.
    uint32_t off = (uint32_t)(address - TMR_BASE);
    if (off == 0x14)                  // TIMERUS_USEC_CFG, reset 0x0000000C
      return mmio_regs.get(address, 0x0000000C);
    if (off != 0x10)
      return mmio_regs.get(address);
    return g_bus_master == BUS_BPMP ? bpmp_timerus_read(state)
                                    : (uint32_t)ccplex_now_us();
  }
  if (address >= GPIO_BASE && address < GPIO_BASE + GPIO_SIZE)
    return gpio_read(state, address);
  if ((address >= PCIE_BLOCK_BASE &&
       address < PCIE_BLOCK_BASE + PCIE_BLOCK_SIZE) ||
      (address >= PCIE_CS_BASE && address < PCIE_CS_BASE + PCIE_CS_MODEL_SIZE) ||
      (address >= PCIE_MEM_BASE && address < PCIE_MEM_BASE + PCIE_MEM_MODEL_SIZE))
    return pcie_read(state, address);
  if (address >= XUSB_PADCTL_BASE &&
      address < XUSB_PADCTL_BASE + XUSB_PADCTL_SIZE)
    return padctl_read(state, address);
  if (address >= MSELECT_BASE && address < MSELECT_BASE + MSELECT_SIZE)
    return mselect_read(state, address);
  if (address >= VIC_BASE && address < VIC_BASE + VIC_SIZE)
    return vic_read(state, address);
  if (address >= SYSREG_BASE && address < SYSREG_BASE + SYSREG_SIZE)
    return sysreg_read(state, address);
  if (address >= BPMP_CACHE_BASE && address < BPMP_CACHE_BASE + BPMP_CACHE_SIZE)
    return bpmp_cache_read(state, address);
  if (address >= SOC_THERM_BASE && address < SOC_THERM_BASE + SOC_THERM_SIZE)
    return soc_therm_read(state, address);
  if (address >= FLOW_CTLR_BASE && address < FLOW_CTLR_BASE + FLOW_CTLR_SIZE)
    return flow_read(state, address);
  if (address >= I2S_BASE && address < I2S_BASE + I2S_SIZE)
    return i2s_read(state, address);
  if (address >= ADMAIF_BASE && address < ADMAIF_BASE + ADMAIF_SIZE)
    return admaif_read(state, address);
  if (address >= ADMA_BASE && address < ADMA_BASE + ADMA_SIZE)
    return adma_read(state, address);
  if (address >= RTC_BASE && address < RTC_BASE + RTC_SIZE)
    return rtc_read(state, address);
  // I²C2 / BH1730 ambient light sensor (slave 0x29).
  if (address >= I2C2_BASE && address < I2C2_BASE + I2C2_SIZE)
    return i2c2_read(state, address);
  // I²C3 / STMFTS touchscreen (slave 0x49). Must be tested before the I²C1
  // branch, which would otherwise swallow the I²C3 page.
  if (address >= I2C3_BASE && address < I2C3_BASE + I2C3_SIZE)
    return i2c3_read(state, address);
  if (address >= I2C1_BASE && address < I2C1_BASE + 0x100)
    return i2c_read(state, address);
  if (address >= I2C5_BASE && address < I2C5_BASE + I2C_CTRL_SIZE)
    return i2c_read(state, address);
  if ((address >= I2C4_BASE && address < I2C4_BASE + I2C_CTRL_SIZE) ||
      (address >= I2C6_BASE && address < I2C6_BASE + I2C_CTRL_SIZE))
    return i2c_empty_bus_read(address);
  if (address >= DISPLAY_A_BASE && address < DISPLAY_A_BASE + DISPLAY_SIZE)
    return display_read(state, address);
  if (address >= PMC_BASE && address < PMC_BASE + PMC_SIZE)
    return pmc_read(state, address);
  if (address >= CLK_RST_BASE && address < CLK_RST_BASE + CLK_RST_SIZE)
    return clk_rst_read(state, address);
  if (address >= FUSE_BASE && address < FUSE_BASE + FUSE_SIZE)
    return fuse_read(state, address);
  if ((address >= EMC_BASE && address < EMC_BASE + EMC_SIZE) ||
      (address >= EMC0_BASE && address < EMC0_BASE + EMC_SIZE) ||
      (address >= EMC1_BASE && address < EMC1_BASE + EMC_SIZE))
    return emc_read(state, address);
  if (address >= SE_BASE && address < SE_BASE + SE_SIZE)
    return se_read(state, address);
  return misc_read(state, address);
}

// THR writes on any of the five UARTs.
static void uart_write(EmuState *state, uint64_t address, uint32_t val) {
  uint32_t offset = 0;
  int port = uart_port_of(address, &offset);
  if (port < 0)
    return;
  // Note offset 0 is the divisor latch LSB - not the transmit register -
  // whenever DLAB is set, so writing the baud divisor must not be mistaken
  // for a character to print. Every register (including this one) was
  // already cached by the caller; the divisor is additionally kept in
  // UartPort because a later THR write overwrites that cache entry.
  UartPort &up = uart_ports[port];
  uint32_t ubase = uart_bases[port];
  bool dlab = mmio_regs.get(ubase + 0x0C) & 0x80;

  if (offset == 0x00 && dlab) {
    up.divisor = (uint16_t)((up.divisor & 0xFF00) | (val & 0xFF));
  } else if (offset == 0x04 && dlab) {
    up.divisor = (uint16_t)((up.divisor & 0x00FF) | ((val & 0xFF) << 8));
  } else if (offset == 0x04) {
    up.ier = (uint8_t)val;
  } else if (offset == 0x08) {
    up.fcr = (uint8_t)val;
    // FCR. RX_CLR is honoured only on the BT port, where the receive FIFO is
    // fed by an emulated device and dropping it is exactly what the hardware
    // does. On the console ports that FIFO models a human at a terminal, and
    // bdk's uart_init() issues an unconditional RX_CLR - which would silently
    // eat scripted keystrokes.
    if (port == UART_D && (val & 0x02))
      state->uart_rx_fifo[port].clear();
  } else if (offset == 0x10) {
    uint8_t old_mcr = up.mcr;
    up.mcr = (uint8_t)val;
    // Entering or leaving 16550 internal loopback swings all four modem
    // inputs at once - MCR[3:0] are tied onto MSR[7:4] while it is on - so
    // every delta bit latches. That is what makes the payload's first MSR
    // read after its loopback self-test 0x4F, not 0x40.
    if ((old_mcr ^ up.mcr) & 0x10)
      up.msr_delta |= 0x0F;
  }

  if (offset != 0 || dlab)
    return;

  uint8_t b = (uint8_t)val;
  if (up.mcr & 0x10) {
    // Internal loopback: the byte is tied straight back to this port's own
    // receiver inside the controller and never reaches a pad. It must NOT be
    // handed to whatever is on the far end, or the BT chip's H4 parser
    // desyncs on the A5 5A 00 FF test pattern - and it must not reach the TX
    // log either, since it is not something the payload actually transmitted.
    state->uart_rx_fifo[port].push_back(b);
    return;
  }

  // UART-D is the Bluetooth radio's H4 transport.
  if (port == UART_D)
    bt_chip_rx(state, b);

  // Append every byte to the per-port TX log — the console window renders
  // this as scrolling text. Trim from the front if it grows past 64 KB so
  // ImGui rendering stays responsive.
  std::string &log = state->uart_tx_log[port];
  if (b == '\n' || (b >= 0x20 && b < 0x7F))
    log.push_back((char)b);
  if (log.size() > 64 * 1024)
    log.erase(0, log.size() - 48 * 1024);

  // Mirror to host stdout in line-buffered form so existing `grep '[uart]'`
  // workflows still work.
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
    if (uart_len[port] + 1 >= sizeof(uart_line[port]))
      flush_uart_line(port);
    uart_line[port][uart_len[port]++] = (char)b;
  }
}

void mmio_bus_write(EmuState *state, uint64_t address, unsigned size,
                    uint64_t value) {
  uint32_t val = (uint32_t)value;
  mmio_regs[address] = val; // read-back cache for the simple registers

  if (address >= TMR_BASE && address < TMR_BASE + TMR_SIZE) {
    tmr_write(state, address, val);
  } else if (address >= GPIO_BASE && address < GPIO_BASE + GPIO_SIZE) {
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
  } else if (address >= MSELECT_BASE && address < MSELECT_BASE + MSELECT_SIZE) {
    mselect_write(state, address, val);
  } else if ((address >= SDMMC1_BASE && address < SDMMC1_BASE + 0x200) ||
             (address >= SDMMC4_BASE && address < SDMMC4_BASE + 0x200)) {
    // DMA targets IRAM/DRAM, which every engine maps from the same host
    // memory, so the BPMP engine can move the data for either master.
    misc_write(state->uc, state, address, (int64_t)value, (int)size);
  } else if (address >= 0x70006000 && address < 0x70006500) {
    uart_write(state, address, val);
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
  } else if (address >= I2C5_BASE && address < I2C5_BASE + I2C_CTRL_SIZE) {
    i2c_write(state, address, val);
  } else if (address >= DISPLAY_A_BASE &&
             address < DISPLAY_A_BASE + DISPLAY_SIZE) {
    display_write(state, address, val);
  } else if (address >= DSI_BASE && address < DSI_BASE + DSI_SIZE) {
    dsi_write(state, address, val);
  } else if (address >= PMC_BASE && address < PMC_BASE + PMC_SIZE) {
    pmc_write(state, address, val);
  } else if (address >= FLOW_CTLR_BASE &&
             address < FLOW_CTLR_BASE + FLOW_CTLR_SIZE) {
    flow_write(state, address, val);
  } else if (address >= CLK_RST_BASE && address < CLK_RST_BASE + CLK_RST_SIZE) {
    clk_rst_write(state, address, val);
  } else if (address >= FUSE_BASE && address < FUSE_BASE + FUSE_SIZE) {
    fuse_write(state, address, val);
  } else if ((address >= EMC_BASE && address < EMC_BASE + EMC_SIZE) ||
             (address >= EMC0_BASE && address < EMC0_BASE + EMC_SIZE) ||
             (address >= EMC1_BASE && address < EMC1_BASE + EMC_SIZE)) {
    emc_write(state, address, val);
  } else if (address >= SE_BASE && address < SE_BASE + SE_SIZE) {
    se_write(state, address, val);
  } else {
    // The TRACE is the only output here. Its fflush used to sit outside the
    // macro, so every write to an unmodelled register forced a write(2) of
    // whatever stdout had buffered even with tracing off.
    if (emu_trace_enabled &&
        !((address >= SDMMC1_BASE && address < SDMMC1_BASE + 0x1000) ||
          (address >= SDMMC4_BASE && address < SDMMC4_BASE + 0x1000) ||
          (address >= I2C1_BASE && address < I2C1_BASE + 0x1000) ||
          (address >= I2C5_BASE && address < I2C5_BASE + 0x1000))) {
      printf("[mmio] W: 0x%08llX = 0x%08X (%s)\n", (unsigned long long)address,
             val, g_bus_master == BUS_CPU0 ? "CPU0" : "BPMP");
      fflush(stdout);
    }
    misc_write(state->uc, state, address, (int64_t)val, (int)size);
  }
}

// ==================== Unicorn MMIO windows ====================

namespace {

// One per uc_mmio_map() call. Unicorn hands the callbacks an offset relative
// to the start of the mapping, so each window remembers its base.
struct MmioWindow {
  EmuState *state;
  uint64_t base;
  uc_engine *uc;
};
std::vector<std::unique_ptr<MmioWindow>> g_windows;

uint64_t mmio_cb_read(uc_engine *uc, uint64_t offset, unsigned size,
                      void *user_data) {
  (void)uc;
  const MmioWindow *w = (const MmioWindow *)user_data;
  uint64_t addr = w->base + offset;
  if (g_bus_master == BUS_CPU0)
    ccplex_bus_tick(w->state);
  if (size == 8) {
    // A 64-bit load from CPU0 reaches these 32-bit register files as two
    // consecutive word accesses, low word first.
    uint64_t lo = mmio_bus_read(w->state, addr, 4);
    uint64_t hi = mmio_bus_read(w->state, addr + 4, 4);
    return lo | (hi << 32);
  }
  uint32_t v = mmio_bus_read(w->state, addr, size);
  if (size < 4)
    v &= (1u << (size * 8)) - 1;
  return v;
}

void mmio_cb_write(uc_engine *uc, uint64_t offset, unsigned size,
                   uint64_t value, void *user_data) {
  (void)uc;
  const MmioWindow *w = (const MmioWindow *)user_data;
  uint64_t addr = w->base + offset;
  if (g_bus_master == BUS_CPU0)
    ccplex_bus_tick(w->state);
  if (size == 8) {
    mmio_bus_write(w->state, addr, 4, value & 0xFFFFFFFFu);
    mmio_bus_write(w->state, addr + 4, 4, value >> 32);
    return;
  }
  mmio_bus_write(w->state, addr, size, value);
}

uc_err map_window(uc_engine *uc, EmuState *state, uint64_t base,
                  uint64_t size) {
  g_windows.push_back(std::make_unique<MmioWindow>(MmioWindow{state, base, uc}));
  MmioWindow *w = g_windows.back().get();
  return uc_mmio_map(uc, base, size, mmio_cb_read, w, mmio_cb_write, w);
}

// Every peripheral window a payload is known to touch. Overlaps and
// duplicates are fine: the list is flattened to 4 KiB pages below.
const struct {
  uint64_t base;
  uint64_t size;
} kRegions[] = {
    {CLK_RST_BASE, CLK_RST_SIZE},
    {TMR_BASE, TMR_SIZE},
    {GPIO_BASE, GPIO_SIZE},
    {PINMUX_BASE, PINMUX_SIZE},
    {UART_A_BASE, UART_SIZE},
    {I2C1_BASE, I2C_SIZE},
    {I2C5_BASE, I2C_SIZE},
    {PWM_BASE, PWM_SIZE},
    {RTC_BASE, RTC_SIZE},
    {PMC_BASE, PMC_SIZE},
    {FUSE_BASE, FUSE_SIZE},
    {FLOW_CTLR_BASE, FLOW_CTLR_SIZE},
    {EXCP_VEC_BASE, EXCP_VEC_SIZE},
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
    {BPMP_CACHE_BASE, BPMP_CACHE_SIZE},
    {XUSB_PADCTL_BASE, XUSB_PADCTL_SIZE},
    {MSELECT_BASE, MSELECT_SIZE},
    // PCIe. T210 puts the root complex at the BOTTOM of the address map,
    // below every other peripheral.
    {PCIE_BLOCK_BASE, PCIE_BLOCK_SIZE},
    {PCIE_CS_BASE, PCIE_CS_MODEL_SIZE},
    {PCIE_MEM_BASE, PCIE_MEM_MODEL_SIZE},
};

// TZRAM is SRAM, not registers, and every master sees the same 64 KiB. It
// used to be an MMIO window whose "contents" were the register cache, which
// only held whole words at the exact address last written.
uint8_t *g_tzram = nullptr;

bool in_mmio_band(uint64_t a) { return a >= 0x50000000 && a < 0x80000000; }

// Unmapped BPMP access. Addresses in the peripheral band get an MMIO page of
// their own, so the generic register cache and misc handlers see them like
// any other register; anything else becomes plain RAM, as before.
bool hook_unmapped(uc_engine *uc, uc_mem_type type, uint64_t address, int size,
                   int64_t value, void *user_data) {
  EmuState *state = (EmuState *)user_data;
  uint32_t pc = 0;
  uc_reg_read(uc, UC_ARM_REG_PC, &pc);
  if (type == UC_MEM_READ_UNMAPPED || type == UC_MEM_FETCH_UNMAPPED) {
    TRACE("[mmio] UNMAPPED R: 0x%08lX (PC=0x%08X)\n", (unsigned long)address, pc);
  } else {
    TRACE("[mmio] UNMAPPED W: 0x%08lX = 0x%08llX (PC=0x%08X)\n",
          (unsigned long)address, (unsigned long long)value, pc);
  }
  uint64_t page = address & ~0xFFFULL;
  if (in_mmio_band(page) && type != UC_MEM_FETCH_UNMAPPED)
    return map_window(uc, state, page, 0x1000) == UC_ERR_OK;
  if (uc_mem_map(uc, page, 0x1000, UC_PROT_ALL) != UC_ERR_OK)
    return false;
  if (type == UC_MEM_WRITE_UNMAPPED)
    uc_mem_write(uc, address, &value, size);
  return true;
}

} // namespace

void mmio_map_bus(uc_engine *uc, EmuState *state) {
  // Flatten to pages, then map each run of contiguous pages as one window.
  std::vector<uint64_t> pages;
  for (const auto &r : kRegions) {
    uint64_t first = r.base & ~0xFFFULL;
    uint64_t end = (r.base + r.size + 0xFFF) & ~0xFFFULL;
    for (uint64_t p = first; p < end; p += 0x1000)
      pages.push_back(p);
  }
  std::sort(pages.begin(), pages.end());
  pages.erase(std::unique(pages.begin(), pages.end()), pages.end());
  for (size_t i = 0; i < pages.size();) {
    size_t j = i + 1;
    while (j < pages.size() && pages[j] == pages[j - 1] + 0x1000)
      j++;
    uint64_t base = pages[i], len = pages[j - 1] + 0x1000 - base;
    uc_err err = map_window(uc, state, base, len);
    if (err != UC_ERR_OK)
      fprintf(stderr, "[mmio] cannot map 0x%08llX+0x%llX: %s\n",
              (unsigned long long)base, (unsigned long long)len,
              uc_strerror(err));
    i = j;
  }

  if (!g_tzram)
    g_tzram = (uint8_t *)calloc(1, TZRAM_SIZE);
  if (g_tzram)
    uc_mem_map_ptr(uc, TZRAM_BASE, TZRAM_SIZE, UC_PROT_ALL, g_tzram);
}

void mmio_unmap_bus(uc_engine *uc) {
  g_windows.erase(std::remove_if(g_windows.begin(), g_windows.end(),
                                 [uc](const std::unique_ptr<MmioWindow> &w) {
                                   return w->uc == uc;
                                 }),
                  g_windows.end());
}

void mmio_init(uc_engine *uc, EmuState *state) {
  static uc_hook h_unmapped;

  mmio_map_bus(uc, state);
  pcie_reset(state);
  ccplex_reset(state);
  sdhci_reset_regs(sdhci_regs[0]);
  sdhci_reset_regs(sdhci_regs[1]);
  dc_reset(state);

  uc_hook_add(uc, &h_unmapped,
              UC_HOOK_MEM_READ_UNMAPPED | UC_HOOK_MEM_WRITE_UNMAPPED,
              (void *)hook_unmapped, state, 1, 0);
}

void mmio_soft_reset(EmuState *state, bool power_cycle) {
  // A reboot is a SoC reset: every block on the SoC goes back to its power-
  // on state. Only the always-on side survives - PMC SCRATCH0 and the reset
  // status (TRM 12.x: "MAIN_RST resets everything but scratch 0 and reset
  // status"), the RTC - and, unless the board was power-cycled, the PMIC and
  // the regulators behind it, which are not on the SoC at all.
  //
  // This used to reset only the CCPLEX and PCIe models, so a payload run
  // after a reboot found GPIOs, pinmux, UART modes, powered partitions,
  // unlocked PLLs, SE keys and a half-read touch event where the previous run
  // left them - and, after a PMIC reset, CPU rails still switched on.
  ccplex_reset(state);
  pcie_reset(state);
  se_engine_reset();
  i2c3_reset(state);

  mmio_regs.clear();
  for (auto &u : uart_ports)
    u = UartPort();
  for (auto &fifo : state->uart_rx_fifo)
    fifo.clear();
  bt_chip = BtChip();
  codec_regs.clear();

  i2c_slave_addr = i2c_reg_addr = 0;
  i2c_cmd_data1 = 0;
  i2c_cnfg_reg[0] = i2c_cnfg_reg[1] = 0;
  pkt_i2c1 = PacketState();
  pkt_i2c5 = PacketState();
  i2c2_slave = i2c2_reg = 0;
  i2c2_cnfg = 0;
  if (power_cycle) {
    max77620_regs_ready = false;   // re-seeded on next access
    max77812_ready = false;
    memcpy(max77621_regs, kMax77621Seed, sizeof(max77621_regs));
  }

  g_kfuse_keyaddr = 0;
  pmc_pwrgate_status = 1u << 3;
  pmc_io_dpd_status[0] = pmc_io_dpd_status[1] = 0;
  uint32_t scratch0 = pmc_regs[0x50 / 4], rst_status = pmc_regs[0x1B4 / 4];
  memset(pmc_regs, 0, sizeof(pmc_regs));
  if (!power_cycle) {
    pmc_regs[0x50 / 4] = scratch0;
    pmc_regs[0x1B4 / 4] = rst_status;
  }
  if (power_cycle) {
    rtc_base_us = 0;
    rtc_sec_adjust = 0;
  } else {
    rtc_base_us += state->emu_usec;   // the RTC keeps counting through it
  }
  rtc_shadow_seconds = 0;

  flow_ram_repair = RAM_REPAIR_RESET;
  memset(tmr_armed_us, 0, sizeof(tmr_armed_us));
  car_banks = CarBanks();
  g_fuse_ctrl_addr = 0;
  emc = EmcState();
  g_dsi_pending_dcs_cmd = 0;
  memset(g_dsi_rx_fifo, 0, sizeof(g_dsi_rx_fifo));
  g_dsi_rx_count = g_dsi_rx_pos = 0;
  vic = VicRegs();
  for (auto &d : actmon_dev)
    d = ActmonDev();
  actmon_glb_period = 0;
  audio_pcm.clear();
  audio_pcm_full = false;

  // Storage controllers. The images stay open; the eMMC is back in its user
  // area, as after CMD0.
  state->sdmmc_arg = state->sdmmc_sysad = 0;
  state->sdmmc_norintsts = state->sdmmc_errintsts = 0;
  memset(state->sdmmc_rsp, 0, sizeof(state->sdmmc_rsp));
  state->sdmmc_hostctl = 0;
  state->sdmmc_blksize = state->sdmmc_blkcnt = state->sdmmc_trnmod = 0;
  state->sdmmc_adma_addr = 0;
  state->sdmmc4_arg = state->sdmmc4_sysad = 0;
  state->sdmmc4_norintsts = state->sdmmc4_errintsts = 0;
  memset(state->sdmmc4_rsp, 0, sizeof(state->sdmmc4_rsp));
  state->sdmmc4_hostctl = 0;
  state->sdmmc4_blksize = state->sdmmc4_blkcnt = state->sdmmc4_trnmod = 0;
  state->sdmmc4_adma_addr = 0;
  state->emmc_partition = 0;
  state->last_cmd_was_55 = state->last_cmd4_was_55 = false;
  sdhci_reset_regs(sdhci_regs[0]);
  sdhci_reset_regs(sdhci_regs[1]);
  if (power_cycle)
    g_ext_csd_ready = false;
  else
    emmc_ext_csd_card_reset();
  sd_card = SdCard(); // the reset drops PE4, and the card's VDD with it

  // Display controller and VIC back to their defaults.
  dc_reset(state);

  state->bpmp_halted = false;
}
