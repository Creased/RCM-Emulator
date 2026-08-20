#include "pcie.h"

#include <cstdio>
#include <cstring>

#include "../emu_state.h"
#include "memory_map.h"

// ==========================================================================
// Register map (tegra210.dtsi pcie@1003000, U-Boot drivers/pci/pci_tegra.c)
// ==========================================================================

#define AFI_AXI_BAR0_SZ      0x00
#define AFI_AXI_BAR0_START   0x18
#define AFI_FPCI_BAR0        0x30
#define AFI_CONFIGURATION    0xAC
#define  AFI_CONFIGURATION_EN_FPCI (1u << 0)
#define AFI_PCIE_CONFIG      0xF8
#define  AFI_PCIE_CONFIG_DISABLE(x) (1u << ((x) + 1))
#define AFI_FUSE             0x104
#define AFI_PEX0_CTRL        0x110
#define AFI_PEX1_CTRL        0x118
#define  AFI_PEX_CTRL_RST         (1u << 0)
#define  AFI_PEX_CTRL_REFCLK_EN   (1u << 3)
#define AFI_PLLE_CONTROL     0x160

#define PADS_REFCLK_CFG0     0xC8

#define RP_LINK_CONTROL_STATUS 0x090
#define  RP_LINK_DL_ACTIVE     (1u << 29)
#define RP_VEND_XP             0xF00
#define  RP_VEND_XP_DL_UP      (1u << 30)
#define RP_VEND_CTL2           0xFA8
#define RP_PRIV_MISC           0xFE0

// XUSB pad controller (U-Boot arch/arm/mach-tegra/tegra210/xusb-padctl.c).
#define PADCTL_ELPG_PROGRAM  0x024
#define PADCTL_USB3_PAD_MUX  0x028
#define PADCTL_UPHY_PLL_P0_CTL1 0x360
#define  UPHY_CTL1_LOCKDET   (1u << 15)
#define  UPHY_CTL1_ENABLE    (1u << 3)
#define  UPHY_CTL1_IDDQ      (1u << 0)
#define PADCTL_UPHY_PLL_P0_CTL2 0x364
#define  UPHY_CTL2_CAL_DONE  (1u << 1)
#define  UPHY_CTL2_CAL_EN    (1u << 0)
#define PADCTL_UPHY_PLL_P0_CTL4 0x36C
#define PADCTL_UPHY_PLL_P0_CTL5 0x370
#define PADCTL_UPHY_PLL_P0_CTL8 0x37C
// T210B01 configures this PLL through an indirect config port instead of the
// CAL_CTRL/DCO_CTRL magic numbers every public T210 driver writes. L4T's
// tegra210_pex_uphy_enable forks on t210b01_compatible(); take the T210
// branch on a Mariko and the brick is never configured, so CAL_DONE never
// asserts. Measured on hardware, and it cost a long hunt on the PCIe side
// before anyone looked at the PLL - so the model enforces it.
#define PADCTL_UPHY_PLL_P0_CTL10 0x384
#define PADCTL_UPHY_PLL_P0_CTL8 0x37C
#define  UPHY_CTL8_RCAL_DONE (1u << 31)
#define  UPHY_CTL8_RCAL_EN   (1u << 12)

// CLK_RST bits this model cares about (bdk soc/clock.h).
#define CAR_RST_DEVICES_U   0x00C
#define CAR_CLK_OUT_ENB_U   0x018
#define CAR_PLLE_BASE       0x0E8
#define  PLLE_BASE_ENABLE   (1u << 31)
#define CAR_PLLE_MISC       0x0EC
#define  PLLE_MISC_LOCK     (1u << 11)
#define CAR_RST_DEVICES_Y   0x2A4
#define CAR_RST_DEV_Y_SET   0x2A8
#define CAR_RST_DEV_Y_CLR   0x2AC
#define CAR_RST_DEVICES_W   0x35C
#define CAR_RST_DEV_U_SET   0x310
#define CAR_RST_DEV_U_CLR   0x314
#define CAR_CLK_ENB_U_SET   0x330
#define CAR_CLK_ENB_U_CLR   0x334
#define CAR_RST_DEV_W_SET   0x438
#define CAR_RST_DEV_W_CLR   0x43C
#define CAR_PLLREFE_BASE    0x4C4
#define  PLLREFE_BASE_ENABLE (1u << 30)
#define CAR_PLLREFE_MISC    0x4C8
#define  PLLREFE_MISC_LOCK  (1u << 27)
// T210B01 ONLY. NVIDIA's tegra210b01 device tree gives the padctl PCIe pad a
// SECOND clock, "uphy_mgmt" = PLL_P_UPHY_OUT, that Erista's pad does not have,
// and L4T enables it in tegra210_pcie_pad_probe() before any UPHY register is
// touched. It clocks the UPHY calibration state machine: gated, CAL_EN latches
// and the FSM never advances, so CAL_DONE stays 0 for ever while every plain
// padctl register still behaves. Measured on a real Mariko. PLLP_MISC1 carries
// the branch gates; PEX_SATA_USB_RX_BYP is repurposed on B01 as this clock's
// 7.1 divider (bits 7:0) plus enable gate (bit 8).
#define CAR_PLLP_MISC1      0x680
#define  PLLP_MISC1_XUSB_EN (1u << 28)
#define CAR_PEX_SATA_USB_RX_BYP 0x6D0
#define  UPHY_MGMT_CLK_EN   (1u << 8)

#define CAR_RST_DEVICES_V   0x358
#define CAR_RST_DEV_V_SET   0x430
#define CAR_RST_DEV_V_CLR   0x434
#define CLK_V_MSELECT       3

#define CLK_U_PCIE          6
#define CLK_U_AFI           8
#define CLK_U_PCIEXCLK      10
#define CLK_W_XUSB_PADCTL   14
#define CLK_Y_PEX_USB_UPHY  13

// CYW4356 Table 62 (p147). The emulated endpoint refuses to answer before
// these have elapsed, so a payload that skips them fails here rather than
// on a bench.
static constexpr uint64_t T_VDD_TO_POR_US   = 57000;  // WL_REG_ON -> POR done
static constexpr uint64_t T_REFCLK_STABLE_US = 10000; // refclk -> PERST# high

// Broadcom identity (brcm_hw_ids.h, brcmfmac/pcie.c).
static constexpr uint32_t BRCM_4356_CFG0    = 0x43EC14E4;
static constexpr uint32_t BRCM_BAR0_WINDOW  = 0x80;
static constexpr uint32_t BRCM_SI_ENUM_BASE = 0x18000000;
// ChipCommon ChipID: id 0x4356, rev 2, package 0, 12 cores, type 0.
static constexpr uint32_t BRCM_4356_CHIPID  = 0x0C024356;

// ==========================================================================

namespace {

struct RootPort {
    uint32_t reg[0x1000 / 4] = {};
    bool     link = false;
    uint64_t refclk_us = 0;   // emu time REFCLK_EN went high, 0 = never
    uint64_t rst_us = 0;      // emu time PERST# was released, 0 = still held
    const char *last_block = nullptr;
};

struct PcieModel {
    uint32_t afi[0x800 / 4] = {};
    uint32_t pads[0x800 / 4] = {};
    RootPort rp[2];

    // Endpoint config space, 256 bytes. Only bus 1 device 0 exists.
    uint32_t ep[64] = {};
    bool     ep_present = false;

    // Shadow of the clock controller bits the link depends on.
    uint32_t rst_u = 0xFFFFFFFF;   // everything in reset at cold start
    uint32_t enb_u = 0;
    uint32_t rst_y = 0xFFFFFFFF;
    uint32_t rst_w = 0xFFFFFFFF;
    // MSELECT sits between every master and the AXI slaves, PCIe included.
    // bdk's hw_init() clocks it but leaves it in reset; only ccplex.c, on
    // the path that boots the A57s, clears it. Held in reset it does not
    // error a transaction, it never completes one - so on real silicon the
    // first PCIe read hangs the BPMP with no abort and no way back. That is
    // not something this emulator can reproduce faithfully (Unicorn has no
    // notion of a stalled bus), so it does the next best thing: it refuses
    // the access, says so loudly, and keeps the link down.
    uint32_t rst_v = 0xFFFFFFFF;
    bool     mselect_warned = false;
    bool     padctl_warned = false;
    uint32_t mselect[0x1000 / 4] = {};
    uint32_t pllp_misc1 = 0;
    uint32_t uphy_mgmt = 0;
    bool     mgmt_warned = false;
    bool     plle = false;
    bool     pllrefe = false;
    bool     powergated = true;

    // XUSB pad controller.
    uint32_t padctl[0x1000 / 4] = {};
    bool     uphy_cal_ran = false;
    bool     uphy_brick_cfg = false;   // B01 table loaded through CTL10
    bool     brick_warned = false;
    bool     uphy_locked = false;
    bool     uphy_rcal_done = false;

    // WL_REG_ON.
    bool     wl_on = false;
    uint64_t wl_on_us = 0;
};

PcieModel pcie;

// BAR sizing masks: a write of all-ones reads back ~(size-1) with the type
// bits preserved, which is how any enumerator discovers how big a BAR is.
// BAR0 is 32 KiB of registers, BAR2 the 2 MiB TCM; both are 64-bit
// prefetchable, which is why brcmfmac calls them resource 0 and resource 2.
constexpr uint32_t EP_BAR_TYPE = 0x0000000C;   // 64-bit, prefetchable
constexpr uint32_t EP_BAR0_MASK = ~(0x8000u - 1);
constexpr uint32_t EP_BAR2_MASK = ~(0x200000u - 1);

void ep_init() {
    memset(pcie.ep, 0, sizeof(pcie.ep));
    pcie.ep[0x00 / 4] = BRCM_4356_CFG0;          // 14E4:43EC
    pcie.ep[0x04 / 4] = 0x00100000;              // status: capability list
    pcie.ep[0x08 / 4] = 0x02800000;              // network controller, other
    pcie.ep[0x0C / 4] = 0x00000000;              // header type 0
    pcie.ep[0x10 / 4] = EP_BAR_TYPE;             // BAR0 lo: 32 KiB registers
    pcie.ep[0x18 / 4] = EP_BAR_TYPE;             // BAR2 lo: 2 MiB TCM
    pcie.ep[0x2C / 4] = 0x435614E4;              // subsystem 14E4:4356
    pcie.ep[0x34 / 4] = 0x00000040;              // capabilities pointer
    pcie.ep[0x3C / 4] = 0x00000100;              // INTA
    pcie.ep[0x40 / 4] = 0x48030001;              // PM cap, next 0x48
    pcie.ep[0x48 / 4] = 0x00020010;              // PCIe cap, endpoint, next 0
    pcie.ep[BRCM_BAR0_WINDOW / 4] = BRCM_SI_ENUM_BASE;
}

bool radio_fitted(EmuState *state) {
    return state && state->wifi_radio.load() != WIFI_RADIO_ABSENT;
}

bool radio_core_alive(EmuState *state) {
    return state && state->wifi_radio.load() == WIFI_RADIO_HEALTHY;
}

// Every condition real silicon imposes before root port `port` can train.
// Returns nullptr when the link may come up, or the name of the first thing
// still missing. Port 0's x4 group goes nowhere on this board, so it is
// always "no endpoint" -- which is exactly what the payload should conclude.
// True when the PCIe apertures are reachable at all. See rst_v above: on
// hardware this is the difference between a register read and a dead
// console, so it is checked before every single access rather than folded
// into the link-training preconditions.
// Everything this emulator executes is BPMP code - Unicorn runs the ARM7,
// there is no CCPLEX model - and the BPMP cannot reach PCIe on real silicon.
// Measured on an Erista and a Mariko: with clocks, resets, power and PLLE all
// correct, the AFI window reads 0xFFFFFFFF and every offset of MSELECT reads
// one constant. The TRM agrees (ch.19: BPMP-Lite is an AHB master, PCIe is
// not an AHB slave; ch.16: MSELECT is CPU-complex hardware), and so does the
// T210 block diagram, where MSelect's only master is CCPLEX/CPUCIF.
//
// This model used to let the BPMP read and write these apertures happily,
// which is how a probe that cannot work on hardware passed here for days.
// Refusing them is the single most valuable thing this file does.
static bool bpmp_pcie_denied(uint64_t addr) {
    static bool warned = false;
    if (!warned) {
        warned = true;
        printf("[pcie] BPMP access to %08X denied: PCIe and MSELECT are\n",
               (unsigned)addr);
        printf("[pcie] behind the CPU complex and are NOT reachable from the\n");
        printf("[pcie] BPMP on real hardware. Use the CPU0 stub handoff.\n");
        fflush(stdout);
    }
    return true;
}

bool mselect_up() {
    return (pcie.rst_v & (1u << CLK_V_MSELECT)) == 0;
}

void mselect_complain(uint64_t addr) {
    if (pcie.mselect_warned)
        return;
    pcie.mselect_warned = true;
    printf("[pcie] *** access to %08X with MSELECT still in reset ***\n",
           (unsigned)addr);
    printf("[pcie] *** on real silicon this STALLS THE BPMP for good: no ***\n");
    printf("[pcie] *** abort, no timeout, no reboot path. Clear         ***\n");
    printf("[pcie] *** RST_DEV_V bit %d first (bdk ccplex.c:121).        ***\n",
           CLK_V_MSELECT);
    fflush(stdout);
}

const char *link_blocked(EmuState *state, int port) {
    if (!mselect_up())                               return "MSELECT still in reset";
    if (pcie.powergated)                             return "PCIE partition still gated";
    if (pcie.rst_u & (1u << CLK_U_PCIE))             return "PCIE still in reset";
    if (pcie.rst_u & (1u << CLK_U_AFI))              return "AFI still in reset";
    if (pcie.rst_u & (1u << CLK_U_PCIEXCLK))         return "PCIEXCLK still in reset";
    if (!(pcie.enb_u & (1u << CLK_U_AFI)))           return "AFI clock disabled";
    if (!pcie.pllrefe)                               return "PLLREFE not locked";
    if (!pcie.plle)                                  return "PLLE not locked";
    if (!(pcie.afi[AFI_CONFIGURATION / 4] & AFI_CONFIGURATION_EN_FPCI))
                                                     return "AFI FPCI not enabled";
    if (pcie.afi[AFI_PCIE_CONFIG / 4] & AFI_PCIE_CONFIG_DISABLE(port))
                                                     return "root port disabled in AFI_PCIE_CONFIG";
    if (pcie.rst_w & (1u << CLK_W_XUSB_PADCTL))      return "XUSB_PADCTL still in reset";
    if (pcie.rst_y & (1u << CLK_Y_PEX_USB_UPHY))     return "PEX_USB_UPHY still in reset";
    if (!pcie.uphy_locked)                           return "UPHY PLL P0 not locked";
    if (!pcie.uphy_rcal_done)                        return "UPHY resistor calibration not run";

    // Lane pcie-0 carries the x1 link to the radio: 2 bits at shift 12 must
    // select pcie-x1 (0) and its IDDQ-disable bit (1) must be set.
    uint32_t mux = pcie.padctl[PADCTL_USB3_PAD_MUX / 4];
    if (port == 1) {
        if (((mux >> 12) & 3u) != 0)                 return "lane pcie-0 not muxed to pcie-x1";
        if (!(mux & (1u << 1)))                      return "lane pcie-0 still in IDDQ";
    }

    const RootPort &rp = pcie.rp[port];
    if (!rp.rst_us)                                  return "PERST# still asserted";
    if (!rp.refclk_us)                               return "reference clock never enabled";
    if (rp.rst_us < rp.refclk_us + T_REFCLK_STABLE_US)
        return "PERST# released before Trefclkstable (10 ms)";

    if (port != 1)                                   return "no endpoint on this port";
    if (!radio_fitted(state))                        return "no WLAN module fitted";
    if (!pcie.wl_on)                                 return "WL_REG_ON low";
    if (state->emu_usec < pcie.wl_on_us + T_VDD_TO_POR_US)
        return "endpoint still inside Tvddtopor (57 ms)";
    return nullptr;
}

// Re-evaluate on every root-port read: the payload polls DL_UP in a loop, so
// this is where emulated time gets a chance to satisfy the timing gates.
void link_update(EmuState *state, int port) {
    RootPort &rp = pcie.rp[port];
    const char *why = link_blocked(state, port);
    bool up = (why == nullptr);
    if (up == rp.link)
        return;
    rp.link = up;
    if (up) {
        pcie.ep_present = true;
        printf("[pcie] root port %d: link UP, gen1 x1 (%s WLAN)\n", port,
               wifi_radio_name(state->wifi_radio.load()));
    } else {
        printf("[pcie] root port %d: link down\n", port);
    }
    fflush(stdout);
}

// Print the first blocking reason once per distinct reason, so a payload that
// gets the sequence wrong is told what it got wrong instead of just seeing a
// dead link.
void link_report_block(EmuState *state, int port) {
    const char *why = link_blocked(state, port);
    if (!why || why == pcie.rp[port].last_block)
        return;
    pcie.rp[port].last_block = why;
    printf("[pcie] root port %d cannot train: %s\n", port, why);
    fflush(stdout);
}

// Is the endpoint reachable for a config or memory access right now?
bool ep_reachable(EmuState *state) {
    if (!pcie.rp[1].link || !pcie.ep_present)
        return false;
    // The root port is a bridge: without a secondary bus number it never
    // turns a type-1 access into a type-0 TLP downstream.
    uint32_t bus = pcie.rp[1].reg[0x18 / 4];
    (void)state;
    return ((bus >> 8) & 0xFF) == 1;
}

} // namespace

// ==========================================================================

void pcie_reset(EmuState *state) {
    pcie = PcieModel();
    ep_init();
    (void)state;
}

void pcie_set_powergate(bool ungated) {
    if (pcie.powergated == !ungated)
        return;
    pcie.powergated = !ungated;
    printf("[pcie] PCIE power partition %s\n", ungated ? "ungated" : "gated");
    fflush(stdout);
}

void pcie_wl_reg_on(EmuState *state, bool level) {
    if (level == pcie.wl_on)
        return;
    pcie.wl_on = level;
    if (level) {
        pcie.wl_on_us = state->emu_usec;
    } else {
        // Section dropped: the link cannot survive it.
        pcie.rp[1].link = false;
        pcie.ep_present = false;
    }
    printf("[pcie] WL_REG_ON %s (%s WLAN)\n", level ? "HIGH - POR starts" : "LOW",
           wifi_radio_name(state->wifi_radio.load()));
    fflush(stdout);
}

// ---- clock and reset controller -----------------------------------------

void pcie_car_write(uint32_t offset, uint32_t val) {
    switch (offset) {
    case CAR_RST_DEVICES_U: pcie.rst_u  =  val; break;
    case CAR_CLK_OUT_ENB_U: pcie.enb_u  =  val; break;
    case CAR_RST_DEV_U_SET: pcie.rst_u |=  val; break;
    case CAR_RST_DEV_U_CLR: pcie.rst_u &= ~val; break;
    case CAR_CLK_ENB_U_SET: pcie.enb_u |=  val; break;
    case CAR_CLK_ENB_U_CLR: pcie.enb_u &= ~val; break;
    case CAR_RST_DEV_Y_SET: pcie.rst_y |=  val; break;
    case CAR_RST_DEV_Y_CLR: pcie.rst_y &= ~val; break;
    case CAR_PLLP_MISC1:          pcie.pllp_misc1 = val; break;
    case CAR_PEX_SATA_USB_RX_BYP: pcie.uphy_mgmt  = val; break;
    case CAR_RST_DEVICES_V: pcie.rst_v  =  val; break;
    case CAR_RST_DEV_V_SET: pcie.rst_v |=  val; break;
    case CAR_RST_DEV_V_CLR: pcie.rst_v &= ~val; break;
    case CAR_RST_DEV_W_SET: pcie.rst_w |=  val; break;
    case CAR_RST_DEV_W_CLR: pcie.rst_w &= ~val; break;
    case CAR_PLLE_BASE:     pcie.plle    = (val & PLLE_BASE_ENABLE) != 0; break;
    case CAR_PLLREFE_BASE:  pcie.pllrefe = (val & PLLREFE_BASE_ENABLE) != 0; break;
    default: break;
    }
}

// PLLE and PLLREFE do not follow the generic "ENABLE bit 30, LOCK bit 27 in
// _BASE" shape the rest of the CAR model assumes: PLLE enables on _BASE bit
// 31 and reports lock in _MISC bit 11, PLLREFE enables on _BASE bit 30 and
// reports lock in _MISC bit 27. Left to the generic path both polls would
// spin forever.
bool pcie_car_read(uint32_t offset, uint32_t *out) {
    if (offset == CAR_PLLE_MISC) {
        *out = (*out & ~PLLE_MISC_LOCK) | (pcie.plle ? PLLE_MISC_LOCK : 0);
        return true;
    }
    if (offset == CAR_PLLREFE_MISC) {
        *out = (*out & ~PLLREFE_MISC_LOCK) | (pcie.pllrefe ? PLLREFE_MISC_LOCK : 0);
        return true;
    }
    // The RST_DEVICES_* banks must read back the truth, because "is this
    // module still in reset" is the difference between a register read and a
    // permanently hung console, and a payload has no other way to ask. These
    // used to read 0 unconditionally, i.e. "nothing is in reset", which is
    // both false at RCM entry and the most dangerous possible lie: it is
    // exactly what let a padctl-read-before-ungate bug pass here and then
    // kill a real Mariko.
    if (offset == CAR_PLLP_MISC1)          { *out = pcie.pllp_misc1; return true; }
    if (offset == CAR_PEX_SATA_USB_RX_BYP) { *out = pcie.uphy_mgmt;  return true; }
    if (offset == CAR_RST_DEVICES_V) { *out = pcie.rst_v; return true; }
    if (offset == CAR_RST_DEVICES_W) { *out = pcie.rst_w; return true; }
    if (offset == CAR_RST_DEVICES_Y) { *out = pcie.rst_y; return true; }
    return false;
}

// ---- MSELECT -------------------------------------------------------------

uint32_t mselect_read(EmuState *state, uint64_t addr) {
    (void)state;
    // Same reasoning as pcie_read: MSELECT is CPU-complex hardware. The BPMP
    // sees one constant at every offset; 0xEAFFFFFE is what a real Erista
    // returned, so return that rather than a tidier lie.
    if (bpmp_pcie_denied(addr))
        return 0xEAFFFFFE;
    // Reading MSELECT while it is held in reset is the exact access that
    // killed a console: it never completes. A payload that does it here gets
    // told, loudly, instead of getting a plausible-looking zero.
    if (!mselect_up()) {
        mselect_complain(addr);
        return 0xFFFFFFFF;
    }
    return pcie.mselect[((uint32_t)(addr - MSELECT_BASE) & 0xFFC) / 4];
}

void mselect_write(EmuState *state, uint64_t addr, uint32_t val) {
    (void)state;
    uint32_t off = (uint32_t)(addr - MSELECT_BASE) & 0xFFC;
    // Held in reset, MSELECT latches nothing - which is exactly the state a
    // payload is in when it thinks it configured the fabric and did not.
    if (!mselect_up()) {
        mselect_complain(addr);
        return;
    }
    pcie.mselect[off / 4] = val;
}

// ---- XUSB pad controller -------------------------------------------------

uint32_t padctl_read(EmuState *state, uint64_t addr) {
    (void)state;
    // Same rule as MSELECT, and measured the same way: XUSB_PADCTL held in
    // reset does not error a read, it never completes one. A diagnostic dump
    // that read ELPG_PROGRAM before clearing RST_DEV_W bit 14 hung a Mariko.
    if (pcie.rst_w & (1u << CLK_W_XUSB_PADCTL)) {
        if (!pcie.padctl_warned) {
            pcie.padctl_warned = true;
            printf("[pcie] *** read of XUSB_PADCTL %08X with the block still ***\n",
                   (unsigned)addr);
            printf("[pcie] *** in reset. On real silicon this STALLS THE     ***\n");
            printf("[pcie] *** BPMP for good. Clear RST_DEV_W bit %d first.   ***\n",
                   CLK_W_XUSB_PADCTL);
            fflush(stdout);
        }
        return 0xFFFFFFFF;
    }
    uint32_t off = (uint32_t)(addr - XUSB_PADCTL_BASE) & 0xFFC;
    return pcie.padctl[off / 4];
}

void padctl_write(EmuState *state, uint64_t addr, uint32_t val) {
    (void)state;
    uint32_t off = (uint32_t)(addr - XUSB_PADCTL_BASE) & 0xFFC;

    if (pcie.rst_w & (1u << CLK_W_XUSB_PADCTL)) {
        // Held in reset, writes do not stick. This is the trap a payload
        // falls into if it programs the lane mux before ungating padctl.
        return;
    }

    switch (off) {
    case PADCTL_UPHY_PLL_P0_CTL2: {
        // Frequency calibration completes as soon as CAL_EN is raised, and
        // CAL_DONE only drops once CAL_EN is lowered again -- both edges are
        // mandatory in the real sequence, and a payload that waits for only
        // one of them hangs on hardware.
        uint32_t v = val & ~UPHY_CTL2_CAL_DONE;
        if (val & UPHY_CTL2_CAL_EN) {
            // On T210B01 the calibration FSM runs off the "uphy_mgmt" clock.
            // Gated, CAL_EN latches and CAL_DONE never follows -- measured on
            // a real Mariko, and invisible to every T210-only reference
            // driver. Erista has no such clock, so it calibrates regardless.
            bool mariko = state && state->is_mariko.load();
            bool mgmt = (pcie.pllp_misc1 & PLLP_MISC1_XUSB_EN) &&
                        (pcie.uphy_mgmt & UPHY_MGMT_CLK_EN);
            // B01 also needs the brick's own config table loaded through
            // CTL10. Without it CAL_EN latches and CAL_DONE never follows -
            // exactly what a real Mariko does, and the reason a sequence
            // that works on every T210 silently dies here.
            if (mariko && !pcie.uphy_brick_cfg && !pcie.brick_warned) {
                pcie.brick_warned = true;
                printf("[pcie] UPHY CAL_EN on Mariko without the T210B01 PLL\n");
                printf("[pcie] table loaded through UPHY_PLL_P0_CTL10 (0x384).\n");
                printf("[pcie] CAL_DONE will never assert. L4T writes\n");
                printf("[pcie] usb3_pll_g1_init_data there INSTEAD of\n");
                printf("[pcie] CAL_CTRL/DCO_CTRL - it is an either/or.\n");
                fflush(stdout);
            }
            if (!mariko || (mgmt && pcie.uphy_brick_cfg)) {
                v |= UPHY_CTL2_CAL_DONE;
                pcie.uphy_cal_ran = true;
            } else if (!pcie.mgmt_warned) {
                pcie.mgmt_warned = true;
                printf("[pcie] UPHY CAL_EN raised on Mariko with the uphy_mgmt\n");
                printf("[pcie] clock gated: CAL_DONE will never assert. Enable\n");
                printf("[pcie] PLLP_MISC1(0x680) bit 28 and set the divider +\n");
                printf("[pcie] gate bit 8 in PEX_SATA_USB_RX_BYP(0x6D0) first.\n");
                fflush(stdout);
            }
        }
        pcie.padctl[off / 4] = v;
        return;
    }
    case PADCTL_UPHY_PLL_P0_CTL8: {
        uint32_t v = val & ~UPHY_CTL8_RCAL_DONE;
        if (val & UPHY_CTL8_RCAL_EN) {
            v |= UPHY_CTL8_RCAL_DONE;
            pcie.uphy_rcal_done = true;
        }
        pcie.padctl[off / 4] = v;
        return;
    }
    case PADCTL_UPHY_PLL_P0_CTL1: {
        uint32_t v = val & ~UPHY_CTL1_LOCKDET;
        // The PLL only locks with IDDQ released, ENABLE set, and frequency
        // calibration already run to completion. Anything else leaves
        // LOCKDET clear and the payload's poll times out -- which is what
        // real silicon does too.
        bool cal = pcie.uphy_cal_ran &&
                   !(pcie.padctl[PADCTL_UPHY_PLL_P0_CTL2 / 4] & UPHY_CTL2_CAL_DONE);
        if ((val & UPHY_CTL1_ENABLE) && !(val & UPHY_CTL1_IDDQ) && cal) {
            v |= UPHY_CTL1_LOCKDET;
            if (!pcie.uphy_locked) {
                pcie.uphy_locked = true;
                printf("[pcie] UPHY PLL P0 locked\n");
                fflush(stdout);
            }
        } else if (!(val & UPHY_CTL1_ENABLE)) {
            pcie.uphy_locked = false;
        }
        pcie.padctl[off / 4] = v;
        return;
    }
    case PADCTL_UPHY_PLL_P0_CTL10:
        // Any write with the write-strobe set counts as the brick being
        // programmed; the exact table contents are NVIDIA's business.
        if (val & (1u << 24))
            pcie.uphy_brick_cfg = true;
        pcie.padctl[off / 4] = val;
        return;

    default:
        pcie.padctl[off / 4] = val;
        return;
    }
}

// ---- PCIe apertures ------------------------------------------------------

static uint32_t afi_read(EmuState *state, uint32_t off) {
    (void)state;
    return pcie.afi[(off & 0x7FC) / 4];
}

static void afi_write(EmuState *state, uint32_t off, uint32_t val) {
    off &= 0x7FC;
    uint32_t prev = pcie.afi[off / 4];
    pcie.afi[off / 4] = val;

    if (off == AFI_PEX0_CTRL || off == AFI_PEX1_CTRL) {
        int port = (off == AFI_PEX1_CTRL) ? 1 : 0;
        RootPort &rp = pcie.rp[port];
        if (!(prev & AFI_PEX_CTRL_REFCLK_EN) && (val & AFI_PEX_CTRL_REFCLK_EN))
            rp.refclk_us = state->emu_usec ? state->emu_usec : 1;
        if (!(val & AFI_PEX_CTRL_REFCLK_EN))
            rp.refclk_us = 0;
        if (!(prev & AFI_PEX_CTRL_RST) && (val & AFI_PEX_CTRL_RST))
            rp.rst_us = state->emu_usec ? state->emu_usec : 1;
        if (!(val & AFI_PEX_CTRL_RST)) {
            rp.rst_us = 0;
            rp.link = false;
        }
        link_report_block(state, port);
    }
}

uint32_t pcie_read(EmuState *state, uint64_t addr) {
    if (bpmp_pcie_denied(addr))
        return 0xFFFFFFFF;

    if (!mselect_up()) {
        mselect_complain(addr);
        return 0xFFFFFFFF;
    }

    // Root port register windows, 0x01000000 and 0x01001000.
    if (addr >= PCIE_RP0_BASE && addr < PCIE_RP0_BASE + 0x2000) {
        int port = (addr >= PCIE_RP1_BASE) ? 1 : 0;
        uint32_t off = (uint32_t)(addr & 0xFFC);
        link_update(state, port);
        RootPort &rp = pcie.rp[port];

        switch (off) {
        case 0x00:  // NVIDIA root port, one device ID per port
            return port ? 0x0FAF10DE : 0x0FAE10DE;
        case 0x08:  // PCI-to-PCI bridge, revision A1
            return 0x060400A1;
        case 0x0C:  // header type 1
            return 0x00010000;
        case RP_VEND_XP:
            return rp.reg[off / 4] | (rp.link ? RP_VEND_XP_DL_UP : 0);
        case RP_LINK_CONTROL_STATUS:
            // The CYW4356 is a Gen1 x1 endpoint even with Gen2 enabled on
            // the root port (datasheet s10.2: "PCIe v3.0 ... running at Gen1
            // speeds"), so speed = 1 and width = 1.
            if (!rp.link)
                return rp.reg[off / 4] & 0xFFFF;
            return (rp.reg[off / 4] & 0xFFFF) | RP_LINK_DL_ACTIVE |
                   (1u << 16) | (1u << 20);
        default:
            return rp.reg[off / 4];
        }
    }

    if (addr >= PCIE_PADS_BASE && addr < PCIE_PADS_BASE + 0x800)
        return pcie.pads[((uint32_t)(addr - PCIE_PADS_BASE) & 0x7FC) / 4];

    if (addr >= PCIE_AFI_BASE && addr < PCIE_AFI_BASE + 0x800)
        return afi_read(state, (uint32_t)(addr - PCIE_AFI_BASE));

    // Type-1 extended configuration aperture. Only bus 1 device 0 exists,
    // and only once the root port has a secondary bus number and a link.
    if (addr >= PCIE_CS_BASE && addr < PCIE_CS_BASE + PCIE_CS_MODEL_SIZE) {
        uint32_t a = (uint32_t)(addr - PCIE_CS_BASE);
        uint32_t bus = (a >> 16) & 0xFF, dev = (a >> 11) & 0x1F;
        uint32_t fn = (a >> 8) & 0x7, reg = a & 0xFC;
        link_update(state, 1);
        if (bus != 1 || dev != 0 || fn != 0 || !ep_reachable(state))
            return 0xFFFFFFFF;          // unsupported request -> all ones
        return pcie.ep[reg / 4];
    }

    // BAR0 backplane window. This is the read that separates a live WLAN die
    // from a PCIe front-end whose radio is dead, so it is gated on the
    // endpoint actually having been given a memory window and MEM enable.
    if (addr >= PCIE_MEM_BASE && addr < PCIE_MEM_BASE + PCIE_MEM_MODEL_SIZE) {
        link_update(state, 1);
        if (!ep_reachable(state))
            return 0xFFFFFFFF;
        if (!(pcie.ep[0x04 / 4] & 0x2))                 // MEM space disabled
            return 0xFFFFFFFF;
        if ((pcie.ep[0x10 / 4] & ~0xFu) != PCIE_MEM_BASE)  // BAR0 unassigned
            return 0xFFFFFFFF;
        if (!radio_core_alive(state))
            return 0xFFFFFFFF;          // front-end answers, the die does not

        uint32_t win = pcie.ep[BRCM_BAR0_WINDOW / 4] & ~0xFFFu;
        uint32_t bp = win + ((uint32_t)(addr - PCIE_MEM_BASE) & 0xFFF);
        if (bp == BRCM_SI_ENUM_BASE)
            return BRCM_4356_CHIPID;
        return 0;
    }

    return 0;
}

void pcie_write(EmuState *state, uint64_t addr, uint32_t val) {
    if (bpmp_pcie_denied(addr))
        return;

    if (!mselect_up()) {
        mselect_complain(addr);
        return;
    }

    if (addr >= PCIE_RP0_BASE && addr < PCIE_RP0_BASE + 0x2000) {
        int port = (addr >= PCIE_RP1_BASE) ? 1 : 0;
        pcie.rp[port].reg[(uint32_t)(addr & 0xFFC) / 4] = val;
        return;
    }

    if (addr >= PCIE_PADS_BASE && addr < PCIE_PADS_BASE + 0x800) {
        pcie.pads[((uint32_t)(addr - PCIE_PADS_BASE) & 0x7FC) / 4] = val;
        return;
    }

    if (addr >= PCIE_AFI_BASE && addr < PCIE_AFI_BASE + 0x800) {
        afi_write(state, (uint32_t)(addr - PCIE_AFI_BASE), val);
        return;
    }

    if (addr >= PCIE_CS_BASE && addr < PCIE_CS_BASE + PCIE_CS_MODEL_SIZE) {
        uint32_t a = (uint32_t)(addr - PCIE_CS_BASE);
        uint32_t bus = (a >> 16) & 0xFF, dev = (a >> 11) & 0x1F;
        uint32_t fn = (a >> 8) & 0x7, reg = a & 0xFC;
        if (bus != 1 || dev != 0 || fn != 0 || !ep_reachable(state))
            return;
        switch (reg) {
        case 0x04:                       // command: only the low half is RW
            pcie.ep[reg / 4] = (pcie.ep[reg / 4] & 0xFFFF0000u) | (val & 0xFFFF);
            return;
        case 0x00: case 0x08: case 0x2C: // read-only identity
            return;
        case 0x10: case 0x18: {          // BARs keep their type bits
            uint32_t mask = (reg == 0x10) ? EP_BAR0_MASK : EP_BAR2_MASK;
            pcie.ep[reg / 4] = (val & mask) | EP_BAR_TYPE;
            return;
        }
        default:
            pcie.ep[reg / 4] = val;
            return;
        }
    }
    // Writes into the BAR0 window are accepted and dropped: nothing this
    // payload does needs the backplane to be writable.
}
