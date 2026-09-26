#ifndef T210_PCIE_H
#define T210_PCIE_H

#include <cstdint>

struct EmuState;

/*
 * Tegra X1 PCIe root complex + the Broadcom CYW4356 WLAN endpoint that hangs
 * off root port 1 on a Nintendo Switch.
 *
 * Everything a BPMP payload can reach is here: the AFI and PADS register
 * banks (0x01003800 / 0x01003000), both root ports' 4 KiB config windows
 * (0x01000000 / 0x01001000), the type-1 extended config aperture
 * (0x02000000), the endpoint's BAR0 backplane window, and the UPHY PLL P0
 * calibration/lock state machine in the XUSB pad controller.
 *
 * The link only trains once the payload has satisfied the preconditions
 * this model checks -- power partition, AFI clock, resets, PLLE, UPHY, lane
 * mux, IDDQ, and the refclk/PERST#/POR timing of Table 62. A payload that
 * skips one gets a link that stays down and a `[pcie]` line on stdout naming
 * what it missed. Not every step of the TRM's bring-up (34.3) is checked:
 * the PCIe clock enable, PRSNT_MAP, PEX_BIAS_PWRD, the PEX_L1_RST_N pinmux
 * and the AUX_MUX_LP0 clamps are not, so a sequence that leaves one out can
 * train here and not on silicon.
 */

void     pcie_reset(EmuState *state);

// 0x01000000..0x01003FFF (root ports, PADS, AFI), the 0x02000000 config
// aperture and the 0x13000000 non-prefetchable memory window.
uint32_t pcie_read(EmuState *state, uint64_t addr);
void     pcie_write(EmuState *state, uint64_t addr, uint32_t val);

// MSELECT, 0x50060000.
uint32_t mselect_read(EmuState *state, uint64_t addr);
void     mselect_write(EmuState *state, uint64_t addr, uint32_t val);

// XUSB pad controller, 0x7009F000. Lane mux, IDDQ and UPHY PLL P0.
uint32_t padctl_read(EmuState *state, uint64_t addr);
void     padctl_write(EmuState *state, uint64_t addr, uint32_t val);

// Clock-and-reset forwarding. The PCIe model needs PLLE/PLLREFE lock and the
// PCIE/AFI/PCIEXCLK/UPHY reset+enable bits, all of which live in CAR.
// pcie_car_read returns true when it wants to override the CAR model's answer.
void     pcie_car_write(EmuState *state, uint32_t offset, uint32_t val);
bool     pcie_car_read(uint32_t offset, uint32_t *out);

// MSELECT clocked out of reset (RST_DEV_V bit 3 clear). The CCPLEX needs it
// as much as PCIe does: it is the CPU cluster's path to every slave.
bool     pcie_mselect_up();

// The PCIE power partition, toggled through PMC_PWRGATE_TOGGLE.
void     pcie_set_powergate(bool ungated);

// WL_REG_ON (GPIO PH1) changed level. The rising edge starts the endpoint's
// internal power-on reset, i.e. t=0 for the datasheet's timing budget.
void     pcie_wl_reg_on(EmuState *state, bool level);

#endif // T210_PCIE_H
