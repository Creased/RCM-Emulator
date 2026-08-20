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
 * The link only trains when the payload has satisfied every precondition
 * real silicon and the CYW4356 datasheet impose -- power partition, clocks,
 * resets, PLLE, UPHY, lane mux, IDDQ, and the refclk/PERST#/POR timing of
 * Table 62. A payload that skips one gets a link that stays down and a
 * `[pcie]` line on stdout naming what it missed.
 */

void     pcie_reset(EmuState *state);

// 0x01000000..0x01003FFF (root ports, PADS, AFI), 0x02000000 config
// aperture, and the 0x13000000 BAR0 memory window.
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
void     pcie_car_write(uint32_t offset, uint32_t val);
bool     pcie_car_read(uint32_t offset, uint32_t *out);

// The PCIE power partition, toggled through PMC_PWRGATE_TOGGLE.
void     pcie_set_powergate(bool ungated);

// WL_REG_ON (GPIO PH1) changed level. The rising edge starts the endpoint's
// internal power-on reset, i.e. t=0 for the datasheet's timing budget.
void     pcie_wl_reg_on(EmuState *state, bool level);

#endif // T210_PCIE_H
