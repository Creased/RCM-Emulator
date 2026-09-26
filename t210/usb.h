#pragma once

#include <cstdint>

struct EmuState;

// USB device mode (see usb.cpp): the Erista USB2 device controller, and the
// PC a --usb-host run plugs into it.
uint32_t usb2d_read(EmuState *state, uint64_t addr, unsigned size);
void usb2d_write(EmuState *state, uint64_t addr, unsigned size, uint32_t val);

// The Mariko XUSB device controller (XHCI registers, PCI config at +0x8000,
// the device block at +0x9000).
uint32_t xusbd_read(EmuState *state, uint64_t addr, unsigned size);
void xusbd_write(EmuState *state, uint64_t addr, unsigned size, uint32_t val);

// SoC reset: the controller back to its reset state, the host unplugged.
void usb_reset(EmuState *state);
