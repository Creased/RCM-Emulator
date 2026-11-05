#ifndef T210_GIC_H
#define T210_GIC_H

#include <cstdint>

/*
 * GIC-400 - the interrupt controller in front of the CCPLEX (Tegra X1 TRM
 * ch.3; ARM IHI 0048B, GICv2). Distributor at 0x50041000, CPU interface at
 * 0x50042000 (8 KiB: GICC_DIR is at +0x1000).
 *
 * One CPU interface is modelled: CPU0 is the only core the emulator runs.
 * Security is not: both groups behave as one, which is all a kernel booted
 * non-secure ever sees. Peripherals drive level lines (gic_set_spi); SGIs
 * and edge-configured interrupts latch.
 */

constexpr uint64_t GICD_BASE = 0x50041000;
constexpr uint64_t GICD_SIZE = 0x1000;
constexpr uint64_t GICC_BASE = 0x50042000;
constexpr uint64_t GICC_SIZE = 0x2000;

constexpr int GIC_NR_IRQS = 256;    // 32 private + 224 SPIs (ITLinesNumber 7)
constexpr int GIC_PPI_VTIMER = 27;  // EL1 virtual timer
constexpr int GIC_PPI_PTIMER = 30;  // EL1 physical (non-secure) timer

void     gic_reset();
uint32_t gic_dist_read(uint32_t off, unsigned size);
void     gic_dist_write(uint32_t off, unsigned size, uint32_t val);
uint32_t gic_cpu_read(uint32_t off, unsigned size);
void     gic_cpu_write(uint32_t off, unsigned size, uint32_t val);

// A peripheral's interrupt line. `spi` is the SPI number as device trees
// write it (<GIC_SPI n ...>); the interrupt ID is 32 + n.
void gic_set_spi(int spi, bool level);
// A private peripheral interrupt line (ID 16..31) of CPU0.
void gic_set_ppi(int id, bool level);

// CPU0's nIRQ: something enabled, pending, not active and above the
// priority mask and the running priority.
bool gic_irq_pending();

// For the log: the interrupt ID that IRQ would deliver, or 1023.
int  gic_highest_pending();

// One line of the distributor and CPU interface state for interrupt `id`.
void gic_describe(int id, char *buf, unsigned n);

#endif // T210_GIC_H
