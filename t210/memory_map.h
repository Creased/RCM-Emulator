#ifndef T210_MEMORY_MAP_H
#define T210_MEMORY_MAP_H

#include <cstdint>

/*
 * Tegra X1 (T210) Memory Map for RCM Payload Emulation
 *
 * RCM payloads (hekate, etc.) run ARM32 code on the BPMP.
 * They primarily use IRAM and access hardware via MMIO registers.
 */

// ==================== RAM Regions ====================

// IRAM (Internal SRAM) - 256KB
constexpr uint64_t IRAM_BASE = 0x40000000;
constexpr uint64_t IRAM_SIZE = (16 * 1024 * 1024); // 16 MB mapped (actual 512 KB, but hekate needs relocation space)

// DRAM - We allocate a modest amount for framebuffer + heap
constexpr uint64_t DRAM_BASE = 0x80000000;
constexpr uint64_t DRAM_SIZE = 0x80000000; // 2GB (Full T210 DRAM space for FatFs buffers)

// Payload load address within IRAM
constexpr uint64_t IPL_LOAD_ADDR = 0x40010000;

// Stack addresses used by hekate
constexpr uint64_t IPL_STACK_ADDR = 0x4003FF00;

// Heap (in DRAM)
constexpr uint64_t IPL_HEAP_START = 0x90000000;
constexpr uint64_t IPL_HEAP_SIZE  = 0x02000000; // 32MB

// ==================== MMIO Regions ====================
// All MMIO is in the 0x50000000-0x7FFFFFFF range on T210

// Clock and Reset Controller
constexpr uint64_t CLK_RST_BASE = 0x60006000;
constexpr uint64_t CLK_RST_SIZE = 0x1000;

// Timer (TMR)
constexpr uint64_t TMR_BASE = 0x60005000;
constexpr uint64_t TMR_SIZE = 0x1000;

// GPIO
constexpr uint64_t GPIO_BASE = 0x6000D000;
constexpr uint64_t GPIO_SIZE = 0x1000;

// PINMUX
constexpr uint64_t PINMUX_BASE = 0x70003000;
constexpr uint64_t PINMUX_SIZE = 0x1000;

// UART
constexpr uint64_t UART_A_BASE = 0x70006000;
constexpr uint64_t UART_SIZE   = 0x1000;

// I2C controllers
// Note: Tegra X1 base offsets per Hekate bdk/soc/i2c.c _i2c_base_offsets[]:
// I2C1=0x0, I2C2=0x400, I2C3=0x500, I2C4=0x700, I2C5=0x1000, I2C6=0x1100
// All measured from APB I2C base 0x7000C000.
constexpr uint64_t I2C1_BASE = 0x7000C000;
constexpr uint64_t I2C3_BASE = 0x7000C500; // GEN3_I2C, used by STMFTS touchscreen at slave 0x49
constexpr uint64_t I2C5_BASE = 0x7000D000;
constexpr uint64_t I2C_SIZE  = 0x1000;
constexpr uint64_t I2C3_SIZE = 0x100;

// PWM
constexpr uint64_t PWM_BASE = 0x7000A000;
constexpr uint64_t PWM_SIZE = 0x1000;

// RTC
constexpr uint64_t RTC_BASE = 0x7000E000;
constexpr uint64_t RTC_SIZE = 0x1000;

// PMC (Power Management Controller)
constexpr uint64_t PMC_BASE = 0x7000E400;
constexpr uint64_t PMC_SIZE = 0x0C00;

// Fuse Controller
constexpr uint64_t FUSE_BASE = 0x7000F800;
constexpr uint64_t FUSE_SIZE = 0x0400;

// EMC (External Memory Controller). EMC_BASE is the broadcast/control bank;
// EMC0/EMC1 are the per-channel banks used to read mode-register responses
// (sdram_read_mrx splits the 16-bit MRR into low byte = ch0, high byte = ch1).
constexpr uint64_t EMC_BASE  = 0x7001B000;
constexpr uint64_t EMC_SIZE  = 0x1000;
constexpr uint64_t EMC0_BASE = 0x7001E000;
constexpr uint64_t EMC1_BASE = 0x7001F000;

// MC (Memory Controller)
constexpr uint64_t MC_BASE = 0x70019000;
constexpr uint64_t MC_SIZE = 0x1000;

// Display Controller (DC/DSI)
constexpr uint64_t DISPLAY_A_BASE = 0x54200000;
constexpr uint64_t DISPLAY_SIZE   = 0x40000;

// DSI Controller
constexpr uint64_t DSI_BASE = 0x54300000;
constexpr uint64_t DSI_SIZE = 0x1000;

// MIPI CAL
constexpr uint64_t MIPI_CAL_BASE = 0x700E3000;
constexpr uint64_t MIPI_CAL_SIZE = 0x1000;

// SOR (Serial Output Resource) / HDMI
constexpr uint64_t SOR_BASE = 0x54540000;
constexpr uint64_t SOR_SIZE = 0x1000;

// SDMMC controllers
constexpr uint64_t SDMMC1_BASE = 0x700B0000; // SD Card
constexpr uint64_t SDMMC4_BASE = 0x700B0600; // eMMC
constexpr uint64_t SDMMC_SIZE  = 0x1000;

// APB Misc
constexpr uint64_t APB_MISC_BASE = 0x70000000;
constexpr uint64_t APB_MISC_SIZE = 0x1000;

// VIC (Video Image Compositor)
constexpr uint64_t VIC_BASE = 0x54340000;
constexpr uint64_t VIC_SIZE = 0x40000;

// Host1x (display sync)
constexpr uint64_t HOST1X_BASE = 0x50000000;
constexpr uint64_t HOST1X_SIZE = 0x40000;

// BPMP Cache
constexpr uint64_t BPMP_CACHE_BASE = 0x50040000;
constexpr uint64_t BPMP_CACHE_SIZE = 0x1000;

// System Registers (SYSREG)
constexpr uint64_t SYSREG_BASE = 0x6000C000;
constexpr uint64_t SYSREG_SIZE = 0x1000;

// I2S (Audio)
constexpr uint64_t I2S_BASE = 0x702D1000;
constexpr uint64_t I2S_SIZE = 0x1000;

// SE (Security Engine)
constexpr uint64_t SE_BASE = 0x70012000;
constexpr uint64_t SE_SIZE = 0x2000;

// TSEC (Falcon control regs are in the 0x1000–0x11FF window within TSEC_BASE)
constexpr uint64_t TSEC_BASE = 0x54500000;
constexpr uint64_t TSEC_SIZE = 0x2000;

// TZRAM
constexpr uint64_t TZRAM_BASE = 0x7C010000;
constexpr uint64_t TZRAM_SIZE = 0x10000; // 64KB

// System Registers (SYSCTR0)
constexpr uint64_t SYSCTR0_BASE = 0x700F0000;
constexpr uint64_t SYSCTR0_SIZE = 0x1000;

// ==================== Framebuffer ====================

// hekate uses a display window within DRAM for framebuffer
// The display init code sets up a linear framebuffer at predefined addresses.
// IPL_FB_ADDRESS is 0xF5A00000, NYX is 0xF6200000.
// We allocate a 16MB block covering 0xF5A00000 - 0xF6A00000
constexpr uint64_t FB_BASE = 0xF5A00000;
constexpr uint64_t FB_SIZE = 0x01000000; // 16MB

// Display dimensions (hekate: 720 width, 1280 height in portrait)
constexpr int DISPLAY_WIDTH  = 720;
constexpr int DISPLAY_HEIGHT = 1280;

// Landscape dimensions for SDL window
constexpr int WINDOW_WIDTH  = 720;
constexpr int WINDOW_HEIGHT = 1280;

#endif // T210_MEMORY_MAP_H
