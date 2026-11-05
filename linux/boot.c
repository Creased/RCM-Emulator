/*
 * BPMP side of booting a Linux kernel on the emulator's CPU0.
 *
 * run.sh has the emulator place the kernel Image at 0x80080000 and the
 * device tree at 0x83000000 (--load), as a bootloader would have. This
 * payload powers CPU0 the way bdk's ccplex_boot_cpu0() does (Erista rail),
 * releases it at shim.S - which drops to EL1 and enters the kernel - and
 * then halts the BPMP for good, so every emulated cycle goes to CPU0.
 */

typedef unsigned int u32;

#define REG(a) (*(volatile u32 *)(a))

#define TIMERUS     0x60005010u
#define CAR         0x60006000u
#define FLOW        0x60007000u
#define SB          0x6000C200u
#define UARTB_THR   0x70006040u
#define PMC         0x7000E400u
#define I2C5        0x7000D000u

#define CPU_ENTRY   0xA0000000u
#define KERNEL      0x80080000u

static void putc_(char c) { REG(UARTB_THR) = (u32)(unsigned char)c; }

static void puts_(const char *s) {
    while (*s)
        putc_(*s++);
}

static void puthex(u32 v) {
    for (int i = 28; i >= 0; i -= 4)
        putc_("0123456789ABCDEF"[(v >> i) & 0xF]);
}

static void udelay(u32 us) {
    u32 start = REG(TIMERUS);
    while (REG(TIMERUS) - start < us)
        ;
}

/* bdk-style normal-mode I2C write: [reg, val] in CMD_DATA1, 2 bytes, GO. */
static void i2c5_write(u32 dev, u32 reg, u32 val) {
    REG(I2C5 + 0x04) = dev << 1;                 /* CMD_ADDR0 */
    REG(I2C5 + 0x0C) = reg | (val << 8);         /* CMD_DATA1 */
    REG(I2C5 + 0x00) = (1u << 1) | (1u << 9);    /* CNFG: 2 bytes, write, GO */
}

static void pwrgate_on(u32 part) {
    if (!(REG(PMC + 0x38) & (1u << part)))       /* PWRGATE_STATUS */
        REG(PMC + 0x30) = part | (1u << 8);      /* PWRGATE_TOGGLE */
}

#define RST_CPU0 ((1u << 30) | (1u << 24) | (1u << 16) | (1u << 0))
#define RST_NONCPU (1u << 29)

static void release_cpu0(void) {
    REG(SB + 0x30) = CPU_ENTRY | 1u;             /* SB_AA64_RESET_LOW */
    REG(SB + 0x34) = 0;                          /* SB_AA64_RESET_HIGH */
    REG(CAR + 0x454) = RST_NONCPU;               /* RST_CPUG_CMPLX_CLR */
    REG(CAR + 0x454) = RST_CPU0;
}

static void hold_cpu0(void) {
    REG(CAR + 0x450) = RST_CPU0;                 /* RST_CPUG_CMPLX_SET */
    REG(CAR + 0x450) = RST_NONCPU;
}



#include "shim_blob.h"

__attribute__((section(".text.start"), noreturn))
void _start(void) {
    puts_("linux boot: kernel header ");
    puthex(((volatile u32 *)KERNEL)[14]);        /* 'ARM\x64' at offset 0x38 */
    putc_('\n');
    for (u32 i = 0; i < sizeof(shim_blob) / sizeof(shim_blob[0]); i++)
        ((volatile u32 *)CPU_ENTRY)[i] = shim_blob[i];

    /* The clocks a bootloader has set before any OS runs: bdk hw_init()'s
     * _config_oscillators(), as hekate leaves them for switchroot's chain.
     * The kernel's super-clock code BUG()s on a burst policy still in
     * STANDBY, which is what these registers hold until someone writes
     * them. */
    REG(CAR + 0x55C) = (REG(CAR + 0x55C) & 0xFFFFFFF3) | 4;   /* CLK_M divisor 2 */
    REG(0x700F0000 + 0x20) = 19200000;           /* SYSCTR0_CNTFID0 */
    REG(0x60005014) = 0x45F;                     /* TIMERUS_USEC_CFG, 19.2 MHz */
    REG(CAR + 0x050) = 0x50000071;               /* OSC_CTRL: 38.4 MHz */
    REG(CAR + 0x030) = 0x10;                     /* CLK_SYSTEM_RATE */
    REG(CAR + 0x400) = 0;                        /* CLK_SOURCE_SYS */
    REG(CAR + 0x028) = 0x20004444;               /* SCLK_BURST_POLICY: run, PLLP_OUT2 */
    REG(CAR + 0x02C) = 0x80000000;               /* SUPER_SCLK_DIVIDER */
    REG(CAR + 0x030) = 2;                        /* HCLK /1, PCLK /3 */
    REG(CAR + 0x370) = 0x20000000;               /* CCLKLP_BURST_POLICY: run, clk_m */

    i2c5_write(0x3C, 0x40, 0x1C);                /* MAX77620 AME_GPIO: GPIO5 plain */
    i2c5_write(0x3C, 0x3B, 0x09);                /* GPIO5: push-pull, out high */
    i2c5_write(0x1B, 0x00, 0x80 | 55);           /* MAX77621 VOUT: on, 0.950 V */
    REG(CAR + 0x0E0) = (1u << 30) | (2u << 20) | (156u << 8) | 2u;  /* PLLX */
    REG(CAR + 0x3B4) = 4u << 1;                  /* CLK_SOURCE_MSELECT */
    REG(CAR + 0x440) = 1u << 3;                  /* CLK_ENB_V_SET: MSELECT */
    REG(CAR + 0x020) = 0x20008888;               /* CCLK_BURST: PLLX_OUT0_LJ */
    REG(CAR + 0x024) = 1u << 31;                 /* SUPER_CCLK_DIVIDER */
    REG(CAR + 0x440) = 1u << 0;                  /* CLK_ENB_V_SET: CPUG */
    pwrgate_on(0);                               /* CRAIL */
    pwrgate_on(15);                              /* C0NC */
    pwrgate_on(14);                              /* CE0 */
    REG(FLOW + 0x40) = 1;                        /* RAM_REPAIR_REQ */
    while (!(REG(FLOW + 0x40) & 2))
        ;
    REG(CAR + 0x434) = 1u << 3;                  /* RST_DEV_V_CLR: MSELECT */
    release_cpu0();
    puts_("linux boot: CPU0 released, BPMP halting\n");

    /* bdk's bpmp_halt(): WAITEVENT | JTAG, no wake source. */
    REG(FLOW + 0x04) = (2u << 29) | (1u << 28);
    for (;;)
        ;
}
