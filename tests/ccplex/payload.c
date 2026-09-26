/*
 * CCPLEX regression payload.
 *
 * A freestanding BPMP payload (ARM mode, loaded at 0x40010000 like any RCM
 * payload) that drives the emulator's CPU0 model through the same register
 * sequence as bdk's ccplex_boot_cpu0(), and reports what happened on UART-B.
 * tests/ccplex/run.sh checks the emulator's log for the expected lines.
 *
 *   1. Release CPU0 from reset with nothing powered. The core must NOT run,
 *      and the emulator must say which precondition is missing.
 *   2. Do the real bring-up - CPU rail (MAX77620 GPIO5 + MAX77621 VOUT on
 *      I2C5), CRAIL/C0NC/CE0 partitions, CPUG + MSELECT clocks, MSELECT out
 *      of reset, PLLX, RAM repair, SB_AA64_RESET_LOW - then release it. An
 *      AArch64 blob copied to DRAM records CurrentEL and a TIMERUS delta in
 *      a mailbox, writes a magic word last, and parks in WFE.
 *      Released first with its clock stopped (CLK_CPUG_CMPLX), it must
 *      stay still until the stop is cleared.
 *   3. Put CPU0 back into reset and power the console off.
 *
 * Built by `make test` with arm-none-eabi-gcc; no other toolchain needed, as
 * the AArch64 side is a handful of pre-assembled words.
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
#define MBOX        0xA0030000u
#define PCIE_MAGIC  0x50434945u

/* AArch64, entered at EL3 with the MMU off:
 *   mov  w1, #0xa0030000        ; mailbox
 *   mrs  x8, currentel
 *   str  w8, [x1, #4]           ; [1] = CurrentEL
 *   mov  w4, #0x5010
 *   movk w4, #0x6000, lsl #16   ; TIMERUS
 *   ldr  w5, [x4]
 *   mov  w6, #2000
 * 1:subs w6, w6, #1
 *   b.ne 1b
 *   ldr  w7, [x4]
 *   sub  w7, w7, w5
 *   str  w7, [x1, #8]           ; [2] = TIMERUS delta across the loop
 *   mov  w2, #0x4945
 *   movk w2, #0x5043, lsl #16
 *   str  w2, [x1]               ; [0] = 'PCIE', written last
 * 2:wfe
 *   b    2b
 */
static const u32 cpu0_blob[] = {
    0x52b40061, 0xd5384248, 0xb9000428, 0x528a0204, 0x72ac0004, 0xb9400085,
    0x5280fa06, 0x710004c6, 0x54ffffe1, 0xb9400087, 0x4b0500e7, 0xb9000827,
    0x528928a2, 0x72aa0862, 0xb9000022, 0xd503205f, 0x17ffffff,
};

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

__attribute__((section(".text.start"), noreturn))
void _start(void) {
    volatile u32 *mb = (volatile u32 *)MBOX;

    puts_("ccplex test: start\n");
    for (u32 i = 0; i < sizeof(cpu0_blob) / sizeof(cpu0_blob[0]); i++)
        ((volatile u32 *)CPU_ENTRY)[i] = cpu0_blob[i];
    for (u32 i = 0; i < 4; i++)
        mb[i] = 0;

    /* 1. Nothing powered: the release must be refused. */
    release_cpu0();
    udelay(2000);
    puts_(mb[0] == PCIE_MAGIC ? "unpowered: CPU0 RAN (wrong)\n"
                              : "unpowered: CPU0 stayed dark\n");
    hold_cpu0();

    /* 2. The real bring-up, in ccplex_boot_cpu0() order. */
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

    /* Released with its clock stopped, CPU0 must not run until the stop is
     * cleared; the strobes and the active-cluster reset view read back. */
    REG(CAR + 0x460) = 1u << 8;                  /* CLK_CPUG_CMPLX_SET: CPU0 */
    release_cpu0();
    udelay(5000);
    puts_(mb[0] == PCIE_MAGIC ? "clock stopped: CPU0 RAN (wrong)\n"
                              : "clock stopped: CPU0 held still\n");
    puts_("clock stop reads ");
    puthex(REG(CAR + 0x378));                    /* CLK_CPUG_CMPLX */
    putc_(' ');
    puthex(REG(CAR + 0x464));                    /* CLK_CPUG_CMPLX_CLR */
    putc_('\n');
    puts_(REG(CAR + 0x340) == REG(CAR + 0x450)   /* RST_CPU(G)_CMPLX_SET */
              ? "reset view: RST_CPU_CMPLX matches RST_CPUG_CMPLX\n"
              : "reset view: RST_CPU_CMPLX differs\n");
    REG(CAR + 0x464) = 1u << 8;                  /* CLK_CPUG_CMPLX_CLR */

    u32 start = REG(TIMERUS);
    while (mb[0] != PCIE_MAGIC && REG(TIMERUS) - start < 100000)
        udelay(100);
    puts_("powered: magic=");
    puthex(mb[0]);
    puts_(" el=");
    puthex(mb[1]);
    puts_(" dt=");
    puthex(mb[2]);
    putc_('\n');

    /* Let CPU0 sit in its WFE park for a while: it must cost next to
     * nothing, which the retired-instruction count at reset shows. */
    udelay(50000);

    /* 3. Back into reset, then power off. */
    hold_cpu0();
    puts_("ccplex test: done\n");
    i2c5_write(0x3C, 0x41, 0x02);                /* MAX77620 ONOFFCNFG1: PWR_OFF */
    for (;;)
        ;
}
