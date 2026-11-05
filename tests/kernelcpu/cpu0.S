/*
 * CPU0 side of the kernel-CPU test: what a kernel needs from the core
 * model, exercised the way Linux uses it.
 *
 * Entered at EL3, MMU off, at 0xA0000000. Drops to EL1h the way a boot
 * shim hands over to a kernel, then:
 *   [1] CurrentEL at EL1                  (expect 0x4)
 *   [2] PSCI_VERSION over SMC             (expect 0x00010000)
 *   [3] ESR_EL1 seen by the BRK handler   (expect 0xF2000123)
 *   [4] CNTFRQ_EL0                        (expect 19200000)
 *   [5] virtual timer interrupts taken    (expect 10)
 *   [6] CNTVCT ticks across them          (expect about 10 x 19200)
 *   [7] the last interrupt ID acknowledged (expect 27)
 *   [8] instructions spent waiting: the WFI loop count (expect small)
 *   [0] magic 'KCPU', written last
 * then parks in WFE.
 */
        .equ MBOX,  0xA0030000
        .equ GICD,  0x50041000
        .equ GICC,  0x50042000
        .equ TICK,  19200          /* 1 ms at 19.2 MHz */

        .text
        .global _start
_start:
        mov     x0, #0x431             /* SCR_EL3: RW, RES1, NS */
        msr     scr_el3, x0
        mov     x0, #(1 << 31)         /* HCR_EL2.RW: EL1 is AArch64 */
        msr     hcr_el2, x0
        mov     x0, #3
        msr     cnthctl_el2, x0
        msr     cntvoff_el2, xzr
        ldr     x0, =0x30C50830        /* SCTLR_EL1: RES1, MMU off */
        msr     sctlr_el1, x0
        mov     x0, #0x3C5             /* EL1h, DAIF masked */
        msr     spsr_el3, x0
        adr     x0, el1
        msr     elr_el3, x0
        eret

el1:
        adr     x0, vectors
        msr     vbar_el1, x0
        isb
        ldr     x1, =MBOX
        mrs     x2, currentel
        str     w2, [x1, #4]

        ldr     x0, =0x84000000        /* PSCI_VERSION */
        smc     #0
        str     w0, [x1, #8]

        brk     #0x123                 /* the sync handler steps over it */

        mrs     x3, cntfrq_el0
        str     w3, [x1, #16]

        ldr     x4, =GICD
        mov     w5, #1
        str     w5, [x4]               /* GICD_CTLR */
        mov     w5, #(1 << 27)
        str     w5, [x4, #0x100]       /* ISENABLER0: PPI 27 */
        ldr     w5, [x4, #0x418]
        and     w5, w5, #0x00FFFFFF
        mov     w12, #0xA0000000
        orr     w5, w5, w12
        str     w5, [x4, #0x418]       /* IPRIORITYR[27] = 0xA0 */
        ldr     x6, =GICC
        mov     w5, #0xF0
        str     w5, [x6, #4]           /* GICC_PMR */
        mov     w5, #1
        str     w5, [x6]               /* GICC_CTLR */

        mov     w7, #TICK
        msr     cntv_tval_el0, x7
        mov     w7, #1
        msr     cntv_ctl_el0, x7
        mrs     x8, cntvct_el0
        mov     w11, #0
        msr     daifclr, #2            /* IRQs on */
1:      wfi
        add     w11, w11, #1
        ldr     w9, [x1, #20]
        cmp     w9, #10
        b.lo    1b
        msr     daifset, #2
        mrs     x10, cntvct_el0
        sub     x10, x10, x8
        str     w10, [x1, #24]
        str     w11, [x1, #32]
        mov     w7, #0
        msr     cntv_ctl_el0, x7

        ldr     w2, =0x4B435055        /* 'KCPU' */
        str     w2, [x1]
2:      wfe
        b       2b

        .ltorg

        .balign 2048
vectors:
        .skip   0x200                  /* current EL with SP_EL0: unused */
        /* 0x200: synchronous, current EL with SP_ELx */
        mrs     x20, esr_el1
        str     w20, [x1, #12]
        mrs     x21, elr_el1
        add     x21, x21, #4
        msr     elr_el1, x21
        eret
        .balign 0x80
        /* 0x280: IRQ, current EL with SP_ELx */
        ldr     w22, [x6, #0x0C]       /* GICC_IAR */
        ldr     w23, [x1, #20]
        add     w23, w23, #1
        str     w23, [x1, #20]
        str     w22, [x1, #28]
        mov     w24, #TICK
        msr     cntv_tval_el0, x24
        str     w22, [x6, #0x10]       /* GICC_EOIR */
        eret
        .balign 0x80
        .skip   0x80 * 2               /* FIQ, SError */
        .skip   0x400                  /* lower ELs: unused */
