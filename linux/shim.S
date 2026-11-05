/*
 * The hand-over a kernel expects from its bootloader, as switchroot's chain
 * (hekate, ATF, U-Boot) leaves it: AArch64 EL1h, non-secure, MMU and caches
 * off, x0 = the device tree, x1..x3 = 0 (Documentation/arm64/booting.txt).
 *
 * CPU0 enters here at EL3. EL2 is configured so EL1 runs AArch64 with the
 * timers and FP/SIMD untrapped, then one ERET lands in the kernel at EL1.
 * PSCI calls the kernel makes over SMC are answered by the emulator, which
 * plays the secure monitor.
 */
        .equ KERNEL, 0x80080000
        .equ DTB,    0x83000000

        .text
        .global _start
_start:
        mov     x0, #0x431             /* SCR_EL3: RW, RES1, NS */
        msr     scr_el3, x0
        msr     cptr_el3, xzr          /* FP/SIMD not trapped to EL3 */
        mov     x0, #0x33FF
        msr     cptr_el2, x0           /* RES1 bits, nothing trapped to EL2 */
        mov     x0, #(1 << 31)         /* HCR_EL2.RW: EL1 is AArch64 */
        msr     hcr_el2, x0
        mov     x0, #3                 /* EL1 may use the physical timer */
        msr     cnthctl_el2, x0
        msr     cntvoff_el2, xzr
        ldr     x0, =0x30C50830        /* SCTLR_EL1: RES1, MMU and caches off */
        msr     sctlr_el1, x0
        mov     x0, #0x3C5             /* EL1h, DAIF masked */
        msr     spsr_el3, x0
        ldr     x0, =KERNEL
        msr     elr_el3, x0
        ldr     x0, =DTB
        mov     x1, xzr
        mov     x2, xzr
        mov     x3, xzr
        eret
        .ltorg
