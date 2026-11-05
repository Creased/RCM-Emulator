#!/bin/sh
# Kernel-CPU regression test: CPU0 dropped to EL1 takes timer interrupts
# through the GIC, a BRK into its own vector, and a PSCI call over SMC.
# Usage: tests/kernelcpu/run.sh [rcm_emu] [payload]
EMU=${1:-./rcm_emu}
PAYLOAD=${2:-tests/kernelcpu/payload.bin}
LOG=${LOG:-tests/kernelcpu/out.log}

SDL_VIDEODRIVER=dummy RCM_EMU_NO_AUDIO=1 timeout 120 "$EMU" "$PAYLOAD" --oem erista \
    > "$LOG" 2>&1
status=$?

fail=0
expect() {
    if grep -q -- "$1" "$LOG"; then
        echo "  ok    $2"
    else
        echo "  FAIL  $2 (missing: $1)"
        fail=1
    fi
}

echo "kernelcpu test (emulator exit $status):"
expect 'magic=4B435055'     'CPU0 ran the whole test and wrote the mailbox'
expect 'el=00000004'        'the shim dropped it to EL1'
expect 'psci=00010000'      'PSCI_VERSION over SMC answers 1.0'
expect 'esr=F2000123'       'BRK is taken to the EL1 vector with its ESR'
expect 'cntfrq=0124F800'    'the counter runs at 19.2 MHz'
expect 'irqs=0000000A'      'ten virtual timer interrupts arrive through the GIC'
expect 'irqid=0000001B'     'the GIC acknowledges them as PPI 27'

ticks=$(sed -n 's/.* ticks=\([0-9A-F]*\).*/\1/p' "$LOG" | head -1)
if [ -n "$ticks" ] && [ $((0x$ticks)) -ge 190000 ] && [ $((0x$ticks)) -le 200000 ]; then
    echo "  ok    ten 1 ms ticks take about 192000 counts (0x$ticks)"
else
    echo "  FAIL  the tick count across ten interrupts is off (0x${ticks:-?})"
    fail=1
fi
retired=$(sed -n 's/.*CPU0 stopped (held in reset) after \([0-9]*\) instructions.*/\1/p' "$LOG" | head -1)
if [ -n "$retired" ] && [ "$retired" -lt 5000 ]; then
    echo "  ok    WFI sleeps until the timer ($retired instructions retired)"
else
    echo "  FAIL  WFI spun instead of sleeping (retired: ${retired:-?})"
    fail=1
fi
if [ "$status" -eq 0 ]; then
    echo "  ok    the emulator exited cleanly"
else
    echo "  FAIL  the emulator exited with status $status (124 = timed out)"
    fail=1
fi
[ $fail -eq 0 ] && echo "kernelcpu test: PASS" || echo "kernelcpu test: FAIL"
exit $fail
