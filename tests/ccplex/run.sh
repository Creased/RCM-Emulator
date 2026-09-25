#!/bin/sh
# CCPLEX regression test: boot CPU0 from a BPMP payload and check what the
# emulator and the payload report. Usage: tests/ccplex/run.sh [rcm_emu] [payload]
EMU=${1:-./rcm_emu}
PAYLOAD=${2:-tests/ccplex/payload.bin}
LOG=${LOG:-tests/ccplex/out.log}

SDL_VIDEODRIVER=dummy RCM_EMU_NO_AUDIO=1 timeout 120 "$EMU" "$PAYLOAD" \
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

echo "ccplex test (emulator exit $status):"
expect 'cannot run: CPU rail off'            'an unpowered release is refused, with the reason'
expect 'unpowered: CPU0 stayed dark'         'and the core really did not execute'
expect 'CPU0 released: AArch64 EL3, MMU off, entry 0xA0000000' 'a powered release boots CPU0 at the SB vector'
expect 'powered: magic=50434945'             'CPU0 ran the blob and wrote the mailbox'
expect 'el=0000000C'                         'CPU0 came up at EL3'
expect 'CPU0 stopped (held in reset)'        'RST_CPUG_CMPLX_SET stops the core'
expect 'MAX77620 PWR_OFF received'           'the payload ran to its power-off'
if [ "$status" -eq 0 ]; then
    echo "  ok    the emulator exited cleanly"
else
    echo "  FAIL  the emulator exited with status $status (124 = timed out)"
    fail=1
fi

# TIMERUS as CPU0 saw it must have moved across its busy loop.
dt=$(sed -n 's/.*dt=\([0-9A-F]*\).*/\1/p' "$LOG" | head -1)
if [ -n "$dt" ] && [ "$dt" != "00000000" ]; then
    echo "  ok    CPU0 sees time advance (TIMERUS delta 0x$dt)"
else
    echo "  FAIL  CPU0's TIMERUS delta is zero or missing"
    fail=1
fi

# Parked in WFE for ~50 ms, the core must retire next to nothing: the park
# loop sleeps to the end of each slice instead of spinning.
retired=$(sed -n 's/.*CPU0 stopped (held in reset) after \([0-9]*\) instructions.*/\1/p' "$LOG" | head -1)
if [ -n "$retired" ] && [ "$retired" -lt 100000 ]; then
    echo "  ok    WFE parks the core ($retired instructions retired in total)"
else
    echo "  FAIL  WFE did not park the core (retired: ${retired:-?})"
    fail=1
fi

[ $fail -eq 0 ] && echo "ccplex test: PASS" || { echo "ccplex test: FAIL - see $LOG"; exit 1; }
