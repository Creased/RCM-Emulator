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
expect 'ram repair: done'                    'a powered RAM repair completes'
expect 'unpowered: CPU0 stayed dark'         'and the core really did not execute'
expect 'CPU0 released: AArch64 EL3, MMU off, entry 0xA0000000' 'a powered release boots CPU0 at the SB vector'
expect 'clock stopped: CPU0 held still'      'a core released with its clock stopped does not run'
expect 'clock stop reads 00000100 00000100'  'CLK_CPUG_CMPLX and its CLR strobe read back the stop'
expect 'RST_CPU_CMPLX matches RST_CPUG_CMPLX' 'the active-cluster reset register reads the G cluster'
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


# ---- fault injection ------------------------------------------------------
#
# The same payload against broken hardware. Pinned to Erista, which is what
# the payload's bring-up drives (MAX77621 + MAX77620 GPIO5), so the result
# does not depend on whatever rcm_emu.ini happens to say.
expect_not() {
    if grep -q -- "$1" "$FLOG"; then
        echo "  FAIL  $2 (present: $1)"
        fail=1
    else
        echo "  ok    $2"
    fi
}
fexpect() {
    if grep -q -- "$1" "$FLOG"; then
        echo "  ok    $2"
    else
        echo "  FAIL  $2 (missing: $1)"
        fail=1
    fi
}
run_fault() {
    FLOG=tests/ccplex/out_fault_$1.log
    SDL_VIDEODRIVER=dummy RCM_EMU_NO_AUDIO=1 timeout 120 "$EMU" "$PAYLOAD" \
        --oem erista --fault "$1" > "$FLOG" 2>&1
    fstatus=$?
    echo "ccplex test, --fault $1 (emulator exit $fstatus):"
    if [ "$fstatus" -ne 0 ]; then
        echo "  FAIL  the emulator exited with status $fstatus (124 = timed out)"
        fail=1
    fi
}

# A rail whose buck answers and takes its enable, but delivers nothing. Over
# I2C the console looks healthy; only what the rail feeds notices.
run_fault vdd_cpu
fexpect '\[fault\] vdd_cpu (rail)'                 'the fault is announced before the run'
fexpect 'RAM repair requested with no CPU rail (fault: vdd_cpu)' 'the RAM repair is held, and says why'
fexpect 'ram repair: never completed'              'so STS never sets (bdk would spin here forever)'
fexpect 'cannot run: CPU rail has no output (fault: vdd_cpu' 'the release is refused, naming the fault'
expect_not 'CPU0 released: AArch64'                'and the core never executes'
fexpect 'powered: magic=00000000'                  'the payload sees no mailbox write'
expect_not 'I2C5 0x1B NAK'                         'the buck itself still answers on I2C'
fexpect 'MAX77620 PWR_OFF received'                'the payload still runs to its power-off'

# A dead buck: NAKs, takes no writes, and the rail goes with it - even though
# its register file was seeded with VOUT_EN set.
run_fault max77621_cpu
fexpect 'I2C5 0x1B NAK: max77621_cpu is dead'      'the dead chip NAKs its address'
fexpect 'ram repair: never completed'              'its rail is dead with it'
fexpect 'cannot run: CPU rail has no output (fault: max77621_cpu dead)' 'the release is refused, naming the chip'
expect_not 'CPU0 released: AArch64'                'and the core never executes'
fexpect 'MAX77620 PWR_OFF received'                'the main PMIC, not faulted, still powers off'

# A fault on the other generation's chip must change nothing here: Erista
# has no MAX77812 to break.
run_fault max77812
fexpect 'ram repair: done'                         'an Erista ignores a dead MAX77812'
fexpect 'CPU0 released: AArch64 EL3, MMU off, entry 0xA0000000' 'and CPU0 boots as normal'
expect_not 'NAK: max77812 is dead'                 'a chip this board does not have is not reported dead'

[ $fail -eq 0 ] && echo "ccplex test: PASS" || { echo "ccplex test: FAIL - see $LOG"; exit 1; }
