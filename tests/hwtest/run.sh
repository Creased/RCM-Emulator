#!/bin/sh
# End-to-end: run hwtest's whole hardware sweep in the emulator, headless,
# and check the CPU0-driven PCIe probe enumerated the Wi-Fi endpoint.
# Usage: tests/hwtest/run.sh <rcm_emu> <hwtest.bin> [extra emulator args...]
#
# hwtest sits in its pager once the sweep is dumped, so the run is ended as
# soon as the dump's last line appears (with a hard timeout behind it).
EMU=${1:-./rcm_emu}
BIN=${2:-build-hwtest/hwtest-rcm/build/hwtest.bin}
# Anything after the two positional arguments goes to the emulator. (A bare
# `shift 2` with fewer than two arguments makes dash exit on the spot.)
[ $# -ge 2 ] && shift 2 || set --
LOG=${LOG:-tests/hwtest/out.log}
LIMIT=${LIMIT:-300}

# POWER at 3 s picks "Hardware test" on hwtest's launcher.
SDL_VIDEODRIVER=dummy RCM_EMU_NO_AUDIO=1 "$EMU" "$BIN" --input-script "3000 P" "$@" \
    > "$LOG" 2>&1 &
pid=$!
t=0
until grep -q 'end of dump' "$LOG" 2>/dev/null || ! kill -0 $pid 2>/dev/null; do
    sleep 1
    t=$((t + 1))
    if [ $t -ge "$LIMIT" ]; then
        echo "hwtest: no end of dump after ${LIMIT}s"
        break
    fi
done
kill $pid 2>/dev/null
wait $pid 2>/dev/null
echo "hwtest: sweep took ~${t}s of wall time"

fail=0
expect() {
    if grep -q -- "$1" "$LOG"; then
        echo "  ok    $2"
    else
        echo "  FAIL  $2 (missing: $1)"
        fail=1
    fi
}
expect 'end of dump'                                 'the whole sweep ran to its end'
expect '\[ccplex\] CPU0 released: AArch64 EL3'       'hwtest booted CPU0'
expect 'CPU0          : ran to completion'           'the CPU0 stub ran to completion'
expect 'AFI (CPU)     : cfg=00103025 witness=A5A50000 -> APERTURE LIVE' 'the PCIe aperture decodes for CPU0'
expect 'RP1 link      : UP, DL active (LNKSTA=3011'  'root port 1 trained'
expect 'EP config     : 14E4:43EC'                   'the CYW4356 endpoint enumerated'
expect 'CPU0          : powergated'                  'the cluster was powergated again'
expect 'Wireless       : pass'                       'the Wireless verdict passed'
expect '\[sdmmc\] SDMMC4 tuned: 128 iterations'      'the eMMC passed HS200 tuning'
expect 'card_clock   : 199.68 MHz'                    'the eMMC runs at HS400 speed'

[ $fail -eq 0 ] && echo "hwtest: PASS" || { echo "hwtest: FAIL - see $LOG"; exit 1; }
