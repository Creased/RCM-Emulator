#!/bin/sh
# Security Engine regression test: run tests/se/payload.c and check what it
# reports. Usage: tests/se/run.sh [rcm_emu] [payload]
EMU=${1:-./rcm_emu}
PAYLOAD=${2:-tests/se/payload.bin}
LOG=${LOG:-tests/se/out.log}

SDL_VIDEODRIVER=dummy RCM_EMU_NO_AUDIO=1 timeout 60 "$EMU" "$PAYLOAD" \
    > "$LOG" 2>&1
status=$?

echo "se test (emulator exit $status):"
# The payload's lines arrive through the UART-B log, tagged [uartB].
grep '^\[uartB\] se test: \(ok\|FAIL\) ' "$LOG" |
    sed -e 's/^.*se test: ok  */  ok    /' -e 's/^.*se test: FAIL  */  FAIL  /'
fail=0
grep -q 'se test: FAIL' "$LOG" && fail=1
if ! grep -q '^\[uartB\] se test: done$' "$LOG"; then
    echo "  FAIL  the payload did not finish cleanly"
    fail=1
fi
if [ "$status" -ne 0 ]; then
    echo "  FAIL  the emulator exited with status $status (124 = timed out)"
    fail=1
fi
[ $fail -eq 0 ] && echo "se test: PASS" || { echo "se test: FAIL (log: $LOG)"; exit 1; }
