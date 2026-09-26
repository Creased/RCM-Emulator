#!/bin/sh
# Display regression test: run each scenario payload (tests/display/payload.c,
# built with -DSCENARIO=n) and compare the frame the emulator shows with the
# picture the payload drew.
# Usage: tests/display/run.sh [rcm_emu] [payload dir]
EMU=$(cd "$(dirname "${1:-./rcm_emu}")" && pwd)/$(basename "${1:-./rcm_emu}")
DIR=$(cd "${2:-tests/display}" && pwd)
HERE=$(cd "$(dirname "$0")" && pwd)
LIMIT=${LIMIT:-60}

names="pitch window|VIC 270-degree turn into a pitch surface|VIC turn into a block-linear surface|DC column scan with an address offset|VIC block-linear source|V mirror with window D blended over"

run() {
    n=$1
    work=$(mktemp -d)
    # The frame snapshot lands in the working directory.
    (cd "$work" && SDL_VIDEODRIVER=dummy RCM_EMU_NO_AUDIO=1 \
        "$EMU" "$DIR/payload_$n.bin" > out.log 2>&1) &
    pid=$!
    t=0
    until grep -q 'display test: ready' "$work/out.log" 2>/dev/null; do
        sleep 1
        t=$((t + 1))
        [ $t -ge "$LIMIT" ] && break
    done
    # The renderer writes a snapshot about a second after the frame changes.
    sleep 3
    kill $pid 2>/dev/null
    wait $pid 2>/dev/null
    name=$(echo "$names" | cut -d'|' -f"$n")
    if [ ! -f "$work/last_fb.rgba" ]; then
        echo "  FAIL  $n: $name (no frame snapshot)"
        rc=1
    elif python3 "$HERE/check.py" "$n" "$work/last_fb.rgba" "$work/last_fb.meta"; then
        echo "  ok    $n: $name"
        rc=0
    else
        echo "  FAIL  $n: $name (log: $work/out.log)"
        rc=1
    fi
    [ $rc -eq 0 ] && rm -rf "$work"
    return $rc
}

echo "display test:"
fail=0
pids=""
for n in 1 2 3 4 5 6; do
    run "$n" > "$DIR/result_$n.txt" 2>&1 &
    pids="$pids $!"
done
n=0
for p in $pids; do
    n=$((n + 1))
    wait "$p" || fail=1
    cat "$DIR/result_$n.txt"
    rm -f "$DIR/result_$n.txt"
done
[ $fail -eq 0 ] && echo "display test: PASS" || { echo "display test: FAIL"; exit 1; }
