#!/bin/sh
# USB device mode regression test: tests/usb/payload.c brings up a mass
# storage gadget on each device controller and the emulator's --usb-host PC
# enumerates it, reads it back, writes its last sectors back unchanged and
# ejects it; on a write-protected disk it leaves the write out. With
# --usb-host nbd the disk is served over NBD instead, and nbd_check.py reads
# and writes it. Without a host the gadget must see nothing on the cable.
# Usage: tests/usb/run.sh [rcm_emu] [payload dir]
EMU=${1:-./rcm_emu}
DIR=${2:-tests/usb}
fail=0

run() {
    log=$DIR/out_$1.log
    shift
    SDL_VIDEODRIVER=dummy RCM_EMU_NO_AUDIO=1 timeout 60 "$EMU" "$@" > "$log" 2>&1
    echo $?
}

expect() {
    if grep -q -- "$2" "$1"; then
        echo "  ok    $3"
    else
        echo "  FAIL  $3 (missing: $2)"
        fail=1
    fi
}

expect_not() {
    if grep -q -- "$2" "$1"; then
        echo "  FAIL  $3 (found: $2)"
        fail=1
    else
        echo "  ok    $3"
    fi
}

crc() { sed -n "s/.*disk crc32 first [0-9A-F]* last [0-9A-F]* all \([0-9A-F]*\).*/\1/p" "$1"; }

# Enumeration and the reads, the same in every mass storage run.
check_disk() {
    log=$1
    name=$2
    expect "$log" '\[usb-host\] device attached'                       "$name: the host sees the device attach"
    expect "$log" 'high-speed device 1209:7E57 "RCM-Emulator" "USB test disk" serial "0001", address 1' \
                                                                        "$name: enumerated, strings read, address set"
    expect "$log" 'configuration 1: mass storage, bulk-only (EP 81 IN, EP 01 OUT, 512-byte packets)' \
                                                                        "$name: configuration parsed"
    expect "$log" 'LUN 0: "RCMEMU  Test disk       ", removable'       "$name: INQUIRY"
    expect "$log" 'TEST UNIT READY: sense key 6, ASC 29/00'             "$name: unit attention, then REQUEST SENSE"
    expect "$log" '384 blocks of 512 bytes (192 KiB)'                   "$name: READ CAPACITY"
}

check_end() {
    log=$1
    name=$2
    want=$3
    expect "$log" "usb test: ejected; disk crc32 $want"                 "$name: START STOP UNIT ejects, the disk as expected"
    expect "$log" '\[usb-host\] device detached'                        "$name: the host sees the device go"
    expect "$log" 'usb test: done'                                      "$name: the payload ran to its end"
}

check_host() {
    log=$1
    name=$2
    check_disk "$log" "$name"
    first=$(sed -n 's/.*disk crc32 first \([0-9A-F]*\) last.*/\1/p' "$log")
    last=$(sed -n 's/.*disk crc32 first [0-9A-F]* last \([0-9A-F]*\).*/\1/p' "$log")
    expect "$log" "read blocks 0-63 (crc32 $first)"                     "$name: READ(10) of blocks 0-63 returns the disk"
    expect "$log" "read blocks 376-383 (crc32 $last)"                   "$name: READ(10) of the last 8 blocks returns the disk"
    if [ "$3" = ro ]; then
        expect "$log" '\[usb-host\] write-protected'                    "$name: MODE SENSE reports write protection"
        expect "$log" 'write check skipped: the disk is write-protected' "$name: no write to a write-protected disk"
        expect_not "$log" 'usb test: WRITE(10)'                         "$name: the gadget sees no WRITE(10)"
    else
        expect "$log" '\[usb-host\] writable'                           "$name: MODE SENSE reports a writable disk"
        expect "$log" 'usb test: WRITE(10) blocks 376-383'              "$name: WRITE(10) of the last 8 blocks"
        expect "$log" 'usb test: SYNCHRONIZE CACHE'                     "$name: SYNCHRONIZE CACHE"
        expect "$log" 'wrote blocks 376-383 back unchanged; read back: the same data' \
                                                                        "$name: the blocks read back as written"
    fi
    check_end "$log" "$name" "$(crc "$log")"
}

# --usb-host nbd: nbd_check.py reads, writes (or is refused) and disconnects.
check_nbd() {
    n=$1
    name=$2
    mode=$3
    shift 3
    port=$((PORT_BASE + n))
    log=$DIR/out_nbd$n.log
    SDL_VIDEODRIVER=dummy RCM_EMU_NO_AUDIO=1 timeout 60 "$EMU" "$@" --usb-host nbd:$port > "$log" 2>&1 &
    emu=$!
    python3 "$DIR/nbd_check.py" $port "$log" $mode > "$DIR/out_nbd${n}_client.log" 2>&1
    client=$?
    wait $emu
    st=$?
    cat "$DIR/out_nbd${n}_client.log"
    [ $client -eq 0 ] || { echo "  FAIL  $name: NBD client"; fail=1; }
    check_disk "$log" "$name"
    expect "$log" "serving the disk over NBD at 127.0.0.1:$port"        "$name: serving over NBD"
    expect "$log" 'NBD client has the disk'                             "$name: the client negotiated"
    expect "$log" 'NBD client disconnected'                             "$name: the client disconnected"
    expect "$log" 'ejecting the disk'                                   "$name: the disk is ejected on disconnect"
    want=$(sed -n 's/^nbd check: disk crc32 \([0-9A-F]*\)$/\1/p' "$DIR/out_nbd${n}_client.log")
    check_end "$log" "$name" "${want:-none}"
    [ "$st" -eq 0 ] || { echo "  FAIL  $name: emulator exit $st"; fail=1; }
}

echo "usb test:"
st=$(run 1 "$DIR/payload_1.bin" --usb-host)
check_host "$DIR/out_1.log" "USB2 controller"
[ "$st" -eq 0 ] || { echo "  FAIL  USB2 controller: emulator exit $st"; fail=1; }

st=$(run 2 "$DIR/payload_2.bin" --usb-host --oem mariko)
check_host "$DIR/out_2.log" "XUSB controller"
[ "$st" -eq 0 ] || { echo "  FAIL  XUSB controller: emulator exit $st"; fail=1; }

st=$(run 4 "$DIR/payload_4.bin" --usb-host)
check_host "$DIR/out_4.log" "write-protected disk" ro
[ "$st" -eq 0 ] || { echo "  FAIL  write-protected disk: emulator exit $st"; fail=1; }

st=$(run 3 "$DIR/payload_3.bin" --usb-host)
log=$DIR/out_3.log
expect "$log" 'configuration 1: HID (EP 81 IN, interrupt every 4)' "HID gadget: configuration parsed"
expect "$log" 'HID report descriptor: 21 bytes'                    "HID gadget: report descriptor read"
expect "$log" 'SET_IDLE done; polling EP 81 for reports'           "HID gadget: SET_IDLE"
expect "$log" 'HID report 1: 10 11 12 13 14 15 16 17'              "HID gadget: first interrupt report"
expect "$log" 'HID report 3: 30 31 32 33 34 35 36 37'              "HID gadget: third interrupt report"
expect "$log" '\[usb-host\] device detached'                       "HID gadget: the host sees the device go"
[ "$st" -eq 0 ] || { echo "  FAIL  HID gadget: emulator exit $st"; fail=1; }

st=$(run 0 "$DIR/payload_1.bin")
expect "$DIR/out_0.log" 'usb test: no bus reset, nothing on the cable' "no host: the gadget waits on an empty cable"
if grep -q '\[usb-host\]' "$DIR/out_0.log"; then
    echo "  FAIL  no host: the host logged activity"
    fail=1
fi

if command -v python3 > /dev/null; then
    PORT_BASE=${USB_TEST_NBD_PORT:-$((20000 + $$ % 20000))}
    check_nbd 1 "NBD, USB2 controller" rw "$DIR/payload_1.bin"
    check_nbd 2 "NBD, XUSB controller" rw "$DIR/payload_2.bin" --oem mariko
    check_nbd 4 "NBD, write-protected disk" ro "$DIR/payload_4.bin"
else
    echo "  skip  NBD: no python3"
fi

[ $fail -eq 0 ] && echo "usb test: PASS" || { echo "usb test: FAIL (logs: $DIR/out_*.log)"; exit 1; }
