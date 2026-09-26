#!/bin/sh
# USB device mode regression test: tests/usb/payload.c brings up a mass
# storage gadget on each device controller and the emulator's --usb-host PC
# enumerates it, reads it back and ejects it; without a host the gadget must
# see nothing on the cable. Usage: tests/usb/run.sh [rcm_emu] [payload dir]
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

check_host() {
    log=$1
    name=$2
    expect "$log" '\[usb-host\] device attached'                       "$name: the host sees the device attach"
    expect "$log" 'high-speed device 1209:7E57 "RCM-Emulator" "USB test disk" serial "0001", address 1' \
                                                                        "$name: enumerated, strings read, address set"
    expect "$log" 'configuration 1: mass storage, bulk-only (EP 81 IN, EP 01 OUT, 512-byte packets)' \
                                                                        "$name: configuration parsed"
    expect "$log" 'LUN 0: "RCMEMU  Test disk       ", removable'       "$name: INQUIRY"
    expect "$log" 'TEST UNIT READY: sense key 6, ASC 29/00'             "$name: unit attention, then REQUEST SENSE"
    expect "$log" '128 blocks of 512 bytes'                             "$name: READ CAPACITY"
    first=$(sed -n 's/.*disk crc32 first \([0-9A-F]*\) last.*/\1/p' "$log")
    last=$(sed -n 's/.*disk crc32 first [0-9A-F]* last \([0-9A-F]*\).*/\1/p' "$log")
    expect "$log" "read blocks 0-63 (crc32 $first)"                     "$name: READ(10) of blocks 0-63 returns the disk"
    expect "$log" "read blocks 120-127 (crc32 $last)"                   "$name: READ(10) of the last 8 blocks returns the disk"
    expect "$log" 'usb test: ejected'                                   "$name: START STOP UNIT ejects"
    expect "$log" '\[usb-host\] device detached'                        "$name: the host sees the device go"
    expect "$log" 'usb test: done'                                      "$name: the payload ran to its end"
}

echo "usb test:"
st=$(run 1 "$DIR/payload_1.bin" --usb-host)
check_host "$DIR/out_1.log" "USB2 controller"
[ "$st" -eq 0 ] || { echo "  FAIL  USB2 controller: emulator exit $st"; fail=1; }

st=$(run 2 "$DIR/payload_2.bin" --usb-host --oem mariko)
check_host "$DIR/out_2.log" "XUSB controller"
[ "$st" -eq 0 ] || { echo "  FAIL  XUSB controller: emulator exit $st"; fail=1; }

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

[ $fail -eq 0 ] && echo "usb test: PASS" || { echo "usb test: FAIL (logs: $DIR/out_*.log)"; exit 1; }
