#!/bin/sh
# linux/run.sh IMAGE DTB [LOG] [SECS] [extra rcm_emu args...]
#
# Boots a Linux kernel (an arm64 Image, e.g. switchroot's L4T 4.9) on the
# emulator's CPU0: the emulator places the Image at 0x80080000 and the
# device tree at 0x83000000, linux/boot.bin powers CPU0 and hands over at
# EL1 through linux/shim.S. Run linux/mkdtb.sh on the device tree first so
# it carries an earlycon on UART-B, which the emulator prints.
#
# Needs an emulator built against Unicorn 2.1 or later (make UNICORN_DIR=...).
set -e
D=$(cd "$(dirname "$0")" && pwd)
E=$(cd "$D/.." && pwd)
IMAGE=$1; DTB=$2; LOG=${3:-$D/boot.log}; SECS=${4:-600}
shift 4 2>/dev/null || shift $#
export PATH=/opt/devkitpro/devkitARM/bin:/opt/devkitpro/devkitA64/bin:$PATH
[ -f "$D/boot.bin" ] || sh "$D/build.sh"
cd "$E"
SDL_VIDEODRIVER=${SDL_VIDEODRIVER:-dummy} RCM_EMU_NO_AUDIO=1 timeout "$SECS" \
    ./rcm_emu "$D/boot.bin" --oem erista \
    --load "$IMAGE@0x80080000" --load "$DTB@0x83000000" "$@" > "$LOG" 2>&1 || true
echo "log: $LOG ($(grep -a -c '\[uartB\]' "$LOG") kernel lines)"
