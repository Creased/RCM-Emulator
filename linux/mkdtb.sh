#!/bin/sh
# linux/mkdtb.sh IN.dtb OUT.dtb - the switchroot device tree with what the
# emulator boot needs: a console on UART-B (the port the emulator prints),
# kept past init, one CPU, verbose logging. The bootloader normally edits
# bootargs the same way; nothing else changes.
set -e
DTC=${DTC:-$HOME/l4t/kernel/kernel-4.9/scripts/dtc/dtc}
IN=$1; OUT=$2
T=$(mktemp -d)
"$DTC" -I dtb -O dts -q -o "$T/in.dts" "$IN"
ARGS=${BOOTARGS:-"earlycon=uart8250,mmio32,0x70006040 keep_bootcon console=tty0 maxcpus=1 nosmp loglevel=8 ignore_loglevel clk_ignore_unused printk.time=1"}
python3 - "$T/in.dts" "$T/out.dts" "$ARGS" <<'PY'
import re, sys
s = open(sys.argv[1]).read()
args = sys.argv[3]
s, n = re.subn(r'(\n\tchosen \{[^}]*?bootargs = )"[^"]*";', lambda m: m.group(1) + '"' + args + '";', s, count=1)
if n != 1:
    raise SystemExit("no /chosen bootargs found")
open(sys.argv[2], "w").write(s)
PY
"$DTC" -I dts -O dtb -q -o "$OUT" "$T/out.dts"
rm -rf "$T"
echo "built $OUT"
