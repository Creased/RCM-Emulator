#!/bin/sh
# Build hwtest-rcm (https://github.com/Creased/hwtest-rcm) with distro
# toolchains, CPU0 stub included. Usage: tests/hwtest/build.sh <workdir>
# Produces <workdir>/hwtest-rcm/build/hwtest.bin.
#
# hwtest's Makefile only needs $(DEVKITARM)/base_rules to exist - it names
# arm-none-eabi-gcc itself - so an empty file stands in for devkitARM, and
# Debian/Ubuntu's gcc-aarch64-linux-gnu builds the AArch64 stub, which is
# what hwtest's own CI does too.
#
# Pinned so a change on the hwtest side cannot turn this emulator's CI red
# without a commit here; bump HWTEST_REF deliberately.
set -e
WORK=${1:-build-hwtest}
HWTEST_REF=${HWTEST_REF:-3738e9a47e1833605fc5a30eeca0ea95caa3b936}

mkdir -p "$WORK"
cd "$WORK"
if [ ! -d hwtest-rcm/.git ]; then
    git init -q hwtest-rcm
    git -C hwtest-rcm remote add origin https://github.com/Creased/hwtest-rcm.git
fi
git -C hwtest-rcm fetch -q --depth 1 origin "$HWTEST_REF"
git -C hwtest-rcm checkout -q FETCH_HEAD
git -C hwtest-rcm submodule update -q --init --depth 1 third_party/hekate

mkdir -p devkitARM
: > devkitARM/base_rules
make -C hwtest-rcm -j"$(nproc)" DEVKITARM="$PWD/devkitARM" \
    A64_CC=aarch64-linux-gnu-gcc A64_OC=aarch64-linux-gnu-objcopy REQUIRE_A64=1
ls -l hwtest-rcm/build/hwtest.bin
