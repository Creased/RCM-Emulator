#pragma once
#include <cstdio>

// Per-access register tracing. Off unless asked for, because it is enormous:
// a full hwtest sweep emits ~52,000 lines, ~51,000 of which are [gpio],
// [mmio] and [sdmmc] traces. Piped to a file that is merely wasteful, but on
// a Windows console every line goes through conhost one write at a time and
// the emulator slows to a crawl - which is exactly what a user sees when
// they double-click the executable instead of running it from a script.
//
// Enable with --trace on the command line or RCM_EMU_TRACE=1.
extern bool emu_trace_enabled;

#define TRACE(...)                                                            \
    do {                                                                      \
        if (emu_trace_enabled)                                                \
            printf(__VA_ARGS__);                                              \
    } while (0)
