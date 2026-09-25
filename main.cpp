/*
 * RCM Payload Emulator - Main Entry Point
 *
 * Loads an RCM binary payload (.bin), initializes ARM32 emulation
 * via Unicorn Engine, sets up T210 memory map and MMIO hooks,
 * and runs the payload with SDL2 display output.
 *
 * Usage: ./rcm_emu <payload.bin>
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#include <unicorn/unicorn.h>

#include "emu_state.h"
#include "platform.h"
#include "payload_picker.h"
#include "trace.h"
#include "t210/bpmp.h"
#include "t210/ccplex.h"
#include "t210/memory_map.h"
#include "t210/mmio.h"
#include "display/sdl_display.h"
#include "display/config_window.h"
#include "display/console_window.h"
#include "input_script.h"

// Low 16 MB of the map (NULL-pointer writes land there); freed at exit.
static uint8_t *g_low_ptr = nullptr;

// Scripted-navigation progress (--auto-pin-recovery, --auto-te-script). A
// soft reboot restarts both, as it restarts the emulated clock they run on.
struct AutoScripts {
    int      pin_stage = 0;
    uint64_t pin_t = 0;
    int      pin_logged_stage = -1;
    size_t   te_idx = 0;
};
static AutoScripts g_auto;

// The IPL framebuffer starts out in hekate's background colour (0x1B1B1B),
// at power-on and again after a reboot has handed DRAM back zeroed.
static void fill_fb_background(EmuState *state) {
    for (size_t i = 0; i < FB_SIZE; i += 4) {
        state->fb_ptr[i + 0] = 0x1B; // B
        state->fb_ptr[i + 1] = 0x1B; // G
        state->fb_ptr[i + 2] = 0x1B; // R
        state->fb_ptr[i + 3] = 0xFF; // A
    }
}

// ==================== Payload Loading ====================

static uint8_t *load_payload(const char *path, size_t *out_size) {
    FILE *f = platform_fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "[error] Cannot open payload: %s\n", path);
        return nullptr;
    }

    long len = -1;
    if (fseek(f, 0, SEEK_END) == 0)
        len = ftell(f);
    if (len <= 0 || fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        fprintf(stderr, "[error] Payload is empty or unreadable: %s\n", path);
        return nullptr;
    }
    size_t size = (size_t)len;

    uint8_t *buf = (uint8_t *)malloc(size);
    if (!buf) {
        fclose(f);
        fprintf(stderr, "[error] Failed to allocate %zu bytes\n", size);
        return nullptr;
    }

    size_t nread = fread(buf, 1, size, f);
    if (nread != size) {
        fprintf(stderr, "[warning] Short read: %zu/%zu bytes\n", nread, size);
        size = nread;
    }
    fclose(f);

    *out_size = size;
    printf("[loader] Loaded payload: %s (%zu bytes / %.1f KB)\n", path, size, size / 1024.0);

    // Try to identify the payload
    if (size >= 0x120) {
        uint32_t magic = *(uint32_t *)(buf + 0x118);
        if (magic == 0x43544349) { // "ICTC" - hekate
            uint32_t ver = *(uint32_t *)(buf + 0x11C);
            int major = (ver & 0xFF) - '0';
            int minor = ((ver >> 8) & 0xFF) - '0';
            int hotfix = ((ver >> 16) & 0xFF) - '0';
            printf("[loader] Detected: Hekate v%d.%d.%d\n", major, minor, hotfix);
        } else if (magic == 0x4E595849) { // "IXYN" - Nyx
            printf("[loader] Detected: Nyx GUI\n");
        } else {
            printf("[loader] Unknown payload type (magic: 0x%08X)\n", magic);
        }
    }

    return buf;
}

// ==================== Emulation Setup ====================

static uc_engine *setup_emulation(EmuState *state, uint8_t *payload, size_t payload_size) {
    uc_engine *uc;
    uc_err err;
    
    // Open Unicorn in ARM32 mode with Thumb support
    err = uc_open(UC_ARCH_ARM, UC_MODE_ARM, &uc);
    if (err != UC_ERR_OK) {
        fprintf(stderr, "[error] uc_open failed: %s\n", uc_strerror(err));
        return nullptr;
    }
    state->uc = uc;

    // No UC_HOOK_CODE anywhere: a per-instruction callback keeps Unicorn off
    // its fast path for every block it covers. This used to register three
    // of them (one of them twice) - an instruction-trace ring buffer nothing
    // read, a NOP-slide check that did a uc_mem_read per instruction, and
    // the clock. The clock and the NOP-slide check now run once per block
    // (t210/bpmp.cpp).

    // Enable VFP/NEON
    uint32_t cpacr = 0x00F00000;
    uc_reg_write(uc, UC_ARM_REG_C1_C0_2, &cpacr);

    // Map IRAM (16MB)
    state->iram_ptr = (uint8_t *)calloc(1, IRAM_SIZE);
    err = uc_mem_map_ptr(uc, IRAM_BASE, IRAM_SIZE, UC_PROT_ALL, state->iram_ptr);
    if (err != UC_ERR_OK) {
        fprintf(stderr, "[error] Failed to map IRAM: %s\n", uc_strerror(err));
        uc_close(uc);
        return nullptr;
    }
    printf("[emu] Mapped IRAM: 0x%08llX - 0x%08llX (%u KB)\n",
           (unsigned long long)IRAM_BASE,
           (unsigned long long)(IRAM_BASE + IRAM_SIZE),
           (unsigned)(IRAM_SIZE / 1024));

    // Pre-set Hekate's "watchdog fired" magic at IRAM 0x4003FF18 (cookie "WDT")
    // so its early boot does `goto skip_lp0_minerva_config`, skipping both
    // libsys_lp0.bso and the Minerva DRAM-training path. The matching
    // EXCP_EN_ADDR (0x4003FF1C) is intentionally left zeroed so ERR_EXCEPTION
    // is *not* set and the user doesn't see a "hang detected" warning screen.
    // We can't model EMC/MC well enough for real Minerva training, so this is
    // the cleanest opt-out (the same path Hekate uses on hardware after a
    // legitimate WDT reset).
    {
        uint32_t wdt_magic = 0x544457; // "WDT"
        uc_mem_write(uc, 0x4003FF18, &wdt_magic, sizeof(wdt_magic));
    }

    // ---- Map DRAM as one contiguous region ----
    // 0x80000000 .. 0x100000000, i.e. the whole 32-bit-addressable DRAM window
    // the BPMP can reach. This used to be two islands (256 MB at 0x80000000
    // plus 1 GB at 0xC0000000) with a 768 MB hole between them; a payload that
    // walks RAM - a memory tester marching over the reported DRAM size - fell
    // straight into that hole and took an unmapped-access fault partway
    // through. One contiguous block removes the hole and matches what real
    // hardware presents.
    //
    // Allocated host-side as demand-zero pages (zeroed_alloc), so the full
    // 2 GB is only ever resident if a payload actually touches all of it, and
    // kept as a host pointer so a soft reboot can hand it back zeroed: Nyx
    // loads here and its file-static SD/eMMC caches would otherwise survive.
    state->dram_low_ptr = zeroed_alloc(DRAM_WINDOW_SIZE);
    if (!state->dram_low_ptr) {
        fprintf(stderr, "[error] Failed to allocate %zu MB DRAM host buffer\n",
                (size_t)(DRAM_WINDOW_SIZE / (1024 * 1024)));
        uc_close(uc);
        return nullptr;
    }
    err = uc_mem_map_ptr(uc, DRAM_BASE, DRAM_WINDOW_SIZE, UC_PROT_ALL,
                         state->dram_low_ptr);
    if (err != UC_ERR_OK) {
        fprintf(stderr, "[error] Failed to map DRAM: %s\n", uc_strerror(err));
        uc_close(uc);
        return nullptr;
    }
    printf("[emu] Mapped DRAM: 0x%08llX - 0x%09llX (%zu MB, contiguous)\n",
           (unsigned long long)DRAM_BASE,
           (unsigned long long)(DRAM_BASE + DRAM_WINDOW_SIZE),
           (size_t)(DRAM_WINDOW_SIZE / (1024 * 1024)));

    // dram_ptr keeps its historical meaning ("high DRAM at 0xC0000000") as a
    // view into the same block, so existing 0xC0000000-relative code is
    // unchanged.
    state->dram_ptr = state->dram_low_ptr + (0xC0000000ULL - DRAM_BASE);

    // ---- Map Framebuffer pointer ----
    state->fb_ptr = state->dram_low_ptr + (FB_BASE - DRAM_BASE);
    fill_fb_background(state);
    state->fb_addr = FB_BASE;
    printf("[emu] Defined FB:   0x%08llX - 0x%08llX (%u MB)\n",
           (unsigned long long)FB_BASE,
           (unsigned long long)(FB_BASE + FB_SIZE),
           (unsigned)(FB_SIZE / (1024 * 1024)));

    // ---- Map low memory (16MB @ 0x0) ----
    // hekate seems to do a memset(0, ...) for clear screen if some ptr is NULL.
    // This covers the iROM range (0x100000, 96 KB) too, as zero-filled RAM:
    // there are no BootROM contents to show, and Hekate's "Bootrom Info" /
    // "Dump Bootrom" read it without faulting. (A read-only iROM map on top
    // of this used to be attempted as well; it overlapped, so it always
    // failed, unnoticed - and a read-only iROM would fault the NULL-pointer
    // clear above.)
    g_low_ptr = (uint8_t *)calloc(1, 0x01000000);
    err = g_low_ptr ? uc_mem_map_ptr(uc, 0, 0x01000000, UC_PROT_ALL, g_low_ptr)
                    : UC_ERR_NOMEM;
    if (err != UC_ERR_OK) {
        fprintf(stderr, "[error] Failed to map low memory: %s\n", uc_strerror(err));
        uc_close(uc);
        return nullptr;
    }

    // ---- Nyx Storage (16MB @ 0xED000000) ----
    // Already mapped as part of 2GB DRAM chunk

    // Peripherals are all mapped by mmio_init() below as MMIO windows. PWM,
    // SDMMC1, RTC/PMC and SYSREG used to be mapped here first as plain RAM,
    // which only worked because MMIO was a pair of address-range hooks over
    // whatever happened to be mapped.

    // The IPATCH CAM at 0x6001DC00 is zero-mapped so the ipatches table
    // renders empty (which matches an unpatched SoC).
    err = uc_mem_map(uc, 0x6001D000, 0x1000, UC_PROT_ALL);
    if (err != UC_ERR_OK)
        fprintf(stderr, "[warn] IPATCH page not mapped: %s\n", uc_strerror(err));

    // ---- heap region (32MB @ 0x90000000) ----
    // Already mapped as part of 2GB DRAM chunk

    // ---- Copy payload to IRAM at IPL_LOAD_ADDR ----
    size_t copy_size = payload_size;
    if (copy_size > IRAM_SIZE - (IPL_LOAD_ADDR - IRAM_BASE)) {
        copy_size = IRAM_SIZE - (IPL_LOAD_ADDR - IRAM_BASE);
        printf("[loader] Warning: payload truncated to %zu bytes\n", copy_size);
    }
    memcpy(state->iram_ptr + (IPL_LOAD_ADDR - IRAM_BASE), payload, copy_size);
    printf("[emu] Payload copied to IRAM @ 0x%08llX\n", (unsigned long long)IPL_LOAD_ADDR);

    // ---- Set initial register state ----
    uint32_t sp = IPL_STACK_ADDR;
    uint32_t pc = IPL_LOAD_ADDR;

    uc_reg_write(uc, UC_ARM_REG_SP, &sp);
    uc_reg_write(uc, UC_ARM_REG_PC, &pc);

    printf("[emu] Initial PC=0x%08X SP=0x%08X\n", pc, sp);

    // ---- Peripherals and the deterministic clock ----
    mmio_init(uc, state);
    bpmp_attach(uc, state);
    printf("[emu] MMIO windows mapped\n");

    return uc;
}

// ==================== Main ====================

// Defined here, declared in trace.h. Off by default - see the note there.
bool emu_trace_enabled = false;

int main(int argc, char *argv[]) {
    if (getenv("RCM_EMU_TRACE"))
        emu_trace_enabled = true;
    for (int i = 1; i < argc; i++)
        if (strcmp(argv[i], "--trace") == 0)
            emu_trace_enabled = true;

    // Started with no payload - usually a double-click, where a usage line
    // on stderr goes to a console nobody is looking at. Offer the picker
    // instead: drop a file on its window, or press O for the system dialog.
    // Dragging a .bin onto the executable still bypasses all of this, since
    // it arrives as argv[1].
    std::string picked;
    if (argc < 2) {
        picked = payload_picker_run();
        if (picked.empty()) {
            fprintf(stderr, "Usage: %s <payload.bin> [--sd <sd.img>] [--boot0 <boot0.bin>] [--rawnand <rawnand_prefix>] [--prod-keys <prod.keys>] [--oem erista|mariko] [--bt-radio healthy|faulty|absent] [--wifi-radio healthy|faulty|absent]\n", argv[0]);
            fprintf(stderr, "\nControls:\n");
            fprintf(stderr, "  Arrow Up/Down = VOL+/VOL- buttons\n");
            fprintf(stderr, "  Enter         = POWER button\n");
            fprintf(stderr, "  Escape        = Quit\n");
            fprintf(stderr, "  F8            = Save raw guest FB to fb_dump_NNNN.raw\n");
            return 1;
        }
    }

    // Everything below indexes argv, so present the picked file as argv[1].
    const char *payload_arg = picked.empty() ? argv[1] : picked.c_str();

    printf("=== RCM Payload Emulator ===\n");
    printf("Using Unicorn Engine for ARM32 emulation\n\n");

    // Initialize emulator state
    EmuState state = {};
    state.init_fuse_defaults();
    // Overlay any saved hardware tweaks from rcm_emu.ini (cwd-relative).
    // The load is additive — missing keys keep the defaults set above.
    if (config_window_load_ini(&state, "rcm_emu.ini")) {
        printf("[config] Loaded rcm_emu.ini\n");
    }

    // Load payload binary
    size_t payload_size = 0;
    uint8_t *payload = load_payload(payload_arg, &payload_size);
    if (!payload) return 1;

    // Parse additional arguments for storage. Configuration state
    // (is_mariko, fuses, panel ID, battery values, ...) lives in
    // rcm_emu.ini and is loaded by config_window_load_ini() above.
    // CLI flags only point at on-disk images and one explicit override
    // (--oem) that lets you re-run the same payload+ini with a different
    // SoC family without editing the ini.  Any auto-derivation from
    // BOOT0 was deliberately removed: the emulator must not silently
    // contradict the ini, otherwise "patch the ini and re-run" stops
    // being a deterministic loop.
    const char *sd_path = nullptr;
    const char *boot0_path = nullptr;
    const char *rawnand_prefix = nullptr;
    const char *prod_keys_path = nullptr;

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--sd") == 0 && i + 1 < argc) {
            sd_path = argv[++i];
        } else if (strcmp(argv[i], "--boot0") == 0 && i + 1 < argc) {
            boot0_path = argv[++i];
        } else if (strcmp(argv[i], "--rawnand") == 0 && i + 1 < argc) {
            rawnand_prefix = argv[++i];
        } else if (strcmp(argv[i], "--prod-keys") == 0 && i + 1 < argc) {
            prod_keys_path = argv[++i];
        } else if (strcmp(argv[i], "--oem") == 0 && i + 1 < argc) {
            const char *oem = argv[++i];
            if (strcmp(oem, "mariko") == 0 || strcmp(oem, "t210b01") == 0) {
                state.is_mariko = true;
                state.pmic_otp  = 0x53;
                printf("[emu] OEM: Mariko (T210B01) [overrides ini]\n");
            } else if (strcmp(oem, "erista") == 0 || strcmp(oem, "t210") == 0) {
                state.is_mariko = false;
                state.pmic_otp  = 0x35;
                printf("[emu] OEM: Erista (T210) [overrides ini]\n");
            } else {
                fprintf(stderr, "[emu] Unknown --oem value '%s'; expected 'erista' or 'mariko'\n", oem);
            }
        } else if (strcmp(argv[i], "--bt-radio") == 0 && i + 1 < argc) {
            // Bluetooth radio behaviour. Same contract as --oem: the ini is
            // the source of truth, this is the one-shot override that lets
            // the same payload+ini be re-run against a dead radio.
            const char *mode = argv[++i];
            uint8_t sel = BT_RADIO_HEALTHY;
            if (bt_radio_parse(mode, &sel)) {
                state.bt_radio = sel;
                printf("[emu] BT radio: %s [overrides ini]\n", bt_radio_name(sel));
            } else {
                fprintf(stderr, "[emu] Unknown --bt-radio value '%s'; expected"
                                " 'healthy', 'faulty' or 'absent'\n", mode);
            }
        } else if (strcmp(argv[i], "--wifi-radio") == 0 && i + 1 < argc) {
            // WLAN half of the same package, on PCIe. 'faulty' means
            // something different here from the Bluetooth side: the link
            // trains and config space enumerates, but the radio die behind
            // the PCIe front-end is dead. See WifiRadioMode in emu_state.h.
            const char *mode = argv[++i];
            uint8_t sel = WIFI_RADIO_HEALTHY;
            if (wifi_radio_parse(mode, &sel)) {
                state.wifi_radio = sel;
                printf("[emu] Wi-Fi radio: %s [overrides ini]\n",
                       wifi_radio_name(sel));
            } else {
                fprintf(stderr, "[emu] Unknown --wifi-radio value '%s'; expected"
                                " 'healthy', 'faulty' or 'absent'\n", mode);
            }
        }
    }

    if (prod_keys_path) {
        extern int se_engine_load_prod_keys(const char *);
        se_engine_load_prod_keys(prod_keys_path);
    }

    // Auto-script flag: feed a deterministic button sequence after the menu
    // has settled. Useful for capturing the PIN recovery flow non-interactively
    // (e.g. when verifying a fix without manually clicking through the SDL window).
    bool auto_pin_recovery = false;
    bool auto_te_script    = false;
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--auto-pin-recovery") == 0) auto_pin_recovery = true;
        else if (strcmp(argv[i], "--auto-te-script") == 0) auto_te_script = true;
        // Generic scripted button input, for menu-driven payloads that can't
        // be driven any other way from a headless / CI run.
        else if (strcmp(argv[i], "--input-script") == 0 && i + 1 < argc) {
            // A script that does not parse would leave a CI run idling until
            // its timeout with nothing pressed; stop here instead.
            if (!input_script_load(argv[++i])) {
                fprintf(stderr, "[error] --input-script could not be loaded\n");
                return 1;
            }
        }
    }

    // Default to sd.img in the working directory when --sd is not given, so the
    // payload mounts a real FAT volume instead of failing on an empty card
    // ("f_mount failed (13)" / NO FAT at init). Explicit --sd always wins.
    if (!sd_path && access("sd.img", F_OK) == 0) {
        sd_path = "sd.img";
        printf("[emu] No --sd given; defaulting to sd.img\n");
    }

    if (sd_path) {
        state.sd_fd = open(sd_path, O_RDWR | O_BINARY);
        if (state.sd_fd < 0) perror("[emu] Failed to open SD image");
        else printf("[emu] SD image opened: %s\n", sd_path);
    }
    if (boot0_path) {
        state.emmc_boot0_fd = open(boot0_path, O_RDWR | O_BINARY);
        if (state.emmc_boot0_fd < 0) perror("[emu] Failed to open BOOT0 image");
        else printf("[emu] BOOT0 image opened: %s\n", boot0_path);
    }
    if (rawnand_prefix) {
        for (int i = 0; i < 16; i++) {
            char path[512];
            snprintf(path, sizeof(path), "%s.%02d", rawnand_prefix, i);
            int fd = open(path, O_RDWR | O_BINARY);
            if (fd >= 0) {
                state.emmc_gpp_fds.push_back(fd);
                // printf("[emu] rawnand part %02d opened: %s\n", i, path);
            } else {
                break;
            }
        }
        if (!state.emmc_gpp_fds.empty()) {
            printf("[emu] GPP rawnand opened (%zu parts starting with %s.00)\n", 
                   state.emmc_gpp_fds.size(), rawnand_prefix);
        }
    }

    // Setup ARM emulation. Hold on to the payload buffer in EmuState so the
    // soft-reboot path (config window "Reboot" button) can re-write it into
    // IRAM without re-reading from disk.
    uc_engine *uc = setup_emulation(&state, payload, payload_size);
    state.payload_ptr = payload;
    state.payload_len = payload_size;
    if (!uc) { free(payload); return 1; }

    // Initialize SDL2 display
    if (!sdl_display_init()) {
        fprintf(stderr, "[error] Failed to initialize display\n");
        uc_close(uc);
        return 1;
    }

    // Initialize the hardware-tweak config window (hidden until 'M' is pressed).
    if (!config_window_init()) {
        fprintf(stderr, "[warn] Config window init failed; M-key menu disabled\n");
    }
    // Initialize the UART console window (hidden until 'C' is pressed).
    if (!console_window_init()) {
        fprintf(stderr, "[warn] Console window init failed; C-key console disabled\n");
    }

    printf("\n[emu] Starting emulation...\n");
    printf("[emu] Esc quit | M hardware config | C UART console\n\n");

    // ---- Emulation loop ----
    // We run emulation in batches, interleaving with SDL event handling
    // and display updates to keep the UI responsive.

    const uint64_t BATCH_INSTRUCTIONS = 100000; // Instructions per batch
    const int DISPLAY_UPDATE_MS = 16;           // ~60 FPS
    // While CPU0 is running the two cores advance in lockstep slices of this
    // many BPMP instructions (250 us of emulated time). The BPMP polls CPU0's
    // mailbox every 500 us, so neither side ever waits on a stale view of the
    // other for longer than one of its own poll intervals.
    const uint64_t CPU0_QUANTUM_INSTRUCTIONS = 2500;

    auto last_display_update = std::chrono::steady_clock::now();

    while (state.running) {
        // Process SDL events (keyboard input, window close)
        if (!sdl_display_poll_events(&state, uc)) break;

        // Soft reboot: re-write the payload to IRAM, wipe DRAM (so Nyx and
        // the bootloader's file-static caches reset), reset PC/SP/clock and
        // re-prime the WDT cookie so Hekate's early boot skips Minerva again.
        if (state.reboot_requested.exchange(false)) {
            uc_emu_stop(uc);
            bool cold = state.reboot_cold.exchange(false);
            // The SoC resets as a whole: CPU0 back into reset, the PCIe root
            // complex and every other block back to power-on, the clock's
            // half-counted block gone. A power cycle resets the PMIC too.
            mmio_soft_reset(&state, cold);
            bpmp_clock_reset();
            // Fresh zero pages rather than a memset, which made all 2 GB
            // resident.
            zeroed_reset(state.dram_low_ptr, DRAM_WINDOW_SIZE);
            fill_fb_background(&state);
            size_t reload = state.payload_len;
            if (reload > IRAM_SIZE - (IPL_LOAD_ADDR - IRAM_BASE))
                reload = IRAM_SIZE - (IPL_LOAD_ADDR - IRAM_BASE);
            uc_mem_write(uc, IPL_LOAD_ADDR, state.payload_ptr, reload);
            uint32_t wdt_magic = 0x544457;
            uc_mem_write(uc, 0x4003FF18, &wdt_magic, sizeof(wdt_magic));
            // Neither the zeroed DRAM nor uc_mem_write() reaches the engine's
            // code cache, so what the last run translated - a payload it
            // chainloaded over this one's load address, Nyx in DRAM - would
            // run again in place of the fresh bytes. Drop those translations.
            // (Not UC_CTL_TB_FLUSH: on Unicorn 2.0 that memsets the whole
            // 1 GB code buffer, seconds and a gigabyte of RSS per reboot.)
            uc_ctl_remove_cache(uc, 0, 0x01000000);
            uc_ctl_remove_cache(uc, IRAM_BASE, IRAM_BASE + IRAM_SIZE);
            uc_ctl_remove_cache(uc, DRAM_BASE, DRAM_BASE + DRAM_WINDOW_SIZE);
            // Mode first, as at power-on: SVC, ARM state, IRQ/FIQ/async
            // aborts masked. SP is banked, so it is written in that mode.
            uint32_t reset_cpsr = 0x1D3;
            uint32_t reset_pc = IPL_LOAD_ADDR;
            uint32_t reset_sp = IPL_STACK_ADDR;
            uc_reg_write(uc, UC_ARM_REG_CPSR, &reset_cpsr);
            uc_reg_write(uc, UC_ARM_REG_SP,   &reset_sp);
            uc_reg_write(uc, UC_ARM_REG_PC,   &reset_pc);
            state.fb_addr = FB_BASE; // re-point display at the FB base
            state.emu_usec   = 0;
            state.insn_count = 0;
            state.bpmp_slept_us = 0;
            state.touch_phase = 0;
            state.paused = false;
            state.btn_power = false;
            state.btn_vol_up = false;
            state.btn_vol_down = false;
            g_auto = AutoScripts();
            input_script_restart(state);
            printf("[emu] Soft reboot complete (%s, DRAM wiped)\n",
                   cold ? "power cycle" : "SoC reset");
        }

        if (!state.paused) {
            uint32_t pc, cpsr;
            uc_reg_read(uc, UC_ARM_REG_PC, &pc);
            uc_reg_read(uc, UC_ARM_REG_CPSR, &cpsr);

            // ---- PIN recovery scripted navigation (Lockpick patch menu) ----
            // With EmuNAND grayed out (no emummc.ini), Lockpick's main menu
            // layout effectively becomes:
            //   0: Dump from SysNAND
            //   1: ---  (was EmuNAND, grayed)
            //   2: ---  (caption)
            //   3: Recover Parental PIN  ← target (single VOL_DOWN reaches it)
            // Stage 1: VOL_DOWN to navigate. Stage 2: POWER press-then-release
            // (just long enough for the menu to dispatch recover_pin), then
            // idle so recover_pin's final btn_wait blocks on us — keeps the
            // result text visible in the framebuffer when the run times out.
            if (auto_pin_recovery) {
                int &pin_stage = g_auto.pin_stage;
                uint64_t &pin_t = g_auto.pin_t;
                auto press_release = [&](std::atomic<bool> *btn,
                                          uint64_t hold_us,
                                          uint64_t cooldown_us) {
                    if (state.emu_usec - pin_t < hold_us) {
                        btn->store(true);
                    } else if (state.emu_usec - pin_t < hold_us + cooldown_us) {
                        btn->store(false);
                    } else {
                        pin_t = state.emu_usec;
                        ++pin_stage;
                    }
                };
                if (pin_t == 0 && state.emu_usec > 6000000) {
                    pin_t = state.emu_usec;
                    pin_stage = 1;
                    printf("[emu] Auto PIN recovery armed at emu_usec=%llu\n",
                           (unsigned long long)state.emu_usec);
                }
                int &last_logged_stage = g_auto.pin_logged_stage;
                if (pin_stage != last_logged_stage) {
                    printf("[emu] Auto PIN stage %d at emu_usec=%llu\n", pin_stage,
                           (unsigned long long)state.emu_usec);
                    last_logged_stage = pin_stage;
                }
                switch (pin_stage) {
                  case 1: press_release(&state.btn_vol_down, 1500000, 1500000); break;
                  case 2:
                    if (state.emu_usec - pin_t < 1500000) {
                        state.btn_power.store(true);
                    } else if (state.emu_usec - pin_t < 3000000) {
                        state.btn_power.store(false);
                    } else {
                        printf("[emu] Auto PIN recovery sequence done\n");
                        pin_t = state.emu_usec;
                        pin_stage = 3;
                    }
                    break;
                  case 3: break; // Idle — let recover_pin run to completion.
                  default: break;
                }
            }

            // ---- TegraExplorer scripted nav (recover_pin.te) ----
            // Sequence captured from a manual interactive run on the same
            // testcase (sw051/13.2.1 + TegraExplorer.bin) and replayed
            // verbatim — emu_usec is deterministic, so identical timestamps
            // hit the UI in the same state.
            //
            // Flow: POWER (skip "Grabbing keys... done") → 13×VOL_DOWN
            // (reach recover_pin.te in main menu) → POWER (enter script)
            // → 1×VOL_DOWN (select "Recover from sysmmc") → POWER (run).
            // After the last POWER we idle: returning to TE's main menu
            // means the script crashed/errored; otherwise the PIN stays
            // painted in the framebuffer.
            if (auto_te_script) {
                struct InputEv { uint64_t at_us; char btn; bool down; };
                static const InputEv te_events[] = {
                    // Skip "Grabbing keys... done"
                    {3480000,  'P', true},  {3640000,  'P', false},
                    // 13× VOL_DOWN to recover_pin.te
                    {10080000, 'D', true},  {10460000, 'D', false},
                    {10830000, 'D', true},  {11000000, 'D', false},
                    {11340000, 'D', true},  {11540000, 'D', false},
                    {11850000, 'D', true},  {12010000, 'D', false},
                    {12330000, 'D', true},  {12540000, 'D', false},
                    {12830000, 'D', true},  {13040000, 'D', false},
                    {13360000, 'D', true},  {13580000, 'D', false},
                    {13900000, 'D', true},  {14110000, 'D', false},
                    {14420000, 'D', true},  {14650000, 'D', false},
                    {14980000, 'D', true},  {15210000, 'D', false},
                    {15550000, 'D', true},  {15760000, 'D', false},
                    {16200000, 'D', true},  {16340000, 'D', false},
                    // POWER → enter recover_pin.te
                    {18110000, 'P', true},  {18310000, 'P', false},
                    // VOL_DOWN → select "Recover from sysmmc"
                    {21670000, 'D', true},  {21900000, 'D', false},
                    // POWER → run script
                    {22850000, 'P', true},  {23020000, 'P', false},
                };
                static const size_t te_n = sizeof(te_events)/sizeof(te_events[0]);
                size_t &te_idx = g_auto.te_idx;
                while (te_idx < te_n && state.emu_usec >= te_events[te_idx].at_us) {
                    const InputEv &ev = te_events[te_idx];
                    std::atomic<bool> *btn = (ev.btn == 'P') ? &state.btn_power
                                            : (ev.btn == 'D') ? &state.btn_vol_down
                                            : &state.btn_vol_up;
                    btn->store(ev.down);
                    printf("[auto-te] %s %s @emu_usec=%llu (event %zu/%zu)\n",
                           ev.down ? "DOWN" : "UP  ",
                           ev.btn == 'P' ? "POWER   " :
                           ev.btn == 'D' ? "VOL_DOWN" : "VOL_UP  ",
                           (unsigned long long)state.emu_usec, te_idx + 1, te_n);
                    te_idx++;
                    if (te_idx == te_n)
                        printf("[auto-te] sequence complete; idling for output\n");
                    // A time jump (FLOW_CTLR sleep) can make the release due
                    // in the same tick; the payload must see the press first.
                    if (ev.down)
                        break;
                }
            }

            // Scripted button input (--input-script).
            input_script_tick(state);

            // Touch injection for the Nyx GUI popup (eMMC Issues Warning).
            // Nyx initializes after the IPL stage; we inject a tap once enough
            // emulated time has passed for Nyx to load and render its dialog.
            // touch_x/y are portrait coordinates (720x1280 space); Nyx maps them
            // to landscape (1280x720) internally.
            // Default: (360, 640) = portrait center. Tune if the OK button
            // is not hit (e.g. try touch_x=490, touch_y=640 for the actual button).
            if (state.emu_usec > 3000000 && state.touch_phase == 0) {
                state.touch_phase = 1;
                printf("[emu] Touch injection armed: portrait (%d,%d)\n",
                       state.touch_x, state.touch_y);
            }

            // Run a batch of ARM instructions. With CPU0 up, the batch is cut
            // into lockstep slices: the BPMP runs a quantum, then CPU0 runs up
            // to the same emulated time. Scripted input and the display are
            // still serviced once per full batch, as before.
            uint64_t batch_end = state.insn_count + BATCH_INSTRUCTIONS;
            uint64_t parked_left = BATCH_INSTRUCTIONS;
            while (state.running && !state.paused &&
                   state.insn_count < batch_end) {
                if (state.bpmp_halted) {
                    // The BPMP parked itself for good after handing off to
                    // CPU0. It retires nothing; time runs on for CPU0 at the
                    // rate the BPMP's clock would have advanced it.
                    if (!ccplex_cpu0_running()) {
                        printf("[emu] BPMP halted and CPU0 stopped, shutting down emulator\n");
                        state.running = false;
                        break;
                    }
                    if (parked_left < CPU0_QUANTUM_INSTRUCTIONS)
                        break;
                    parked_left -= CPU0_QUANTUM_INSTRUCTIONS;
                    state.emu_usec += CPU0_QUANTUM_INSTRUCTIONS / 10;
                    state.bpmp_slept_us += CPU0_QUANTUM_INSTRUCTIONS / 10;
                    ccplex_run(&state, state.emu_usec);
                    if (state.reboot_requested.load())
                        break;
                    continue;
                }
                uint64_t budget = batch_end - state.insn_count;
                if (ccplex_cpu0_running() && budget > CPU0_QUANTUM_INSTRUCTIONS)
                    budget = CPU0_QUANTUM_INSTRUCTIONS;

                uc_reg_read(uc, UC_ARM_REG_PC, &pc);
                uc_reg_read(uc, UC_ARM_REG_CPSR, &cpsr);
                uint32_t start_addr = pc;
                if (cpsr & (1 << 5)) start_addr |= 1;

                uc_err err = bpmp_run(uc, &state, start_addr, budget);
                if (err != UC_ERR_OK) {
                    uint32_t error_pc;
                    uc_reg_read(uc, UC_ARM_REG_PC, &error_pc);
                    fprintf(stderr, "\n[emu] FATAL: Emulation error at PC=0x%08X: %s\n", error_pc, uc_strerror(err));
                    state.running = false;
                    break;
                }
                // CPU0 catches up to the BPMP's clock.
                ccplex_run(&state, state.emu_usec);
                if (state.reboot_requested.load())
                    break;
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Update display periodically
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_display_update);
        if (elapsed.count() >= DISPLAY_UPDATE_MS) {
            sdl_display_update(&state, uc);
            config_window_render(&state);
            console_window_render(&state);
            last_display_update = now;
        }
    }

    // Cleanup
    printf("\n[emu] Shutting down...\n");

    uint32_t final_pc;
    uc_reg_read(uc, UC_ARM_REG_PC, &final_pc);
    printf("[emu] Final PC: 0x%08X\n", final_pc);

    // Print the instruction that it halted on as well as the previous 8 bytes
    uint16_t final_insn[8];
    if (uc_mem_read(uc, (final_pc & ~1) - 8, final_insn, 16) == UC_ERR_OK) {
        printf("[emu] Final INSNS (-8): %04X %04X %04X %04X %04X %04X %04X %04X\n", 
            final_insn[0], final_insn[1], final_insn[2], final_insn[3],
            final_insn[4], final_insn[5], final_insn[6], final_insn[7]);
    }

    console_window_shutdown();
    config_window_shutdown();
    sdl_display_shutdown();
    ccplex_reset(&state);   // closes CPU0's engine, if one is up
    uc_close(uc);

    free(state.iram_ptr);
    free(g_low_ptr);
    // dram_ptr is a view into dram_low_ptr's block; only free the base once.
    zeroed_free(state.dram_low_ptr, DRAM_WINDOW_SIZE);
    free(state.payload_ptr);
    // fb_ptr points inside dram_ptr; do not free separately.

    printf("[emu] Done.\n");
    return 0;
}
