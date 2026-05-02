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
#include "t210/memory_map.h"
#include "t210/mmio.h"
#include "display/sdl_display.h"
#include "display/config_window.h"

// ==================== Payload Loading ====================

static uint8_t *load_payload(const char *path, size_t *out_size) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "[error] Cannot open payload: %s\n", path);
        return nullptr;
    }

    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);

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

// Global for instruction tracing
static uint32_t pc_trace[2000];
static size_t pc_trace_idx = 0;
static uint32_t last_valid_block = 0;
static uc_hook trace_h, block_h;

static void trace_callback(uc_engine *uc, uint64_t address, uint32_t size, void *user_data) {
    static uint32_t last_pc = 0;
    if (address == last_pc) return;
    last_pc = (uint32_t)address;

    pc_trace[pc_trace_idx] = (uint32_t)address;
    pc_trace_idx = (pc_trace_idx + 1) % 2000;
    
    // NOP-slide detection
    if (address >= 0x40030000 && address < 0x41000000) {
        uint16_t insn = 0;
        if (uc_mem_read(uc, address, &insn, 2) == UC_ERR_OK && insn == 0) {
            static int nop_count = 0;
            if (++nop_count > 10) {
                printf("\n[emu] NOP-slide detected at 0x%08llX! Last valid block: 0x%08X\n", 
                       (unsigned long long)address, last_valid_block);
                uc_emu_stop(uc);
            }
        } else {
            // nop_count = 0; // would need a persistent counter
        }
    }
}

static void block_callback(uc_engine *uc, uint64_t address, uint32_t size, void *user_data) {
    if (address >= 0x40000000 && address < 0x40030000) {
        last_valid_block = (uint32_t)address;
    }
}

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

    // Setup hooks
    uc_hook_add(uc, &trace_h, UC_HOOK_CODE, (void*)trace_callback, nullptr, 1, 0);
    uc_hook_add(uc, &block_h, UC_HOOK_BLOCK, (void*)block_callback, nullptr, 1, 0);

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

    uc_mem_write(uc, IPL_LOAD_ADDR, payload, payload_size);

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

    // Setup instruction tracing
    uc_hook_add(uc, &trace_h, UC_HOOK_CODE, (void*)trace_callback, nullptr, 1, 0);

    // ---- Map DRAM Low (256MB @ 0x80000000) ----
    // Allocated host-side so soft reboot can memset() it (Nyx loads here and
    // its file-static SD/eMMC caches would otherwise survive across reboots).
    state->dram_low_ptr = (uint8_t *)calloc(1, 256 * 1024 * 1024);
    err = uc_mem_map_ptr(uc, DRAM_BASE, 256 * 1024 * 1024, UC_PROT_ALL, state->dram_low_ptr);
    if (err != UC_ERR_OK) {
        fprintf(stderr, "[error] Failed to map low DRAM: %s\n", uc_strerror(err));
        uc_close(uc);
        return nullptr;
    }
    printf("[emu] Mapped DRAM Low: 0x%08llX - 0x%08llX (256 MB)\n",
           (unsigned long long)DRAM_BASE,
           (unsigned long long)(DRAM_BASE + 256 * 1024 * 1024));

    // ---- Map DRAM High + FB (1GB @ 0xC0000000) ----
    // This covers 0xE5000000 and 0xF5A00000
    size_t high_dram_size = 1024 * 1024 * 1024;
    state->dram_ptr = (uint8_t *)calloc(1, high_dram_size);
    if (!state->dram_ptr) {
        fprintf(stderr, "[error] Failed to allocate high DRAM host buffer\n");
        return nullptr;
    }
    err = uc_mem_map_ptr(uc, 0xC0000000, high_dram_size, UC_PROT_ALL, state->dram_ptr);
    if (err != UC_ERR_OK) {
        fprintf(stderr, "[error] Failed to map high DRAM: %s\n", uc_strerror(err));
        return nullptr;
    }
    printf("[emu] Mapped DRAM High: 0xC0000000 - 0x%08llX (1024 MB)\n",
           (unsigned long long)(0xC0000000 + high_dram_size));

    // ---- Map Framebuffer pointer ----
    // FB_BASE is 0xF5A00000. Offset in 1GB block starting at 0xC0000000 is 0x35A00000.
    state->fb_ptr = state->dram_ptr + (FB_BASE - 0xC0000000);
    // Fill with hekate background color (0x1B1B1B)
    for (size_t i = 0; i < FB_SIZE; i += 4) {
        state->fb_ptr[i + 0] = 0x1B; // B
        state->fb_ptr[i + 1] = 0x1B; // G
        state->fb_ptr[i + 2] = 0x1B; // R
        state->fb_ptr[i + 3] = 0xFF; // A
    }
    state->fb_addr = FB_BASE;
    printf("[emu] Defined FB:   0x%08llX - 0x%08llX (%u MB)\n",
           (unsigned long long)FB_BASE,
           (unsigned long long)(FB_BASE + FB_SIZE),
           (unsigned)(FB_SIZE / (1024 * 1024)));

    // ---- Map low memory (16MB @ 0x0) ----
    // hekate seems to do a memset(0, ...) for clear screen if some ptr is NULL.
    uint8_t *low_ptr = (uint8_t *)calloc(1, 0x01000000);
    uc_mem_map_ptr(uc, 0, 0x01000000, UC_PROT_ALL, low_ptr);

    // ---- Nyx Storage (16MB @ 0xED000000) ----
    // Already mapped as part of 2GB DRAM chunk

    // ---- Map Peripherals (PWM, SDMMC, etc.) ----
    // Map individual pages to allow MMIO hooks
    // The mappings previously declared manually were moved here:
    // Some are mapped earlier in setup_emulation properly using FB_BASE e.g.
    uc_mem_map(uc, 0x7000A000, 0x1000, UC_PROT_ALL); // PWM
    uc_mem_map(uc, 0x700B0000, 0x1000, UC_PROT_ALL); // SDMMC1
    uc_mem_map(uc, 0x7000E000, 0x1000, UC_PROT_ALL); // RTC/PMC
    uc_mem_map(uc, 0x6000C000, 0x2000, UC_PROT_ALL); // SYSREG/APB_SEMAPH

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

    // ---- Setup MMIO hooks ----
    mmio_init(uc, state);
    printf("[emu] MMIO hooks registered\n");

    return uc;
}

// ==================== Main ====================

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <payload.bin> [--sd <sd.img>] [--boot0 <boot0.bin>] [--rawnand <rawnand_prefix>] [--prod-keys <prod.keys>]\n", argv[0]);
        fprintf(stderr, "\nControls:\n");
        fprintf(stderr, "  Arrow Up/Down = VOL+/VOL- buttons\n");
        fprintf(stderr, "  Enter         = POWER button\n");
        fprintf(stderr, "  Escape        = Quit\n");
        fprintf(stderr, "  F8            = Save raw guest FB to fb_dump_NNNN.raw\n");
        return 1;
    }

    printf("=== RCM Payload Emulator ===\n");
    printf("Using Unicorn Engine for ARM32 emulation\n\n");

    // Initialize emulator state
    EmuState state = {};

    // Load payload binary
    size_t payload_size = 0;
    uint8_t *payload = load_payload(argv[1], &payload_size);
    if (!payload) return 1;

    // Parse additional arguments for storage
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
    }

    if (sd_path) {
        state.sd_fd = open(sd_path, O_RDWR);
        if (state.sd_fd < 0) perror("[emu] Failed to open SD image");
        else printf("[emu] SD image opened: %s\n", sd_path);
    }
    if (boot0_path) {
        state.emmc_boot0_fd = open(boot0_path, O_RDWR);
        if (state.emmc_boot0_fd < 0) perror("[emu] Failed to open BOOT0 image");
        else printf("[emu] BOOT0 image opened: %s\n", boot0_path);
    }
    if (rawnand_prefix) {
        for (int i = 0; i < 16; i++) {
            char path[512];
            snprintf(path, sizeof(path), "%s.%02d", rawnand_prefix, i);
            int fd = open(path, O_RDWR);
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

    printf("\n[emu] Starting emulation...\n");
    printf("[emu] Press Escape to quit, M to toggle hardware config\n\n");

    // ---- Emulation loop ----
    // We run emulation in batches, interleaving with SDL event handling
    // and display updates to keep the UI responsive.

    const uint64_t BATCH_INSTRUCTIONS = 100000; // Instructions per batch
    const int DISPLAY_UPDATE_MS = 16;           // ~60 FPS

    auto last_display_update = std::chrono::steady_clock::now();

    while (state.running) {
        // Process SDL events (keyboard input, window close)
        if (!sdl_display_poll_events(&state, uc)) break;

        // Soft reboot: re-write the payload to IRAM, wipe DRAM (so Nyx and
        // the bootloader's file-static caches reset), reset PC/SP/clock and
        // re-prime the WDT cookie so Hekate's early boot skips Minerva again.
        if (state.reboot_requested.exchange(false)) {
            uc_emu_stop(uc);
            memset(state.dram_low_ptr, 0, 256 * 1024 * 1024);
            memset(state.dram_ptr,     0, 1024 * 1024 * 1024);
            uc_mem_write(uc, IPL_LOAD_ADDR, state.payload_ptr, state.payload_len);
            uint32_t wdt_magic = 0x544457;
            uc_mem_write(uc, 0x4003FF18, &wdt_magic, sizeof(wdt_magic));
            uint32_t reset_pc = IPL_LOAD_ADDR;
            uint32_t reset_sp = IPL_STACK_ADDR;
            uint32_t reset_cpsr = 0; // ARM mode, all flags clear
            uc_reg_write(uc, UC_ARM_REG_PC,   &reset_pc);
            uc_reg_write(uc, UC_ARM_REG_SP,   &reset_sp);
            uc_reg_write(uc, UC_ARM_REG_CPSR, &reset_cpsr);
            state.fb_addr = FB_BASE; // re-point display at the FB base
            state.emu_usec   = 0;
            state.insn_count = 0;
            state.touch_phase = 0;
            state.paused = false;
            printf("[emu] Soft reboot complete (DRAM wiped)\n");
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
                static int pin_stage = 0;
                static uint64_t pin_t = 0;
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
                static int last_logged_stage = -1;
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
                static size_t te_idx = 0;
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
                }
            }

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

            // Run a batch of ARM instructions
            uint32_t start_addr = pc;
            if (cpsr & (1 << 5)) start_addr |= 1;

            uc_err err = uc_emu_start(uc, start_addr, 0, 0, BATCH_INSTRUCTIONS);
            if (err != UC_ERR_OK) {
                uint32_t error_pc;
                uc_reg_read(uc, UC_ARM_REG_PC, &error_pc);
                fprintf(stderr, "\n[emu] FATAL: Emulation error at PC=0x%08X: %s\n", error_pc, uc_strerror(err));
                state.running = false;
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

    config_window_shutdown();
    sdl_display_shutdown();
    uc_close(uc);

    free(state.iram_ptr);
    free(state.dram_ptr);
    free(state.dram_low_ptr);
    free(state.payload_ptr);
    // fb_ptr points inside dram_ptr; do not free separately.

    printf("[emu] Done.\n");
    return 0;
}
