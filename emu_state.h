#ifndef EMU_STATE_H
#define EMU_STATE_H

#include <cstdint>
#include <atomic>
#include <vector>
#include <string>

/*
 * Emulator State
 *
 * Shared state between CPU emulation, MMIO handlers, and display.
 */

#include <unicorn/unicorn.h>

struct EmuState {
    uc_engine *uc = nullptr;
    // Button state (updated by SDL keyboard events).
    std::atomic<bool> btn_vol_up{false};
    std::atomic<bool> btn_vol_down{false};
    std::atomic<bool> btn_power{false};

    // Touchscreen state (FTS4/STMFTS stub on I2C3 @ slave 0x49).
    // Updated by SDL mouse events; consumed by i2c3_*/STMFTS code on CPU thread.
    // Coordinates are in panel-raw space (X long axis 0..1264, Y short 0..704)
    // matching what touch.c expects before its rescaling.
    std::atomic<uint16_t> tc_x{0};
    std::atomic<uint16_t> tc_y{0};
    std::atomic<bool>     tc_pressed{false};
    std::atomic<bool>     tc_event_pending{false};
    std::atomic<uint8_t>  tc_event_op{0};   // 0x03=ENTER, 0x04=LEAVE, 0x05=MOTION
    uint8_t               tc_finger_id = 1; // FTS4 finger IDs are 1-indexed
    // last_rot mirrors the rotation last applied by sdl_display_update.
    // Read from the SDL event handler to invert the display→window transform.
    std::atomic<uint32_t> last_rot{0};
    std::atomic<uint32_t> last_out_w{1280};
    std::atomic<uint32_t> last_out_h{720};

    // Display state.
    uint64_t fb_addr = 0, pre_addr = 0;
    uint32_t fb_width = 720, pre_w = 720;
    uint32_t fb_height = 1280, pre_h = 1280;
    uint32_t fb_stride = 2880, pre_stride = 2880;
    uint32_t fb_swizzle = 0, pre_sw = 0;
    uint32_t fb_rotation = 0, pre_rot = 0;
    uint32_t pre_bh = 0; // block height in GOBs from DC surface-kind (0 = unset)
    uint32_t fb_sw_gobs = 80;
    uint32_t fb_bh = 0; // 0 = use display code default until DC surface-kind latches

    // DC window selection: tracks DC_CMD_DISPLAY_WINDOW_HEADER.
    // Bit 4 = Window A, Bit 5 = Window B, Bit 6 = Window C, Bit 7 = Window D.
    uint32_t dc_window_sel = 0x10; // Default: Window A
    // Saved Window A parameters (primary display surface).
    uint64_t winA_addr = 0;
    uint32_t winA_w = 720, winA_h = 1280, winA_stride = 2880;
    uint32_t winA_sw = 0, winA_rot = 0, winA_bh = 0;
    std::atomic<bool> display_dirty{false};
    std::atomic<bool> display_initialized{false};
    int32_t           manual_offset = 0;
    int32_t           rotation_override = -1; // -1 = Auto, 0=0, 1=90, 2=180, 3=270

    // Emulation control.
    std::atomic<bool> running{true};
    std::atomic<bool> paused{false};

    // Backlight brightness (0-255).
    uint32_t backlight = 100;

    // Deterministic timer.
    uint64_t emu_usec = 0;
    uint64_t insn_count = 0;

    // DRAM pointer (host memory backing the emulated DRAM).
    uint8_t *dram_ptr = nullptr;
    uint8_t *iram_ptr = nullptr;
    uint8_t *fb_ptr   = nullptr;

    // SDMMC1 State
    uint32_t sdmmc_arg = 0;
    uint32_t sdmmc_rsp[4] = {0};
    uint32_t sdmmc_norintsts = 0;
    uint32_t sdmmc_errintsts = 0;
    uint32_t sdmmc_sysad = 0;
    uint8_t  sdmmc_hostctl = 0;
    uint16_t sdmmc_blksize = 0;
    uint16_t sdmmc_blkcnt = 0;
    uint16_t sdmmc_trnmod = 0;
    uint64_t sdmmc_adma_addr = 0;

    // SDMMC4 State
    uint32_t sdmmc4_arg = 0;
    uint32_t sdmmc4_rsp[4] = {0};
    uint32_t sdmmc4_norintsts = 0;
    uint32_t sdmmc4_errintsts = 0;
    uint32_t sdmmc4_sysad = 0;
    uint8_t  sdmmc4_hostctl = 0;
    uint16_t sdmmc4_blksize = 0;
    uint16_t sdmmc4_blkcnt = 0;
    uint16_t sdmmc4_trnmod = 0;
    uint64_t sdmmc4_adma_addr = 0;

    // File-backed storage
    int sd_fd = -1;
    int emmc_boot0_fd = -1;
    int emmc_boot1_fd = -1;
    std::vector<int> emmc_gpp_fds;
    uint32_t emmc_partition = 0; // 0=GPP, 1=BOOT0, 2=BOOT1
    
    // Command state tracking
    bool last_cmd_was_55 = false;
    bool last_cmd4_was_55 = false;

    // Touch injection state (STMFTS FTS5 controller, I2C addr 0x49 on I2C1)
    // touch_phase: 0=idle, 1=send_down, 2=send_up, 3=done
    std::atomic<int> touch_phase{0};
    uint16_t touch_x = 360; // Portrait X coordinate (0-719)
    uint16_t touch_y = 640; // Portrait Y coordinate (0-1279)
};

#endif // EMU_STATE_H
