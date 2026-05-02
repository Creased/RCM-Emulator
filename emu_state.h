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
    std::atomic<bool> reboot_requested{false};

    // Payload kept around for soft reboot (re-write to IRAM and reset PC).
    uint8_t *payload_ptr = nullptr;
    size_t   payload_len = 0;

    // Backlight brightness (0-255).
    uint32_t backlight = 100;

    // Deterministic timer.
    uint64_t emu_usec = 0;
    uint64_t insn_count = 0;

    // DRAM pointer (host memory backing the emulated DRAM).
    uint8_t *dram_ptr = nullptr;     // High DRAM (0xC0000000+, 1 GB, includes FB)
    uint8_t *dram_low_ptr = nullptr; // Low DRAM (0x80000000+, 256 MB, holds Nyx)
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

    // ==================== Tweakable hardware values (config window) ====================
    // Battery fuel gauge (MAX17050, I2C_1 @ 0x36).
    // UI-friendly units; the I2C handler converts to MAX17050's raw register
    // encoding (cf. Hekate's max17050_get_property formulas).
    std::atomic<uint16_t> bat_soc_pct{80};          // RepSOC: percent (0-100)
    std::atomic<uint16_t> bat_vcell_mv{3950};       // VCELL:  mV
    std::atomic<int16_t>  bat_temp_c10{250};        // TEMP:   °C * 10 (UI unit), e.g. 250 -> 25.0°C
    std::atomic<int16_t>  bat_current_ma{0};        // Current: signed mA
    std::atomic<uint16_t> bat_capacity_mah{3500};   // RepCap:  mAh (current charge)
    std::atomic<uint16_t> bat_full_cap_mah{4000};   // FullCAP: mAh (learned full capacity)
    std::atomic<uint16_t> bat_design_cap_mah{4310}; // DesignCap: mAh (Switch nominal)
    std::atomic<uint16_t> bat_ocv_mv{3960};         // OCVInternal: mV (open-circuit)
    std::atomic<uint16_t> bat_v_empty_mv{3300};     // V_empty: mV
    std::atomic<uint16_t> bat_min_volt_mv{3500};    // MinMaxVolt low byte * 20
    std::atomic<uint16_t> bat_max_volt_mv{4200};    // MinMaxVolt high byte * 20
    std::atomic<uint8_t>  bat_age_pct{100};         // Age: percent (Age MSB)
    std::atomic<uint16_t> bat_cycles{0};            // Cycles count

    // Main PMIC OEM (MAX77620 CID4): 0x35 = Erista, 0x53 = Mariko, other = Unknown
    std::atomic<uint8_t>  pmic_otp{0x35};

    // Tegra X1 generation (drives APB_MISC_GP_HIDREV major nibble: 1 = T210
    // Erista, 2 = T210B01 Mariko). Hekate's h_cfg.t210b01 derives from this,
    // which controls the pkg1 OEM-header skip among other things.
    std::atomic<bool>     is_mariko{false};
    std::atomic<uint8_t>  pmic_silicon_rev{0};       // MAX77620 CID3 low nibble: "max77620 v%d"
    std::atomic<uint8_t>  cpu_pmic_version{0};       // MAX77621 CHIPID1: "max77621 v%d"

    // SD card insertion (GPIO Port Z bit 1 = 0 means inserted).
    std::atomic<bool>     sd_inserted{true};

    // SD card identity (returned for SDMMC1 CMD2 ALL_SEND_CID).
    // Hekate parses these out of the R2 response per bdk/storage/sdmmc.c
    // _sd_storage_parse_cid; the I2C handler builds the 16-byte CID payload
    // from these atomics each time CMD2 fires.
    std::atomic<uint8_t>  sd_cid_manfid{0x03};      // 0x03 = SanDisk
    std::atomic<uint16_t> sd_cid_oemid{0x5344};     // "SD"
    std::atomic<uint64_t> sd_cid_prod_name{0x3155445355ULL}; // "USDU1" (low byte first)
    std::atomic<uint8_t>  sd_cid_hwrev{1};          // 0-15
    std::atomic<uint8_t>  sd_cid_fwrev{0};          // 0-15
    std::atomic<uint32_t> sd_cid_serial{0xC0FFEE42};
    std::atomic<uint8_t>  sd_cid_month{10};         // 1-12
    std::atomic<uint16_t> sd_cid_year{2024};        // 2000-2255

    // eMMC identity (SDMMC4 CMD2). Layout per _mmc_storage_parse_cid (MMC v4):
    // 8-bit manfid, 8-bit oemid, 6-byte prod_name, 8-bit prv (BCD nibble.nibble),
    // 32-bit serial, 4-bit month, 4-bit year offset (year base = 2013 when
    // ext_csd.rev >= 5, which is what our EXT_CSD reports).
    std::atomic<uint8_t>  emmc_cid_manfid{0x90};         // 0x90 = SK Hynix
    std::atomic<uint8_t>  emmc_cid_oemid{0x01};
    std::atomic<uint64_t> emmc_cid_prod_name{0x326134474248ULL}; // "HBG4a2" (low byte first)
    std::atomic<uint8_t>  emmc_cid_prv{0x07};            // displayed as "0.7" (low.high)
    std::atomic<uint32_t> emmc_cid_serial{0x12345678};
    std::atomic<uint8_t>  emmc_cid_month{6};             // 1-12
    std::atomic<uint16_t> emmc_cid_year{2017};           // 2013-2025 (Hekate caps at 2025: see config_window.cpp)

    // SoC thermal sensor (TMP451, I2C5 @ 0x4C).
    std::atomic<int16_t>  soc_temp_c10{420};        // remote channel (SoC die): °C * 10
    std::atomic<int16_t>  pcb_temp_c10{350};        // local channel (PCB):     °C * 10

    // Charger (BQ24193, I2C1 @ 0x6B).
    std::atomic<uint8_t>  chg_vbus_stat{0};         // 0=none, 1=USB-SDP, 2=adapter, 3=OTG
    std::atomic<uint8_t>  chg_chrg_stat{0};         // 0=not charging, 1=pre, 2=fast, 3=done
    std::atomic<bool>     chg_power_good{false};

    // BQ24193 charger limits (encoded into regs 0x00/0x01/0x02/0x04/0x06).
    // mmio.cpp does the reverse-encoding from these decoded values; the user
    // sees real units, the chip sees the closest legal register value.
    std::atomic<uint16_t> chg_input_current_ma{2000};   // table: 100/150/500/900/1200/1500/2000/3000
    std::atomic<uint16_t> chg_input_voltage_mv{4280};   // 3880-5080 mV, 80 mV step
    std::atomic<uint16_t> chg_system_min_mv{3500};      // 3000-3700 mV, 100 mV step
    std::atomic<uint16_t> chg_fast_current_ma{1856};    // 512-4544 mA, 64 mA step
    std::atomic<uint16_t> chg_charge_voltage_mv{4208}; // 3504-4512 mV, 16 mV step
    std::atomic<uint8_t>  chg_thermal_c{60};            // 60 / 80 / 100 / 120 °C

    // USB-PD controller (BM92T36, I2C1 @ 0x18). One synthesized fixed PDO.
    std::atomic<bool>     usb_pd_inserted{false};
    std::atomic<uint16_t> usb_pd_voltage_mv{5000};
    std::atomic<uint16_t> usb_pd_amperage_ma{1500};

    // Fuse driver (FUSE @ 0x7000F800). Names per Hekate's bdk/soc/fuse.h.
    std::atomic<uint32_t> fuse_0x100{1};            // FUSE_PRODUCTION_MODE  (1 = production unit)
    std::atomic<uint32_t> fuse_0x110{0x83};         // FUSE_SKU_INFO         (0x83 = SKU_ODIN, required by Minerva)
    std::atomic<uint32_t> fuse_0x118{1785};         // FUSE_CPU_IDDQ_CALIB
    std::atomic<uint32_t> fuse_0x148{0x83000001};   // FUSE_OPT_FT_REV / mixed
    std::atomic<uint32_t> fuse_0x1A0{0x06};         // FUSE_OPT_VENDOR_CODE
    std::atomic<uint32_t> fuse_0x1D8{0x20};         // FUSE_RESERVED_ODM4    (bits 7:3 = dram_id, 0x20 = id 4)

    // DRAM mode register responses. Hekate reads these via the EMC controller
    // (sdram_read_mrx) to populate the "Vendor / Rev / Density" columns on
    // the HW & Fuses Info screen. Defaults match a Samsung K4F6E304HB-MGCH
    // 4 GB module on both channels.
    std::atomic<uint8_t>  dram_vendor{1};           // MR5 MAN_ID  (1=Samsung, 6=Hynix, 255=Micron)
    std::atomic<uint8_t>  dram_rev_id1{2};          // MR6 REV_ID1
    std::atomic<uint8_t>  dram_rev_id2{0};          // MR7 REV_ID2
    std::atomic<uint8_t>  dram_density{0x18};       // MR8 DENSITY (bits 5:2 = density code)
};

#endif // EMU_STATE_H
