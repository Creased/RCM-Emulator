// Hardware-tweak configuration window.
// See config_window.h for the public contract.

#include "config_window.h"
#include "emu_state.h"

#include "imgui.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_sdlrenderer2.h"

#include <cstdio>
#include <cstring>

namespace {

SDL_Window   *g_window   = nullptr;
SDL_Renderer *g_renderer = nullptr;
ImGuiContext *g_imgui    = nullptr;
bool          g_visible  = false;
Uint32        g_window_id = 0;

// Reset every tweakable to its EmuState constructor default.
// Kept as one function so the body and emu_state.h stay obviously in sync.
void reset_to_defaults(EmuState *s) {
    s->bat_soc_pct       = 80;
    s->bat_vcell_mv      = 3950;
    s->bat_temp_c10      = 250;
    s->bat_current_ma    = 0;
    s->bat_capacity_mah  = 3500;
    s->bat_full_cap_mah  = 4000;
    s->bat_design_cap_mah = 4310;
    s->bat_ocv_mv        = 3960;
    s->bat_v_empty_mv    = 3300;
    s->bat_min_volt_mv   = 3500;
    s->bat_max_volt_mv   = 4200;
    s->bat_age_pct       = 100;
    s->bat_cycles        = 0;
    s->soc_temp_c10      = 420;
    s->pcb_temp_c10      = 350;
    s->chg_vbus_stat     = 0;
    s->chg_chrg_stat     = 0;
    s->chg_power_good    = false;
    s->chg_input_current_ma  = 2000;
    s->chg_input_voltage_mv  = 4280;
    s->chg_system_min_mv     = 3500;
    s->chg_fast_current_ma   = 1856;
    s->chg_charge_voltage_mv = 4208;
    s->chg_thermal_c         = 60;
    s->usb_pd_inserted   = false;
    s->usb_pd_voltage_mv = 5000;
    s->usb_pd_amperage_ma = 1500;
    s->pmic_otp          = 0x35;
    s->pmic_silicon_rev  = 0;
    s->cpu_pmic_version  = 0;
    s->sd_inserted       = true;
    s->fuse_0x100        = 1;
    s->fuse_0x110        = 0x83;
    s->fuse_0x1A0        = 0x06;
    s->fuse_0x148        = 0x83000001;
    s->fuse_0x118        = 1785;
    s->backlight         = 100;
    s->rotation_override = -1;
}

// Helper: int slider that reads/writes an atomic.
template <typename T>
bool atomic_slider_int(const char *label, std::atomic<T> &a, int lo, int hi, const char *fmt = "%d") {
    int v = (int)a.load();
    if (ImGui::SliderInt(label, &v, lo, hi, fmt)) {
        a.store((T)v);
        return true;
    }
    return false;
}

// Helper: float slider for a temperature stored as °C × 10. The atomic
// holds tenths so MAX17050/TMP451 register encodings stay simple, but the
// user sees the actual decimal Celsius value.
template <typename T>
bool atomic_slider_temp(const char *label, std::atomic<T> &a, float lo_c, float hi_c) {
    float v = (float)a.load() / 10.0f;
    if (ImGui::SliderFloat(label, &v, lo_c, hi_c, "%.1f \xC2\xB0""C")) {
        a.store((T)(v * 10.0f));
        return true;
    }
    return false;
}

// Helper: hex input that reads/writes a uint32 atomic.
bool atomic_hex_input(const char *label, std::atomic<uint32_t> &a) {
    uint32_t v = a.load();
    if (ImGui::InputScalar(label, ImGuiDataType_U32, &v, nullptr, nullptr, "%08X",
                           ImGuiInputTextFlags_CharsHexadecimal)) {
        a.store(v);
        return true;
    }
    return false;
}

void build_ui(EmuState *state) {
    // Fill the OS window with a single ImGui window (no decorations).
    int win_w = 0, win_h = 0;
    SDL_GetWindowSize(g_window, &win_w, &win_h);
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2((float)win_w, (float)win_h));
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoMove    | ImGuiWindowFlags_NoCollapse |
                             ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::Begin("##cfg", nullptr, flags);

    if (ImGui::CollapsingHeader("Battery (MAX17050)", ImGuiTreeNodeFlags_DefaultOpen)) {
        atomic_slider_int<uint16_t>("SOC (%)",          state->bat_soc_pct,        0, 100, "%d %%");
        atomic_slider_int<uint16_t>("Voltage (mV)",     state->bat_vcell_mv,    3000, 4400, "%d mV");
        atomic_slider_int<uint16_t>("OCV (mV)",         state->bat_ocv_mv,      3000, 4400, "%d mV");
        atomic_slider_temp<int16_t> ("Temp",             state->bat_temp_c10,   -20.0f, 80.0f);

        int cur = (int)state->bat_current_ma.load();
        if (ImGui::SliderInt("Current (mA)", &cur, -3000, 3000, "%d mA")) {
            state->bat_current_ma.store((int16_t)cur);
        }
        atomic_slider_int<uint16_t>("RepCap (mAh)",     state->bat_capacity_mah,   0, 6000, "%d mAh");
        atomic_slider_int<uint16_t>("FullCAP (mAh)",    state->bat_full_cap_mah,   0, 6000, "%d mAh");
        atomic_slider_int<uint16_t>("DesignCap (mAh)",  state->bat_design_cap_mah, 0, 6000, "%d mAh");
        atomic_slider_int<uint16_t>("V_empty (mV)",     state->bat_v_empty_mv,  2500, 3800, "%d mV");
        atomic_slider_int<uint16_t>("Min volt (mV)",    state->bat_min_volt_mv, 2500, 4400, "%d mV");
        atomic_slider_int<uint16_t>("Max volt (mV)",    state->bat_max_volt_mv, 2500, 4400, "%d mV");
        atomic_slider_int<uint8_t> ("Age (%)",          state->bat_age_pct,        0, 100, "%d %%");
        atomic_slider_int<uint16_t>("Cycles",           state->bat_cycles,         0, 2000);
    }

    if (ImGui::CollapsingHeader("Thermal (TMP451)", ImGuiTreeNodeFlags_DefaultOpen)) {
        atomic_slider_temp<int16_t>("SoC temp", state->soc_temp_c10, 0.0f, 110.0f);
        atomic_slider_temp<int16_t>("PCB temp", state->pcb_temp_c10, 0.0f,  80.0f);
    }

    if (ImGui::CollapsingHeader("PMIC (MAX77620 / MAX77621)", ImGuiTreeNodeFlags_DefaultOpen)) {
        const char *otp_items[] = {"Erista (0x35)", "Mariko (0x53)"};
        int idx = (state->pmic_otp.load() == 0x53) ? 1 : 0;
        if (ImGui::Combo("OTP", &idx, otp_items, IM_ARRAYSIZE(otp_items))) {
            state->pmic_otp.store(idx == 1 ? 0x53 : 0x35);
        }
        atomic_slider_int<uint8_t>("MAX77620 silicon rev", state->pmic_silicon_rev,    0, 15);
        atomic_slider_int<uint8_t>("MAX77621 chipid",      state->cpu_pmic_version,    0, 15);
    }

    if (ImGui::CollapsingHeader("Charger (BQ24193)", ImGuiTreeNodeFlags_DefaultOpen)) {
        const char *vbus_items[] = {"None", "USB SDP", "Adapter", "OTG"};
        int vbus = (int)state->chg_vbus_stat.load();
        if (ImGui::Combo("VBUS", &vbus, vbus_items, IM_ARRAYSIZE(vbus_items))) {
            state->chg_vbus_stat.store((uint8_t)vbus);
        }
        const char *chrg_items[] = {"Not charging", "Pre-charge", "Fast charge", "Done"};
        int chrg = (int)state->chg_chrg_stat.load();
        if (ImGui::Combo("Charge state", &chrg, chrg_items, IM_ARRAYSIZE(chrg_items))) {
            state->chg_chrg_stat.store((uint8_t)chrg);
        }
        bool pg = state->chg_power_good.load();
        if (ImGui::Checkbox("Power good", &pg)) {
            state->chg_power_good.store(pg);
        }

        // ---- Charger limits (encoded into BQ24193 regs 0x00/0x01/0x02/0x04/0x06) ----
        // InputCurrentLimit is a quantized 8-value table in the chip; expose it
        // as a Combo so the displayed value always matches what Hekate reads back.
        const char *iinlim_items[] = {"100 mA","150 mA","500 mA","900 mA","1200 mA","1500 mA","2000 mA","3000 mA"};
        const uint16_t iinlim_vals[] = {100,150,500,900,1200,1500,2000,3000};
        int iidx = 6;
        for (int i = 0; i < 8; i++) if (state->chg_input_current_ma.load() == iinlim_vals[i]) { iidx = i; break; }
        if (ImGui::Combo("Input current limit", &iidx, iinlim_items, IM_ARRAYSIZE(iinlim_items))) {
            state->chg_input_current_ma.store(iinlim_vals[iidx]);
        }
        // The four remaining limits are linearly quantized; sliders are fine.
        atomic_slider_int<uint16_t>("Input voltage limit (mV)",  state->chg_input_voltage_mv,  3880, 5080, "%d mV");
        atomic_slider_int<uint16_t>("System min voltage (mV)",   state->chg_system_min_mv,     3000, 3700, "%d mV");
        atomic_slider_int<uint16_t>("Fast charge current (mA)",  state->chg_fast_current_ma,    512, 4544, "%d mA");
        atomic_slider_int<uint16_t>("Charge voltage limit (mV)", state->chg_charge_voltage_mv, 3504, 4512, "%d mV");

        const char *therm_items[] = {"60 \xC2\xB0""C","80 \xC2\xB0""C","100 \xC2\xB0""C","120 \xC2\xB0""C"};
        const uint8_t therm_vals[] = {60, 80, 100, 120};
        int tidx = 0;
        for (int i = 0; i < 4; i++) if (state->chg_thermal_c.load() == therm_vals[i]) { tidx = i; break; }
        if (ImGui::Combo("Thermal regulation", &tidx, therm_items, IM_ARRAYSIZE(therm_items))) {
            state->chg_thermal_c.store(therm_vals[tidx]);
        }
    }

    if (ImGui::CollapsingHeader("Storage")) {
        bool ins = state->sd_inserted.load();
        if (ImGui::Checkbox("SD card inserted", &ins)) {
            state->sd_inserted.store(ins);
        }
    }

    if (ImGui::CollapsingHeader("USB-PD (BM92T36)", ImGuiTreeNodeFlags_DefaultOpen)) {
        bool ins = state->usb_pd_inserted.load();
        if (ImGui::Checkbox("Cable inserted", &ins)) {
            state->usb_pd_inserted.store(ins);
        }
        atomic_slider_int<uint16_t>("PDO voltage (mV)",  state->usb_pd_voltage_mv,  5000, 20000, "%d mV");
        atomic_slider_int<uint16_t>("PDO amperage (mA)", state->usb_pd_amperage_ma,  500,  3000, "%d mA");
    }

    if (ImGui::CollapsingHeader("Display")) {
        int bl = (int)state->backlight;
        if (ImGui::SliderInt("Backlight", &bl, 0, 255)) {
            state->backlight = (uint32_t)bl;
        }
        const char *rot_items[] = {"Auto", "0\xC2\xB0", "90\xC2\xB0", "180\xC2\xB0", "270\xC2\xB0"};
        int rot_idx = state->rotation_override + 1; // -1..3 -> 0..4
        if (ImGui::Combo("Rotation", &rot_idx, rot_items, IM_ARRAYSIZE(rot_items))) {
            state->rotation_override = rot_idx - 1;
        }
    }

    if (ImGui::CollapsingHeader("Fuses (advanced)")) {
        atomic_hex_input("FUSE 0x100", state->fuse_0x100);
        atomic_hex_input("FUSE 0x110 (SKU)", state->fuse_0x110);
        atomic_hex_input("FUSE 0x118 (ID)",  state->fuse_0x118);
        atomic_hex_input("FUSE 0x148",       state->fuse_0x148);
        atomic_hex_input("FUSE 0x1A0",       state->fuse_0x1A0);
    }

    if (ImGui::CollapsingHeader("Emulation")) {
        bool paused = state->paused.load();
        if (ImGui::Checkbox("Paused", &paused)) {
            state->paused.store(paused);
        }
        ImGui::Text("emu_usec:    %llu", (unsigned long long)state->emu_usec);
        ImGui::Text("insn_count:  %llu", (unsigned long long)state->insn_count);

        ImGui::Separator();
        ImGui::TextDisabled("Buttons (visual; SDL keys still work):");
        bool vu = state->btn_vol_up.load();
        bool vd = state->btn_vol_down.load();
        bool pw = state->btn_power.load();
        if (ImGui::Checkbox("VOL+",  &vu)) state->btn_vol_up.store(vu);
        ImGui::SameLine();
        if (ImGui::Checkbox("VOL-",  &vd)) state->btn_vol_down.store(vd);
        ImGui::SameLine();
        if (ImGui::Checkbox("POWER", &pw)) state->btn_power.store(pw);
    }

    ImGui::Separator();
    if (ImGui::Button("Reset to defaults")) {
        reset_to_defaults(state);
    }

    ImGui::End();
}

} // namespace

bool config_window_init() {
    g_window = SDL_CreateWindow("rcm_emu - Hardware Config",
                                SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                480, 720,
                                SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE);
    if (!g_window) {
        fprintf(stderr, "[config_window] SDL_CreateWindow failed: %s\n", SDL_GetError());
        return false;
    }
    g_renderer = SDL_CreateRenderer(g_window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!g_renderer) {
        fprintf(stderr, "[config_window] SDL_CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(g_window);
        g_window = nullptr;
        return false;
    }

    IMGUI_CHECKVERSION();
    g_imgui = ImGui::CreateContext();
    ImGui::StyleColorsDark();

    if (!ImGui_ImplSDL2_InitForSDLRenderer(g_window, g_renderer)) {
        fprintf(stderr, "[config_window] ImGui SDL2 backend init failed\n");
        ImGui::DestroyContext(g_imgui);
        g_imgui = nullptr;
        return false;
    }
    if (!ImGui_ImplSDLRenderer2_Init(g_renderer)) {
        fprintf(stderr, "[config_window] ImGui SDLRenderer2 backend init failed\n");
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext(g_imgui);
        g_imgui = nullptr;
        return false;
    }

    g_window_id = SDL_GetWindowID(g_window);
    g_visible = false;
    return true;
}

void config_window_shutdown() {
    if (g_imgui) {
        ImGui_ImplSDLRenderer2_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext(g_imgui);
        g_imgui = nullptr;
    }
    if (g_renderer) { SDL_DestroyRenderer(g_renderer); g_renderer = nullptr; }
    if (g_window)   { SDL_DestroyWindow(g_window);     g_window   = nullptr; }
    g_window_id = 0;
    g_visible = false;
}

bool config_window_handle_event(const SDL_Event &ev) {
    if (!g_imgui) return false;
    ImGui::SetCurrentContext(g_imgui);
    ImGui_ImplSDL2_ProcessEvent(&ev);

    // Hide on close-button click rather than tearing the window down — the M
    // key flips it back on.
    if (ev.type == SDL_WINDOWEVENT &&
        ev.window.windowID == g_window_id &&
        ev.window.event == SDL_WINDOWEVENT_CLOSE) {
        SDL_HideWindow(g_window);
        g_visible = false;
    }
    return true;
}

void config_window_toggle() {
    if (!g_window) return;
    g_visible = !g_visible;
    if (g_visible) {
        SDL_ShowWindow(g_window);
        SDL_RaiseWindow(g_window);
    } else {
        SDL_HideWindow(g_window);
    }
}

bool config_window_is_visible() { return g_visible; }
Uint32 config_window_id()       { return g_window_id; }

void config_window_render(EmuState *state) {
    if (!g_visible || !g_imgui) return;

    ImGui::SetCurrentContext(g_imgui);
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    build_ui(state);

    ImGui::Render();
    SDL_SetRenderDrawColor(g_renderer, 30, 30, 30, 255);
    SDL_RenderClear(g_renderer);
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), g_renderer);
    SDL_RenderPresent(g_renderer);
}
