// In-emulator UART console.
//
// A second SDL window backed by its own ImGui context. The main window
// toggles it via the C key. The window:
//
//   1. Lets you pick which Tegra UART port to inspect (UART_A..UART_E).
//   2. Scrolls the selected port's TX history (every byte the payload
//      writes to that port's THR is appended to EmuState::uart_tx_log[idx]
//      by the mmio dispatcher).
//   3. Lets you type bytes that get pushed into EmuState::uart_rx_fifo[idx]
//      so the next CPU-side `uart_recv()` returns them. There's a "Send"
//      button plus shortcut buttons for the keys hwtest's pager listens
//      for (n / p / r / s / q).
//
// All TX append / RX pop is single-threaded — both the CPU emulation and
// ImGui rendering run on the main thread between SDL polls — so no
// locking. The TX log is trimmed if it grows past ~64 KB to keep imgui
// rendering snappy.

#include "console_window.h"
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
bool          g_sdl2_inited     = false;
bool          g_renderer_inited = false;

// Currently-displayed UART index (0..4).
int g_port = 1;  // UART_B by default — that's hwtest's debug port.

// Send-line buffer. Cleared on submit.
char g_send_buf[256] = {0};

const char *kPortNames[] = {"UART_A", "UART_B", "UART_C", "UART_D", "UART_E"};

void inject_bytes(EmuState *state, const uint8_t *data, size_t n)
{
    if (g_port < 0 || g_port >= (int)EmuState::N_UARTS) return;
    auto &fifo = state->uart_rx_fifo[g_port];
    for (size_t i = 0; i < n; i++)
        fifo.push_back(data[i]);
}

void inject_str(EmuState *state, const char *s)
{
    inject_bytes(state, (const uint8_t *)s, strlen(s));
}

void build_ui(EmuState *state)
{
    int win_w = 0, win_h = 0;
    SDL_GetWindowSize(g_window, &win_w, &win_h);
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2((float)win_w, (float)win_h));
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoMove    | ImGuiWindowFlags_NoCollapse |
                             ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::Begin("##console", nullptr, flags);

    // --- Top row: port selector + clear button ---
    ImGui::SetNextItemWidth(140);
    ImGui::Combo("Port", &g_port, kPortNames, IM_ARRAYSIZE(kPortNames));
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        state->uart_tx_log[g_port].clear();
    }
    ImGui::SameLine();
    ImGui::Text("rx fifo: %d byte(s)", (int)state->uart_rx_fifo[g_port].size());

    // --- Scrolling output area ---
    ImGui::Separator();
    {
        const float input_h = ImGui::GetFrameHeightWithSpacing() * 3 + 4;
        ImGui::BeginChild("scroll", ImVec2(0, -input_h), true,
                          ImGuiWindowFlags_HorizontalScrollbar);
        const std::string &log = state->uart_tx_log[g_port];
        if (!log.empty()) {
            ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0xCC, 0xCC, 0xCC, 0xFF));
            ImGui::TextUnformatted(log.data(), log.data() + log.size());
            ImGui::PopStyleColor();
        }
        // Auto-scroll to bottom unless the user has scrolled up manually.
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f)
            ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
    }

    // --- Send line + shortcuts ---
    ImGui::Separator();
    bool focus_input = false;
    if (ImGui::InputText("##send", g_send_buf, sizeof(g_send_buf),
                         ImGuiInputTextFlags_EnterReturnsTrue)) {
        if (g_send_buf[0]) {
            inject_str(state, g_send_buf);
            g_send_buf[0] = 0;
        }
        focus_input = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Send")) {
        if (g_send_buf[0]) {
            inject_str(state, g_send_buf);
            g_send_buf[0] = 0;
        }
        focus_input = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("+ LF")) {
        if (g_send_buf[0]) {
            inject_str(state, g_send_buf);
            g_send_buf[0] = 0;
        }
        const uint8_t lf = '\n';
        inject_bytes(state, &lf, 1);
        focus_input = true;
    }

    // Shortcut row — hwtest's pager keys.
    ImGui::TextDisabled("Shortcuts:");
    ImGui::SameLine();
    static const struct { const char *label; char key; } kShortcuts[] = {
        {"n (next)",    'n'},
        {"p (prev)",    'p'},
        {"r (refresh)", 'r'},
        {"s (save)",    's'},
        {"q (off)",     'q'},
        {"CR",          '\r'},
        {"LF",          '\n'},
    };
    for (size_t i = 0; i < sizeof(kShortcuts) / sizeof(kShortcuts[0]); i++) {
        if (i > 0) ImGui::SameLine();
        if (ImGui::Button(kShortcuts[i].label)) {
            uint8_t b = (uint8_t)kShortcuts[i].key;
            inject_bytes(state, &b, 1);
        }
    }

    if (focus_input)
        ImGui::SetKeyboardFocusHere(-1);

    ImGui::End();
}

} // namespace

bool console_window_init()
{
    g_window = SDL_CreateWindow("rcm_emu - UART Console",
                                SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                720, 540,
                                SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE);
    if (!g_window) {
        fprintf(stderr, "[console_window] SDL_CreateWindow failed: %s\n", SDL_GetError());
        return false;
    }
    g_renderer = SDL_CreateRenderer(g_window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!g_renderer) {
        fprintf(stderr, "[console_window] SDL_CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(g_window);
        g_window = nullptr;
        return false;
    }

    IMGUI_CHECKVERSION();
    g_imgui = ImGui::CreateContext();
    ImGui::SetCurrentContext(g_imgui);
    ImGui::StyleColorsDark();
    ImGuiIO &io = ImGui::GetIO();
    io.IniFilename = nullptr;  // don't write a sibling imgui.ini

    if (!ImGui_ImplSDL2_InitForSDLRenderer(g_window, g_renderer)) {
        fprintf(stderr, "[console_window] ImGui SDL2 backend init failed\n");
        ImGui::DestroyContext(g_imgui);
        g_imgui = nullptr;
        return false;
    }
    g_sdl2_inited = true;
    if (!ImGui_ImplSDLRenderer2_Init(g_renderer)) {
        fprintf(stderr, "[console_window] ImGui SDLRenderer2 backend init failed\n");
        ImGui_ImplSDL2_Shutdown();
        g_sdl2_inited = false;
        ImGui::DestroyContext(g_imgui);
        g_imgui = nullptr;
        return false;
    }
    g_renderer_inited = true;

    g_window_id = SDL_GetWindowID(g_window);
    return true;
}

void console_window_shutdown()
{
    if (g_imgui) {
        ImGui::SetCurrentContext(g_imgui);
        // Backend Shutdown asserts on `bd != nullptr`. Check the per-context
        // backend pointers directly so we never call Shutdown on a context
        // whose backend data was somehow already cleared (defensive — this
        // handles the case where another ImGui context-switch elsewhere
        // perturbed the global state).
        ImGuiIO &io = ImGui::GetIO();
        if (io.BackendRendererUserData) {
            ImGui_ImplSDLRenderer2_Shutdown();
        }
        if (io.BackendPlatformUserData) {
            ImGui_ImplSDL2_Shutdown();
        }
        g_renderer_inited = false;
        g_sdl2_inited     = false;
        ImGui::DestroyContext(g_imgui);
        g_imgui = nullptr;
    }
    if (g_renderer) {
        SDL_DestroyRenderer(g_renderer);
        g_renderer = nullptr;
    }
    if (g_window) {
        SDL_DestroyWindow(g_window);
        g_window = nullptr;
    }
    g_window_id = 0;
}

bool console_window_handle_event(const SDL_Event &ev)
{
    if (!g_imgui) return false;
    ImGui::SetCurrentContext(g_imgui);
    ImGui_ImplSDL2_ProcessEvent(&ev);

    // Hide on close.
    if (ev.type == SDL_WINDOWEVENT &&
        ev.window.windowID == g_window_id &&
        ev.window.event == SDL_WINDOWEVENT_CLOSE) {
        SDL_HideWindow(g_window);
        g_visible = false;
    }
    return true;
}

void console_window_toggle()
{
    if (!g_window) return;
    g_visible = !g_visible;
    if (g_visible) SDL_ShowWindow(g_window);
    else           SDL_HideWindow(g_window);
}

bool console_window_is_visible() { return g_visible; }

void console_window_render(EmuState *state)
{
    if (!g_visible || !g_imgui) return;

    ImGui::SetCurrentContext(g_imgui);
    SDL_SetRenderDrawColor(g_renderer, 0x10, 0x10, 0x14, 0xFF);
    SDL_RenderClear(g_renderer);

    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    build_ui(state);

    ImGui::Render();
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), g_renderer);
    SDL_RenderPresent(g_renderer);
}

Uint32 console_window_id() { return g_window_id; }
