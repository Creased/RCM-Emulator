// Choosing a payload when the emulator is started without one.
//
// Double-clicking the emulator used to print a usage line to a console
// nobody sees and exit. Instead it now opens a window that takes a dropped
// file, with a key to bring up the system file dialog for people who would
// rather browse. Dragging a .bin onto the executable itself still works and
// still skips all of this - it arrives as argv[1].
//
// The window is deliberately created and destroyed before the emulator's own
// display comes up, so this cannot interfere with the framebuffer window or
// its event handling.

#include "payload_picker.h"

#include <SDL2/SDL.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>

#ifdef _WIN32
#include <windows.h>
#include <commdlg.h>
#endif

// System file dialog. Windows has one in the OS; on Linux there is no such
// thing without pulling in a toolkit, so shell out to whichever of the two
// usual helpers is installed and fall back to drag-and-drop if neither is.
static std::string native_file_dialog(void) {
#ifdef _WIN32
    char path[MAX_PATH] = {0};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner   = nullptr;
    ofn.lpstrFilter = "RCM payloads (*.bin)\0*.bin\0All files\0*.*\0";
    ofn.lpstrFile   = path;
    ofn.nMaxFile    = sizeof(path);
    ofn.lpstrTitle  = "Select an RCM payload";
    ofn.Flags       = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    if (GetOpenFileNameA(&ofn))
        return std::string(path);
    return std::string();
#else
    const char *cmds[] = {
        "zenity --file-selection --title='Select an RCM payload' "
            "--file-filter='RCM payloads | *.bin' --file-filter='All files | *' 2>/dev/null",
        "kdialog --getopenfilename . '*.bin|RCM payloads' 2>/dev/null",
    };
    for (const char *cmd : cmds) {
        FILE *p = popen(cmd, "r");
        if (!p)
            continue;
        char buf[4096] = {0};
        char *got = fgets(buf, sizeof(buf), p);
        int rc = pclose(p);
        if (got && rc == 0) {
            std::string s(buf);
            while (!s.empty() && (s.back() == '\n' || s.back() == '\r'))
                s.pop_back();
            if (!s.empty())
                return s;
        }
    }
    printf("[picker] no zenity or kdialog found - drag a file onto the window\n");
    return std::string();
#endif
}

std::string payload_picker_run(void) {
    // Test hook: run the whole window lifecycle, then return this path as if
    // it had been dropped. Lets the picker path - which is otherwise only
    // reachable by a human dragging a file - be exercised by a script.
    const char *forced = getenv("RCM_EMU_PICK");

    if (SDL_InitSubSystem(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "[picker] no video subsystem (%s)\n", SDL_GetError());
        return std::string();
    }

    SDL_Window *win = SDL_CreateWindow(
        "RCM Emulator - drop a payload",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 640, 220,
        SDL_WINDOW_SHOWN);
    if (!win) {
        fprintf(stderr, "[picker] no window (%s)\n", SDL_GetError());
        return std::string();
    }
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    // Drop events are off by default on some platforms.
    SDL_EventState(SDL_DROPFILE, SDL_ENABLE);

    printf("[picker] drop a payload on the window, or press O to browse"
           " (Esc quits)\n");
    fflush(stdout);

    std::string chosen;
    bool done = false;
    int forced_frames = forced ? 30 : 0;   // let the window really come up
    while (!done) {
        if (forced && --forced_frames <= 0) {
            chosen = forced;
            done = true;
        }
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {
            case SDL_QUIT:
                done = true;
                break;
            case SDL_DROPFILE:
                if (ev.drop.file) {
                    chosen = ev.drop.file;
                    SDL_free(ev.drop.file);
                    done = true;
                }
                break;
            case SDL_KEYDOWN:
                if (ev.key.keysym.sym == SDLK_ESCAPE) {
                    done = true;
                } else if (ev.key.keysym.sym == SDLK_o) {
                    // Raising the dialog while holding the window is fine on
                    // both platforms; the dialog is modal to itself only.
                    chosen = native_file_dialog();
                    if (!chosen.empty())
                        done = true;
                }
                break;
            default:
                break;
            }
        }

        // Draw a drop target: a dashed frame with an upload glyph inside,
        // so the window reads as "put a file here" rather than as a blank
        // panel. All of it is rectangles and one filled triangle - a font
        // would have to be vendored for a screen that lives a few seconds,
        // and the instruction is already on the title bar.
        if (ren) {
            SDL_SetRenderDrawColor(ren, 0x1B, 0x1B, 0x1B, 0xFF);
            SDL_RenderClear(ren);

            const int W = 640, H = 220;
            const int mx = 24, my = 20;           // margin of the frame
            SDL_SetRenderDrawColor(ren, 0x00, 0x8C, 0xC8, 0xFF);

            // Dashed border: 14 px of line, 10 px of gap.
            for (int x = mx; x < W - mx; x += 24) {
                SDL_Rect t = {x, my, 14, 2};
                SDL_Rect b = {x, H - my - 2, 14, 2};
                SDL_RenderFillRect(ren, &t);
                SDL_RenderFillRect(ren, &b);
            }
            for (int y = my; y < H - my; y += 24) {
                SDL_Rect l = {mx, y, 2, 14};
                SDL_Rect r = {W - mx - 2, y, 2, 14};
                SDL_RenderFillRect(ren, &l);
                SDL_RenderFillRect(ren, &r);
            }

            // Upload glyph: arrow out of a tray, centred.
            const int cx = W / 2, cy = H / 2 - 12;
            SDL_Rect stem = {cx - 6, cy - 6, 12, 40};
            SDL_RenderFillRect(ren, &stem);
            for (int i = 0; i < 26; i++) {        // filled triangle head
                SDL_Rect row = {cx - i, cy - 6 - 26 + i, 2 * i + 1, 1};
                SDL_RenderFillRect(ren, &row);
            }
            SDL_Rect tray_l = {cx - 46, cy + 46, 4, 22};
            SDL_Rect tray_r = {cx + 42, cy + 46, 4, 22};
            SDL_Rect tray_b = {cx - 46, cy + 64, 92, 4};
            SDL_RenderFillRect(ren, &tray_l);
            SDL_RenderFillRect(ren, &tray_r);
            SDL_RenderFillRect(ren, &tray_b);

            SDL_RenderPresent(ren);
        }
        SDL_Delay(16);
    }

    if (ren)
        SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    // Pump once so the window is really gone before the emulator's own opens.
    SDL_PumpEvents();

    if (!chosen.empty()) {
        printf("[picker] payload: %s\n", chosen.c_str());
        fflush(stdout);
    }
    return chosen;
}
