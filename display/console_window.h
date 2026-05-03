#ifndef CONSOLE_WINDOW_H
#define CONSOLE_WINDOW_H

#include <SDL2/SDL.h>

struct EmuState;

// UART console window. A second SDL window driven by Dear ImGui that lets
// the user watch any of the five Tegra UART ports' TX history scroll by and
// inject bytes into the corresponding RX FIFO. Toggled with the C key on
// the main window.

bool   console_window_init();
void   console_window_shutdown();
bool   console_window_handle_event(const SDL_Event &ev);
void   console_window_toggle();
bool   console_window_is_visible();
void   console_window_render(EmuState *state);
Uint32 console_window_id();

#endif // CONSOLE_WINDOW_H
