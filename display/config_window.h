#ifndef CONFIG_WINDOW_H
#define CONFIG_WINDOW_H

#include <SDL2/SDL.h>

struct EmuState;

// Hardware-tweak configuration window.
// Renders in a separate SDL window (toggle with the M key on the main window)
// using Dear ImGui. Edits write directly to atomic fields on EmuState; MMIO
// handlers in t210/mmio.cpp pick them up on the next register read.

bool config_window_init();
void config_window_shutdown();

// Called from the main SDL event loop. Returns true if the event targeted the
// config window and was consumed (caller should `continue` the loop).
bool config_window_handle_event(const SDL_Event &ev);

// Show/hide the config window.
void config_window_toggle();
bool config_window_is_visible();

// Build the ImGui frame and present. Cheap when hidden (returns immediately).
void config_window_render(EmuState *state);

// Returns the SDL_WindowID of the config window (0 if not yet created).
Uint32 config_window_id();

#endif // CONFIG_WINDOW_H
