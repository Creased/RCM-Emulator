#ifndef SDL_DISPLAY_H
#define SDL_DISPLAY_H

#include <unicorn/unicorn.h>

struct EmuState;

bool sdl_display_init();
void sdl_display_update(EmuState *state, uc_engine *uc);
bool sdl_display_poll_events(EmuState *state, uc_engine *uc);
void sdl_display_shutdown();

#endif // SDL_DISPLAY_H
