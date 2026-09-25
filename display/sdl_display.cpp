#include "sdl_display.h"
#include "config_window.h"
#include "console_window.h"
#include "../emu_state.h"
#include "../t210/memory_map.h"

#include <SDL2/SDL.h>
#include <cstdio>
#include <cstring>
#include <vector>

// Returns true only for events that carry a windowID matching the given
// auxiliary window's id. Generic so the event router can use it for both
// the config window (M key) and the UART console (C key).
static bool event_targets_window(const SDL_Event &ev, Uint32 wid) {
    if (wid == 0) return false;
    switch (ev.type) {
    case SDL_WINDOWEVENT:      return ev.window.windowID    == wid;
    case SDL_KEYDOWN:
    case SDL_KEYUP:            return ev.key.windowID       == wid;
    case SDL_TEXTINPUT:        return ev.text.windowID      == wid;
    case SDL_MOUSEBUTTONDOWN:
    case SDL_MOUSEBUTTONUP:    return ev.button.windowID    == wid;
    case SDL_MOUSEMOTION:      return ev.motion.windowID    == wid;
    case SDL_MOUSEWHEEL:       return ev.wheel.windowID     == wid;
    default:                   return false;
    }
}

static SDL_Window *window = nullptr;
static SDL_Renderer *renderer = nullptr;
static SDL_Texture *texture = nullptr;
// The last converted frame (ARGB8888, out_w x out_h), reused across updates.
static std::vector<uint32_t> g_frame;
static int g_swizzle_override =
    -1; // -1 = Auto, 0 = Pitch Linear, 2 = Block Linear

bool sdl_display_init() {
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) < 0)
    return false;
  window = SDL_CreateWindow("RCM Payload Emulator", SDL_WINDOWPOS_CENTERED,
                            SDL_WINDOWPOS_CENTERED, 720, 1280,
                            SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
  if (!window)
    return false;
  // No PRESENTVSYNC. The CPU runs on this same thread and the main loop
  // already paces redraws to ~60 Hz by wall clock; with vsync on, each
  // SDL_RenderPresent could block the emulated CPU for up to a whole frame.
  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  if (!renderer)
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
  if (!renderer)
    return false;
  printf("[display] SDL2 simple compositor initialized\n");
  return true;
}

void sdl_display_update(EmuState *state, uc_engine *uc) {
  if (!renderer)
    return;

  uint32_t width = state->fb_width, height = state->fb_height,
           stride = state->fb_stride;
  if (width < 16 || height < 16)
    return;

  // If the current fb_addr is empty but we have a saved Window A with
  // content, prefer the Window A surface. This handles the case where
  // Hekate's TUI renders to IPL_FB (0xF5A00000) but the DC was repointed
  // to NYX_FB (0xF6200000) by a failed Nyx init sequence.
  uint64_t eff_addr = state->fb_addr;
  if (state->winA_addr && state->winA_addr != eff_addr) {
    // Quick check: sample a few dwords from fb_addr.
    uint32_t sample[4] = {0};
    uc_mem_read(uc, eff_addr, sample, sizeof(sample));
    bool empty = (sample[0] == 0 && sample[1] == 0 &&
                  sample[2] == 0 && sample[3] == 0);
    if (empty) {
      // Check if winA_addr has content.
      uc_mem_read(uc, state->winA_addr, sample, sizeof(sample));
      if (sample[0] != 0 || sample[1] != 0 ||
          sample[2] != 0 || sample[3] != 0) {
        eff_addr = state->winA_addr;
        width = state->winA_w;
        height = state->winA_h;
        stride = state->winA_stride;
      }
    }
  }

  // Helper: is the address inside the IPL framebuffer carveout?
  auto addr_in_ipl_fb = [](uint64_t a) -> bool {
    return a >= FB_BASE && a < FB_BASE + FB_SIZE;
  };

  // Auto-detect first; a manual override is applied on top further down.
  int rot = -1;

  uint32_t sw = state->fb_swizzle;
  uint32_t eff_stride = stride;
  uint32_t bpp = 4;
  uint32_t bh = state->fb_bh ? state->fb_bh : 16; // Block height in GOBs

  // --- OVERRIDES & AUTO-DETECTS ---
  constexpr uint64_t kNyxFbBase = 0xF6200000ULL;

  if (g_swizzle_override != -1) {
    sw = (uint32_t)g_swizzle_override; // Manual User Override (S Key)
  } else if (eff_addr == kNyxFbBase) {
    // Nyx GUI uses VIC to rotate its 1280x720 landscape UI into 720x1280 portrait mode (NYX_FB_ADDRESS).
    // Our emulator's mock VIC only memcpys the data, so 0xF6200000 actually holds a 1280x720 landscape image!
    // Since the DC is programmed for 720x1280, it will scramble it.
    // We override the display geometry here to render the raw 1280x720 memory correctly.
    sw = 0;
    width = 1280;
    height = 720;
    eff_stride = 1280 * 4; // 32-bit XRGB
    rot = 0;
    bh = 4;
  } else if (eff_addr == FB_BASE && sw == 0 && width == 720u &&
             height == 1280u && eff_stride == width * 4u) {
    // Hekate TUI or static bootlogo: natively rendered as 720x1280 pitch-linear portrait.
    // Do not apply any automatic layout overrides.
    sw = 0;
    rot = 0; // The UI is drawn sideways in memory to appear upright on a landscape physical Switch screen.
             // On PC, we want to view it rotated so it resembles the physical landscape Switch!
  } else if (addr_in_ipl_fb(eff_addr) && sw == 0) {
    // IPL carveout with Sw=0: Hekate writes block-linear data but the DC
    // surface kind says pitch. Detect when the stride/dimensions don't
    // match a coherent pitch-linear layout and force block-linear decode.
    bool coherent_pitch =
        width > 0 && (eff_stride == width * 4u || eff_stride == width * 2u);
    bool portrait_stride_mismatch =
        height == 1280u && width == 720u && eff_stride != width * 4u;
    bool known_menu_stride = (eff_stride == 2624u && !coherent_pitch);

    // crop_portrait only forces BL when stride is incoherent — if stride
    // matches width*bpp, the data may already be VIC-de-swizzled pitch.
    bool crop_portrait_incoherent =
        height == 1280u && width < 720u && width >= 640u && !coherent_pitch;

    if (!coherent_pitch || portrait_stride_mismatch || known_menu_stride ||
        crop_portrait_incoherent) {
      sw = 2; // Force block-linear decode
      bh = 4; // Hekate menu uses BH=4
    }
  }

  // Final heuristic: NEVER auto-rotate 720x1280 portrait buffers to landscape!
  if (rot == -1) {
    if (width == 720 && height == 1280) {
      rot = 0; // Hekate already drew it sideways for physical portrait, so rot=0 makes it look like a phone on PC monitor!
               // Actually wait: if drawn upright in memory (row-by-row), then rot=0 produces a tall 720x1280 window on PC where it is perfectly upright!
               // The user wants the white lines to be horizontal. This means we MUST let the window be 720x1280!
    } else {
      rot = (height > width) ? 1 : 0; // Legacy heuristic for other stuff
    }
  }

  // Manual rotation override always wins. The auto-detect branches above set
  // rot = 0 for Nyx and Hekate TUI surfaces; if the user pressed R/Shift+R
  // they expect the rotation to actually change, regardless of what the
  // auto-detect picked. -1 means "use auto-detect", anything else overrides.
  state->last_auto_rot.store((uint32_t)(rot & 3));
  if (state->rotation_override != -1)
    rot = state->rotation_override & 3;

  // --- Compute GOB stride for block-linear ---
  uint32_t sw_gobs = (width * bpp + 63) / 64;
  if (sw == 2) {
    // For IPL carveout portrait surfaces, Hekate allocates a full 720px-wide
    // BL surface even when the DC window is cropped to 656px.
    if (addr_in_ipl_fb(state->fb_addr) && state->fb_addr != kNyxFbBase &&
        bpp == 4 && height == 1280u && width >= 640u && width <= 720u) {
      uint32_t sw_720 = (720u * bpp + 63) / 64u;
      if (sw_gobs < sw_720)
        sw_gobs = sw_720;
    }
    // If DC provides a stride that's GOB-aligned, prefer it
    if (eff_stride >= 64u && (eff_stride % 64u) == 0u) {
      uint32_t stride_gobs = eff_stride / 64u;
      if (stride_gobs > sw_gobs)
        sw_gobs = stride_gobs;
    }
  }

  uint32_t out_w = (rot & 1) ? height : width;
  uint32_t out_h = (rot & 1) ? width : height;

  // Cache the effective rotation and output dims so the SDL event handler can
  // invert the framebuffer→window transform when mapping mouse → panel coords.
  state->last_rot.store((uint32_t)((rot < 0) ? 0 : (rot & 3)));
  state->last_out_w.store(out_w);
  state->last_out_h.store(out_h);

  static uint32_t last_w = 0, last_h = 0;
  if (!texture || out_w != last_w || out_h != last_h) {
    if (texture)
      SDL_DestroyTexture(texture);
    // Use ARGB8888: natively matches Tegra's BGRA memory layout on Little
    // Endian
    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
                                SDL_TEXTUREACCESS_STREAMING, out_w, out_h);
    last_w = out_w;
    last_h = out_h;
    SDL_SetWindowSize(window, out_w, out_h);
  }

  static uint32_t last_log_p[6] = {0}; // Addr, W, H, Rot, Sw, BH
  if (state->fb_addr != last_log_p[0] || width != last_log_p[1] ||
      height != last_log_p[2] || (uint32_t)rot != last_log_p[3] ||
      sw != last_log_p[4] || bh != last_log_p[5]) {
    printf("[display] LATCH: 0x%X (%dx%d), Rot: %d (Manual: %d), Sw: %d, "
           "GOBs: %u, BH: %u, BPP: %u\n",
           (uint32_t)state->fb_addr, width, height, rot,
           state->rotation_override, sw, sw_gobs, bh, bpp * 8);
    last_log_p[0] = state->fb_addr;
    last_log_p[1] = width;
    last_log_p[2] = height;
    last_log_p[3] = rot;
    last_log_p[4] = sw;
    last_log_p[5] = bh;
  }

  eff_addr =
      (eff_addr ? eff_addr : FB_BASE) +
      (uint64_t)(int64_t)state->manual_offset;

  // Compute framebuffer byte size
  size_t fb_size = (sw == 2) ? ((size_t)sw_gobs * ((height + 7) / 8) * 512)
                             : ((size_t)eff_stride * height);
  if (fb_size < 1024)
    return;
  if (sw != 2 && eff_stride < 16)
    return; // Pitch-linear sanity check only
  if (fb_size > 32 * 1024 * 1024)
    fb_size = 32 * 1024 * 1024;

  // The surface is read straight out of the host memory backing emulated
  // DRAM when it lives there (it always does for hekate/Nyx), instead of a
  // uc_mem_read into a fresh 3.5 MB vector - and the converted frame reuses
  // one buffer too. The old code allocated and zero-filled two such vectors
  // on every frame, 60 times a second, which showed up in profiles as page-
  // fault and munmap churn on the emulation thread.
  static std::vector<uint8_t> fallback;
  const uint8_t *src = nullptr;
  if (state->dram_low_ptr && eff_addr >= DRAM_BASE &&
      eff_addr + fb_size <= DRAM_BASE + DRAM_WINDOW_SIZE) {
    src = state->dram_low_ptr + (eff_addr - DRAM_BASE);
  } else {
    fallback.resize(fb_size);
    if (uc_mem_read(uc, eff_addr, fallback.data(), fb_size) == UC_ERR_OK)
      src = fallback.data();
  }

  // Skip the de-swizzle and the texture upload when neither the surface
  // bytes nor the way they are laid out has changed since the last frame -
  // the common case for a payload sitting on a menu or waiting on hardware.
  struct Layout {
    uint64_t addr = 0;
    uint32_t w = 0, h = 0, stride = 0, sw = 0, bh = 0, gobs = 0, out_w = 0,
             out_h = 0;
    int rot = 0;
    bool operator==(const Layout &o) const {
      return addr == o.addr && w == o.w && h == o.h && stride == o.stride &&
             sw == o.sw && bh == o.bh && gobs == o.gobs && out_w == o.out_w &&
             out_h == o.out_h && rot == o.rot;
    }
  };
  static Layout last_layout;
  static std::vector<uint8_t> last_src;
  Layout layout;
  layout.addr = eff_addr;
  layout.w = width;
  layout.h = height;
  layout.stride = eff_stride;
  layout.sw = sw;
  layout.bh = bh;
  layout.gobs = sw_gobs;
  layout.out_w = out_w;
  layout.out_h = out_h;
  layout.rot = rot;
  bool unchanged = src && layout == last_layout && last_src.size() == fb_size &&
                   memcmp(last_src.data(), src, fb_size) == 0;
  static bool snapshot_pending = true;

  if (src && !unchanged) {
    last_layout = layout;
    last_src.assign(src, src + fb_size);
    snapshot_pending = true;
    const uint8_t *buf_data = src;
    std::vector<uint32_t> &proc = g_frame;
    proc.assign((size_t)out_w * out_h, 0xFF000000); // opaque black

    for (uint32_t sy = 0; sy < height; sy++) {
      for (uint32_t sx = 0; sx < width; sx++) {
        uint32_t off = 0;
        if (sw == 2) {
          if (bpp == 4) {
            // Use tegra_bl_byte_off_rgba8888 formula inline for 32bpp
            uint32_t gob_x = sx / 16u;
            uint32_t pixel_x = sx % 16u;
            uint32_t gob_y = sy / 8u;
            uint32_t line_y = sy % 8u;
            uint32_t block_y = gob_y / bh;
            uint32_t gob_in_block_y = gob_y % bh;

            uint32_t idx =
                (block_y * sw_gobs * bh * 128u) + (gob_x * bh * 128u) +
                (gob_in_block_y * 128u) + (pixel_x & 3u) +
                ((line_y & 1u) << 2) + ((pixel_x & 4u) << 1) +
                ((line_y & 2u) << 3) + ((pixel_x & 8u) << 2) +
                ((line_y & 4u) << 4);
            off = idx; // Already in uint32 units
          } else {
            // 16bpp block-linear (Nyx RGB565)
            uint32_t byte_x = sx * bpp;
            uint32_t gx = byte_x / 64;
            uint32_t px_in = byte_x % 64;
            uint32_t gy = sy / 8;
            uint32_t ly = sy % 8;
            uint32_t gob_off = ((ly >> 1) << 7) + ((px_in >> 4) << 5) +
                               ((ly & 1) << 4) + (px_in & 0xF);
            uint32_t gob_idx =
                (gy / bh) * (sw_gobs * bh) + gx * bh + (gy % bh);
            uint32_t byte_off = gob_idx * 512 + gob_off;
            // Read 16-bit pixel and expand to 32-bit ARGB
            if (byte_off + 2 <= fb_size) {
              uint16_t u16 = *(const uint16_t *)&buf_data[byte_off];
              uint32_t p = (((u16 >> 11) & 0x1F) << 19) |
                           (((u16 >> 5) & 0x3F) << 10) |
                           ((u16 & 0x1F) << 3) | 0xFF000000;
              uint32_t dx = sx, dy = sy;
              switch (rot) {
              case 1: dx = height - 1 - sy; dy = sx; break;
              case 2: dx = width - 1 - sx; dy = height - 1 - sy; break;
              case 3: dx = sy; dy = width - 1 - sx; break;
              default: break;
              }
              if (dx < out_w && dy < out_h)
                proc[dy * out_w + dx] = p;
            }
            continue; // Skip the common 32bpp path below
          }
        } else {
          off = (sy * eff_stride + sx * bpp) / 4;
        }

        if (off < fb_size / 4) {
          uint32_t p = ((const uint32_t *)buf_data)[off];
          uint32_t dx = sx, dy = sy;

          switch (rot) {
          case 1:
            dx = height - 1 - sy;
            dy = sx;
            break;
          case 2:
            dx = width - 1 - sx;
            dy = height - 1 - sy;
            break;
          case 3:
            dx = sy;
            dy = width - 1 - sx;
            break;
          default:
            break; // 0
          }
          if (dx < out_w && dy < out_h) {
            // Native BGRA to ARGB8888 conversion mapping (forces A to 0xFF)
            proc[dy * out_w + dx] = p | 0xFF000000;
          }
        }
      }
    }
    SDL_UpdateTexture(texture, nullptr, proc.data(), out_w * 4);
  }

  if (src) {
    // Snapshot for PNG conversion — at most once a second (~60 frames), and
    // only when the frame has changed since the last one written, so an idle
    // payload no longer rewrites a 3.5 MB file every second.
    static int snap_counter = 0;
    const std::vector<uint32_t> &proc = g_frame;
    if (++snap_counter >= 60 && snapshot_pending && !proc.empty()) {
      snap_counter = 0;
      snapshot_pending = false;
      FILE *f = fopen("last_fb.rgba", "wb");
      if (f) {
        fwrite(proc.data(), 1, proc.size() * 4, f);
        fclose(f);
        FILE *fm = fopen("last_fb.meta", "w");
        if (fm) {
          fprintf(fm, "%u %u\n", out_w, out_h);
          fclose(fm);
        }
      }
    }
  }

  SDL_RenderClear(renderer);
  SDL_RenderCopy(renderer, texture, nullptr, nullptr);
  SDL_RenderPresent(renderer);
  state->display_initialized = true;
}

// Map SDL window mouse coords to FTS4 panel-raw coords by inverting the
// framebuffer→window rotation pipeline. Returns false if the rendered output
// has not been measured yet (no display update happened).
static bool window_to_panel(EmuState *state, int mx, int my,
                            uint16_t *panel_x, uint16_t *panel_y) {
  uint32_t out_w = state->last_out_w.load();
  uint32_t out_h = state->last_out_h.load();
  if (out_w == 0 || out_h == 0) return false;

  // Scale window pixel coords to texture pixel coords (window may be resized).
  int win_w = 0, win_h = 0;
  SDL_GetWindowSize(window, &win_w, &win_h);
  if (win_w <= 0 || win_h <= 0) return false;
  double tx_d = (double)mx * (double)out_w / (double)win_w;
  double ty_d = (double)my * (double)out_h / (double)win_h;
  if (tx_d < 0) tx_d = 0;
  if (ty_d < 0) ty_d = 0;
  uint32_t tx = (uint32_t)tx_d;
  uint32_t ty = (uint32_t)ty_d;
  if (tx >= out_w) tx = out_w - 1;
  if (ty >= out_h) ty = out_h - 1;

  // With the auto-detected rotation, the rendered SDL view represents the
  // physical Switch screen in its intended orientation (post-rotation, post-
  // de-swizzle): what the user sees is what they would see looking at a
  // Switch held landscape. A manual override (R / Shift+R) turns that view
  // by a further quarter-turn or two, so undo exactly that extra turn first:
  // the render maps a source pixel (sx, sy) of a W x H image to
  //   1: (H-1-sy, sx)   2: (W-1-sx, H-1-sy)   3: (sy, W-1-sx)
  uint32_t delta = (state->last_rot.load() - state->last_auto_rot.load()) & 3;
  uint32_t aw = (delta & 1) ? out_h : out_w;   // auto-rotated view's size
  uint32_t ah = (delta & 1) ? out_w : out_h;
  uint32_t ax = tx, ay = ty;
  switch (delta) {
  case 1: ax = ty;          ay = ah - 1 - tx; break;
  case 2: ax = aw - 1 - tx; ay = ah - 1 - ty; break;
  case 3: ax = aw - 1 - ty; ay = tx;          break;
  }

  // The FTS4 panel sits behind that physical screen with its long axis
  // (panel_x) along the long dimension and its short axis (panel_y) along
  // the short, so whichever of (ax, ay) lives on the longer axis is the
  // long-axis sample.
  bool landscape = (aw >= ah);
  uint32_t long_pix   = landscape ? ax : ay;
  uint32_t long_max   = landscape ? aw : ah;
  uint32_t short_pix  = landscape ? ay : ax;
  uint32_t short_max  = landscape ? ah : aw;
  if (long_max  == 0) long_max  = 1;
  if (short_max == 0) short_max = 1;

  // bdk's touch.c clamps raw samples to EDGE_OFFSET..REAL_MAX (15..1264 and
  // 15..704) and stretches that span over the 1280x720 screen, so the inverse
  // starts at the edge offset, not at 0.
  constexpr uint32_t kEdge = 15, kXMax = 1264, kYMax = 704;
  uint32_t px = kEdge + (long_pix  * (kXMax - kEdge)) / long_max;
  uint32_t py = kEdge + (short_pix * (kYMax - kEdge)) / short_max;
  if (px > kXMax) px = kXMax;
  if (py > kYMax) py = kYMax;
  *panel_x = (uint16_t)px;
  *panel_y = (uint16_t)py;
  return true;
}

bool sdl_display_poll_events(EmuState *state, uc_engine *uc) {
  (void)uc; // Available for future use (e.g. raw FB dump on F8)
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_QUIT) {
      state->running = false;
      return false;
    }
    // SDL_QUIT only fires when the *last* window closes; in multi-window
    // setups the per-window close button just emits SDL_WINDOWEVENT_CLOSE.
    // Treat closing the main window as "quit", and let the config window's
    // close be handled by its own handler below (which just hides it).
    if (event.type == SDL_WINDOWEVENT &&
        event.window.event == SDL_WINDOWEVENT_CLOSE &&
        event.window.windowID != config_window_id() &&
        event.window.windowID != console_window_id()) {
      state->running = false;
      return false;
    }
    // Route events targeting an auxiliary ImGui window to its own handler
    // and skip the main-window handlers below (so typing into a hex input
    // doesn't also press POWER, etc.).
    if (event_targets_window(event, config_window_id())) {
      config_window_handle_event(event);
      continue;
    }
    if (event_targets_window(event, console_window_id())) {
      console_window_handle_event(event);
      continue;
    }
    // ---- Mouse → FTS4 touchscreen events ----
    if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
      uint16_t px = 0, py = 0;
      if (window_to_panel(state, event.button.x, event.button.y, &px, &py)) {
        state->tc_pressed.store(true);
        state->touch_post(0x03, px, py); // FTS4_EV_MULTI_TOUCH_ENTER
        printf("[touch] DOWN win=(%d,%d) panel=(%u,%u)\n",
               event.button.x, event.button.y, px, py);
      }
    }
    if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT) {
      uint16_t px = 0, py = 0;
      if (window_to_panel(state, event.button.x, event.button.y, &px, &py)) {
        state->tc_pressed.store(false);
        state->touch_post(0x04, px, py); // FTS4_EV_MULTI_TOUCH_LEAVE
        printf("[touch] UP   win=(%d,%d) panel=(%u,%u)\n",
               event.button.x, event.button.y, px, py);
      }
    }
    if (event.type == SDL_MOUSEMOTION && state->tc_pressed.load()) {
      uint16_t px = 0, py = 0;
      if (window_to_panel(state, event.motion.x, event.motion.y, &px, &py)) {
        // A release outside the window never reaches it when mouse capture
        // is off (the ImGui backend turns SDL's auto-capture off process-
        // wide), so a motion with the button up ends the touch instead.
        if (!(event.motion.state & SDL_BUTTON_LMASK)) {
          state->tc_pressed.store(false);
          state->touch_post(0x04, px, py); // FTS4_EV_MULTI_TOUCH_LEAVE
        } else {
          state->touch_post(0x05, px, py); // FTS4_EV_MULTI_TOUCH_MOTION
        }
      }
    }
    if (event.type == SDL_KEYDOWN) {
      bool shift = (SDL_GetModState() & KMOD_SHIFT);
      bool ctrl  = (SDL_GetModState() & KMOD_CTRL);
      uint32_t step = shift ? 4096 : 128;
      switch (event.key.keysym.sym) {
      case SDLK_UP:
        if (!state->btn_vol_up)
          printf("[input] DOWN VOL_UP    @emu_usec=%llu\n", (unsigned long long)state->emu_usec);
        state->btn_vol_up = true;
        break;
      case SDLK_DOWN:
        if (!state->btn_vol_down)
          printf("[input] DOWN VOL_DOWN  @emu_usec=%llu\n", (unsigned long long)state->emu_usec);
        state->btn_vol_down = true;
        break;
      case SDLK_RETURN:
        if (!state->btn_power)
          printf("[input] DOWN POWER     @emu_usec=%llu\n", (unsigned long long)state->emu_usec);
        state->btn_power = true;
        break;
      case SDLK_p:
        state->paused = !state->paused;
        printf("[diag] Emulation %s\n", state->paused ? "PAUSED" : "RESUMED");
        break;
      case SDLK_r: {
        if (!ctrl) {
          // Plain R: soft reboot. Same path as the config window's "Reboot"
          // button — re-prime IRAM payload + WDT cookie, wipe DRAM, reset PC.
          state->reboot_cold = true;  // the user's reset is a power cycle
          state->reboot_requested = true;
          printf("[diag] Reboot requested (R)\n");
        } else {
          // Ctrl+R / Ctrl+Shift+R: cycle display rotation
          //   Ctrl+R       : 90° clockwise   (Auto → 0 → 90 → 180 → 270 → Auto)
          //   Ctrl+Shift+R : 90° counter-CW  (Auto → 270 → 180 → 90 → 0 → Auto)
          int r = state->rotation_override;
          if (shift) r = (r == -1) ? 3 : (r ==  0) ? -1 : r - 1;
          else       r = (r ==  3) ? -1 : r + 1;
          state->rotation_override = r;
          const char *label =
              r == -1 ? "Auto" : r == 0 ? "0\xC2\xB0" :
              r ==  1 ? "90\xC2\xB0 CW" :
              r ==  2 ? "180\xC2\xB0" :
                        "270\xC2\xB0 CW";
          printf("[diag] Rotation Override: %d (%s)\n", r, label);
        }
        break;
      }
      case SDLK_i:
        state->manual_offset -= step;
        printf("[diag] Offset: %d\n", state->manual_offset);
        break;
      case SDLK_o:
        state->manual_offset += step;
        printf("[diag] Offset: %d\n", state->manual_offset);
        break;
      case SDLK_s:
        g_swizzle_override =
            (g_swizzle_override == -1) ? 0 : (g_swizzle_override == 0 ? 2 : -1);
        printf("[diag] Swizzle Override: %d (-1=Auto, 0=Pitch, 2=Block)\n",
               g_swizzle_override);
        break;
      case SDLK_c:
        console_window_toggle();
        printf("[ui] UART console %s\n",
               console_window_is_visible() ? "OPEN" : "CLOSED");
        fflush(stdout);
        break;
      case SDLK_m:
        config_window_toggle();
        printf("[diag] Config window %s\n",
               config_window_is_visible() ? "OPEN" : "CLOSED");
        break;
      case SDLK_ESCAPE:
        state->running = false;
        return false;
      }
    }
    if (event.type == SDL_KEYUP) {
      switch (event.key.keysym.sym) {
      case SDLK_UP:
        if (state->btn_vol_up)
          printf("[input] UP   VOL_UP    @emu_usec=%llu\n", (unsigned long long)state->emu_usec);
        state->btn_vol_up = false;
        break;
      case SDLK_DOWN:
        if (state->btn_vol_down)
          printf("[input] UP   VOL_DOWN  @emu_usec=%llu\n", (unsigned long long)state->emu_usec);
        state->btn_vol_down = false;
        break;
      case SDLK_RETURN:
        if (state->btn_power)
          printf("[input] UP   POWER     @emu_usec=%llu\n", (unsigned long long)state->emu_usec);
        state->btn_power = false;
        break;
      }
    }
  }
  return true;
}

void sdl_display_shutdown() {
  SDL_Quit();
  printf("[display] SDL2 shutdown\n");
}