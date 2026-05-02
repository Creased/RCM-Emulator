#include "sdl_display.h"
#include "config_window.h"
#include "../emu_state.h"
#include "../t210/memory_map.h"

#include <SDL2/SDL.h>
#include <cstdio>
#include <cstring>
#include <vector>

// Some SDL events are not window-targeted (e.g. SDL_QUIT). Returns true only
// for events that carry a windowID matching the config window.
static bool event_targets_config_window(const SDL_Event &ev) {
    Uint32 cfg_id = config_window_id();
    if (cfg_id == 0) return false;
    switch (ev.type) {
    case SDL_WINDOWEVENT:      return ev.window.windowID    == cfg_id;
    case SDL_KEYDOWN:
    case SDL_KEYUP:            return ev.key.windowID       == cfg_id;
    case SDL_TEXTINPUT:        return ev.text.windowID      == cfg_id;
    case SDL_MOUSEBUTTONDOWN:
    case SDL_MOUSEBUTTONUP:    return ev.button.windowID    == cfg_id;
    case SDL_MOUSEMOTION:      return ev.motion.windowID    == cfg_id;
    case SDL_MOUSEWHEEL:       return ev.wheel.windowID     == cfg_id;
    default:                   return false;
    }
}

static SDL_Window *window = nullptr;
static SDL_Renderer *renderer = nullptr;
static SDL_Texture *texture = nullptr;
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
  renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
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

  int rot = state->rotation_override;

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

  std::vector<uint8_t> buf(fb_size);
  if (uc_mem_read(uc, eff_addr, buf.data(), fb_size) == UC_ERR_OK) {
    std::vector<uint32_t> proc(out_w * out_h,
                               0xFF000000); // Initialize opaque black

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
              uint16_t u16 = *(uint16_t *)&buf[byte_off];
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
          uint32_t p = ((uint32_t *)buf.data())[off];
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

    // Snapshot for PNG conversion — save periodically (~1s) so the last
    // written file always reflects the most recent frame content.
    static int snap_counter = 0;
    if (++snap_counter >= 60) { // ~1 second at 60 FPS
      snap_counter = 0;
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

  // The rendered SDL view always represents the physical Switch screen in its
  // intended orientation (post-rotation, post-de-swizzle). Whatever the
  // framebuffer's in-memory layout, what the user sees is what they would see
  // looking at a Switch held landscape (long axis horizontal). The FTS4 panel
  // sits behind that physical screen with its long axis (panel_x) along the
  // long dimension and its short axis (panel_y) along the short.
  //
  // So we map the texture coord to panel coords by aspect: whichever of (tx,
  // ty) lives on the longer rendered axis is the long-axis sample.
  bool landscape = (out_w >= out_h);
  uint32_t long_pix   = landscape ? tx     : ty;
  uint32_t long_max   = landscape ? out_w  : out_h;
  uint32_t short_pix  = landscape ? ty     : tx;
  uint32_t short_max  = landscape ? out_h  : out_w;
  if (long_max  == 0) long_max  = 1;
  if (short_max == 0) short_max = 1;

  uint32_t px = (long_pix  * 1264u) / long_max;   // FTS4 X_REAL_MAX = 1264
  uint32_t py = (short_pix * 704u)  / short_max;  // FTS4 Y_REAL_MAX = 704
  if (px > 1264) px = 1264;
  if (py > 704)  py = 704;
  *panel_x = (uint16_t)px;
  *panel_y = (uint16_t)py;
  (void)state; // last_rot still cached for future use; current mapping is rotation-agnostic
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
        event.window.windowID != config_window_id()) {
      state->running = false;
      return false;
    }
    // Route events targeting the hardware config window to ImGui and skip
    // the main-window handlers below (so e.g. typing into a hex input doesn't
    // also press POWER).
    if (event_targets_config_window(event)) {
      config_window_handle_event(event);
      continue;
    }
    // ---- Mouse → FTS4 touchscreen events ----
    if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
      uint16_t px = 0, py = 0;
      if (window_to_panel(state, event.button.x, event.button.y, &px, &py)) {
        state->tc_x.store(px);
        state->tc_y.store(py);
        state->tc_pressed.store(true);
        state->tc_event_op.store(0x03); // FTS4_EV_MULTI_TOUCH_ENTER
        state->tc_event_pending.store(true);
        printf("[touch] DOWN win=(%d,%d) panel=(%u,%u)\n",
               event.button.x, event.button.y, px, py);
      }
    }
    if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT) {
      uint16_t px = 0, py = 0;
      if (window_to_panel(state, event.button.x, event.button.y, &px, &py)) {
        state->tc_x.store(px);
        state->tc_y.store(py);
        state->tc_pressed.store(false);
        state->tc_event_op.store(0x04); // FTS4_EV_MULTI_TOUCH_LEAVE
        state->tc_event_pending.store(true);
        printf("[touch] UP   win=(%d,%d) panel=(%u,%u)\n",
               event.button.x, event.button.y, px, py);
      }
    }
    if (event.type == SDL_MOUSEMOTION && state->tc_pressed.load()) {
      uint16_t px = 0, py = 0;
      if (window_to_panel(state, event.motion.x, event.motion.y, &px, &py)) {
        state->tc_x.store(px);
        state->tc_y.store(py);
        state->tc_event_op.store(0x05); // FTS4_EV_MULTI_TOUCH_MOTION
        state->tc_event_pending.store(true);
      }
    }
    if (event.type == SDL_KEYDOWN) {
      bool shift = (SDL_GetModState() & KMOD_SHIFT);
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
        // R         : step rotation 90° clockwise   (Auto → 0 → 90 → 180 → 270 → Auto)
        // Shift+R   : step rotation 90° counter-CW  (Auto → 270 → 180 → 90 → 0 → Auto)
        int r = state->rotation_override;
        if (shift) {
          // CCW: -1 → 3 → 2 → 1 → 0 → -1
          r = (r == -1) ? 3 : (r == 0) ? -1 : r - 1;
        } else {
          // CW:  -1 → 0 → 1 → 2 → 3 → -1
          r = (r == 3) ? -1 : r + 1;
        }
        state->rotation_override = r;
        const char *label =
            r == -1 ? "Auto" : r == 0 ? "0\xC2\xB0" :
            r == 1 ? "90\xC2\xB0 CW" :
            r == 2 ? "180\xC2\xB0" :
                     "270\xC2\xB0 CW";
        printf("[diag] Rotation Override: %d (%s)\n", r, label);
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