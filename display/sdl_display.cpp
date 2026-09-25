#include "sdl_display.h"
#include "config_window.h"
#include "console_window.h"
#include "../emu_state.h"
#include "../t210/memory_map.h"

#include <SDL2/SDL.h>
#include <cstdio>
#include <algorithm>
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

// ---- Scan-out ------------------------------------------------------------
//
// The frame is built the way the display controller builds it (TRM 24):
// every enabled window's surface is fetched per its SURFACE_KIND (pitch,
// 16x16 tiled or block linear) and LINE_STRIDE, turned per WIN_OPTIONS
// (H/V direction, SCAN_COLUMN), scaled from PRESCALED_SIZE to SIZE and
// placed at POSITION, the windows stacked by layer depth and blended per
// BLEND_*. The result is the 720x1280 portrait picture the panel receives.
//
// For the host window that picture is then turned: when the surface on show
// was itself turned on its way to the panel - by the DC (SCAN_COLUMN) or by
// a VIC compose, as Nyx does - the turn is undone, so a GUI drawn landscape
// is shown landscape. A surface drawn straight into portrait (hekate's TUI,
// hwtest) is shown as the panel gets it. Ctrl+R / Ctrl+Shift+R override the
// turn.

namespace {

constexpr uint32_t kPanelW = 720, kPanelH = 1280;

// A picture turn as a 2x2 matrix on centred coordinates: out = M * in.
struct Turn {
  int a, b, c, d; // [[a b] [c d]]
};

Turn turn_of(uint32_t xf) { // dcwin::XF_* (flip, then transpose)
  int fx = (xf & dcwin::XF_FLIP_X) ? -1 : 1;
  int fy = (xf & dcwin::XF_FLIP_Y) ? -1 : 1;
  if (xf & dcwin::XF_TRANSPOSE)
    return {0, fy, fx, 0};
  return {fx, 0, 0, fy};
}

Turn operator*(const Turn &l, const Turn &r) {
  return {l.a * r.a + l.b * r.c, l.a * r.b + l.b * r.d,
          l.c * r.a + l.d * r.c, l.c * r.b + l.d * r.d};
}

// The view rotation (as the render loop applies it, see rot below) that
// undoes turn t, or 0 when t is a mirror image, which no rotation undoes.
int undo_rot(const Turn &t) {
  if (t.a * t.d - t.b * t.c != 1)
    return 0;
  Turn inv = {t.a, t.c, t.b, t.d}; // a rotation's inverse is its transpose
  if (inv.a == 0 && inv.b == -1)
    return 1;
  if (inv.a == -1)
    return 2;
  if (inv.a == 0 && inv.b == 1)
    return 3;
  return 0;
}

uint32_t depth_bpp(uint32_t depth) {
  if (depth <= 3)
    return 1; // palettised
  if (depth <= 7 || (depth >= 0x1E && depth <= 0x21))
    return 2;
  return 4;
}

// One pixel of a DC_WIN_COLOR_DEPTH format as ARGB8888.
uint32_t depth_argb(const uint8_t *p, uint32_t depth) {
  auto x5 = [](uint32_t v) { return (v << 3) | (v >> 2); };
  auto x6 = [](uint32_t v) { return (v << 2) | (v >> 4); };
  switch (depth) {
  case 0x4: { // B4G4R4A4
    uint32_t v = p[0] | (p[1] << 8);
    return ((v >> 12) * 0x11u) << 24 | ((v >> 8) & 0xF) * 0x11u << 16 |
           ((v >> 4) & 0xF) * 0x11u << 8 | (v & 0xF) * 0x11u;
  }
  case 0x5: { // B5G5R5A
    uint32_t v = p[0] | (p[1] << 8);
    return ((v & 0x8000) ? 0xFF000000u : 0) | x5((v >> 10) & 0x1F) << 16 |
           x5((v >> 5) & 0x1F) << 8 | x5(v & 0x1F);
  }
  case 0x7: { // AB5G5R5
    uint32_t v = p[0] | (p[1] << 8);
    return ((v & 1) ? 0xFF000000u : 0) | x5(v >> 11) << 16 |
           x5((v >> 6) & 0x1F) << 8 | x5((v >> 1) & 0x1F);
  }
  case 0xD: // R8G8B8A8
  case 0xF:
    return (uint32_t)p[3] << 24 | (uint32_t)p[0] << 16 |
           (uint32_t)p[1] << 8 | p[2];
  default:
    break;
  }
  switch (depth_bpp(depth)) {
  case 1:
    return 0xFF000000u | p[0] * 0x010101u;
  case 2: { // B5G6R5 and anything else 16-bit
    uint32_t v = p[0] | (p[1] << 8);
    return 0xFF000000u | x5(v >> 11) << 16 | x6((v >> 5) & 0x3F) << 8 |
           x5(v & 0x1F);
  }
  default: // B8G8R8A8 (bytes B, G, R, A)
    return (uint32_t)p[3] << 24 | (uint32_t)p[2] << 16 |
           (uint32_t)p[1] << 8 | p[0];
  }
}

struct Win {
  int idx = 0;
  uint64_t addr = 0;
  uint32_t depth = 0, bpp = 4, kind = 0, gob_h = 1, stride = 0, gobs = 0;
  uint32_t src_w = 0, src_h = 0; // the surface, before the DC turns it
  uint32_t pre_w = 0, pre_h = 0; // PRESCALED_SIZE, in pixels (post-turn)
  uint32_t out_w = 0, out_h = 0; // SIZE
  int32_t x = 0, y = 0;
  uint32_t xf = 0, layer = 0, blend_layer = 0, blend_match = 0;
  size_t len = 0;
  const uint8_t *mem = nullptr;
  std::vector<uint8_t> copy; // when the surface is not in the DRAM window
};

// Byte offset of surface pixel (sx, sy).
inline size_t fetch_off(const Win &w, uint32_t sx, uint32_t sy) {
  uint32_t xb = sx * w.bpp;
  switch (w.kind) {
  case 1: // 16x16-byte tiles, row-major
    return (size_t)(sy / 16) * w.stride * 16 + (xb / 16) * 256 +
           (sy % 16) * 16 + (xb % 16);
  case 2: { // block linear, TRM 20.1.2 (Figure 47 sector order)
    uint32_t gob_y = sy / 8;
    size_t gob = ((size_t)(gob_y / w.gob_h) * w.gobs + xb / 64) * w.gob_h +
                 gob_y % w.gob_h;
    uint32_t x = xb % 64, y = sy % 8;
    return gob * 512 + (x / 32) * 256 + (y / 2) * 64 + ((x % 32) / 16) * 32 +
           (y % 2) * 16 + (x % 16);
  }
  default:
    return (size_t)sy * w.stride + xb;
  }
}

// Build window w from its active register file; false if it shows nothing.
bool load_window(EmuState *state, uc_engine *uc, int idx, const uint32_t *r,
                 int swizzle_override, Win &w) {
  if (!(r[dcwin::OPTIONS] & dcwin::WIN_ENABLE))
    return false;
  w.idx = idx;
  w.depth = r[dcwin::COLOR_DEPTH] & 0x7F;
  w.bpp = depth_bpp(w.depth);
  uint32_t sk = r[dcwin::SURFACE_KIND];
  w.kind = sk & 3;
  w.gob_h = 1u << ((sk >> 4) & 7);
  if (swizzle_override == 0) {
    w.kind = 0;
  } else if (swizzle_override == 2 && w.kind != 2) {
    w.kind = 2;
    w.gob_h = 16;
  }
  w.out_w = r[dcwin::SIZE] & 0x1FFF;
  w.out_h = (r[dcwin::SIZE] >> 16) & 0x1FFF;
  w.pre_w = (r[dcwin::PRESCALED] & 0x7FFF) / w.bpp;
  w.pre_h = (r[dcwin::PRESCALED] >> 16) & 0x1FFF;
  if (!w.pre_w || !w.pre_h) { // never programmed: no scaling
    w.pre_w = w.out_w;
    w.pre_h = w.out_h;
  }
  if (!w.out_w || !w.out_h)
    return false;
  uint32_t opt = r[dcwin::OPTIONS];
  w.xf = ((opt & dcwin::H_DIRECTION) ? dcwin::XF_FLIP_X : 0) |
         ((opt & dcwin::V_DIRECTION) ? dcwin::XF_FLIP_Y : 0) |
         ((opt & dcwin::SCAN_COLUMN) ? dcwin::XF_TRANSPOSE : 0);
  // SCAN_COLUMN walks the surface's columns: it is pre_h wide, pre_w tall.
  bool column = w.xf & dcwin::XF_TRANSPOSE;
  w.src_w = column ? w.pre_h : w.pre_w;
  w.src_h = column ? w.pre_w : w.pre_h;
  w.x = (int16_t)(r[dcwin::POSITION] & 0xFFFF);
  w.y = (int16_t)(r[dcwin::POSITION] >> 16);
  w.blend_layer = r[dcwin::BLEND_LAYER];
  w.blend_match = r[dcwin::BLEND_MATCH];
  w.layer = w.blend_layer & 0xFF;
  w.stride = r[dcwin::LINE_STRIDE] & 0xFFFF;
  if (!w.stride)
    w.stride = w.src_w * w.bpp;
  w.addr = (uint64_t)r[dcwin::START_ADDR] + (int64_t)state->manual_offset;
  switch (w.kind) {
  case 2:
    w.gobs = (w.stride + 63) / 64;
    w.len = (size_t)w.gobs * 512 * w.gob_h *
            ((w.src_h + 8 * w.gob_h - 1) / (8 * w.gob_h));
    break;
  case 1:
    w.len = (size_t)w.stride * ((w.src_h + 15) / 16) * 16;
    break;
  default:
    w.len = (size_t)w.stride * w.src_h;
    break;
  }
  if (w.len == 0 || w.len > 64u * 1024 * 1024)
    return false;
  if (state->dram_low_ptr && w.addr >= DRAM_BASE &&
      w.addr + w.len <= DRAM_BASE + DRAM_WINDOW_SIZE) {
    w.mem = state->dram_low_ptr + (w.addr - DRAM_BASE);
  } else {
    w.copy.resize(w.len);
    if (uc_mem_read(uc, w.addr, w.copy.data(), w.len) != UC_ERR_OK)
      return false;
    w.mem = w.copy.data();
  }
  return true;
}

// Blend factor (0..255) per BLEND_MATCH_SELECT (TRM 24.10.13).
uint32_t src_factor(uint32_t sel, uint32_t k1, uint32_t sa) {
  switch (sel) {
  case 1: return 255;
  case 2: case 3: return k1;            // K1, K1 x dst alpha (opaque dst)
  case 4: return 255 - k1;              // 1 - K1 x dst alpha
  case 5: return k1 * sa / 255;         // K1 x src alpha
  default: return 0;
  }
}
uint32_t dst_factor(uint32_t sel, uint32_t k1, uint32_t k2, uint32_t sa) {
  switch (sel) {
  case 1: return 255;
  case 2: case 4: return k1;
  case 3: return k2;
  case 5: case 7: return 255 - k1;
  case 6: return 255 - k1 * sa / 255;
  default: return 0;
  }
}

// Draw window w onto the panel picture.
void draw_window(const Win &w, std::vector<uint32_t> &panel) {
  bool blend = !(w.blend_layer & (1u << 24));
  uint32_t k1 = (w.blend_layer >> 8) & 0xFF, k2 = (w.blend_layer >> 16) & 0xFF;
  uint32_t ssel = w.blend_match & 7, dsel = (w.blend_match >> 4) & 7;
  bool hflip = w.xf & dcwin::XF_FLIP_X, vflip = w.xf & dcwin::XF_FLIP_Y;
  bool column = w.xf & dcwin::XF_TRANSPOSE;
  // Column scaling map, worked out once per window rather than per pixel.
  static std::vector<uint32_t> txs;
  txs.resize(w.out_w);
  for (uint32_t ox = 0; ox < w.out_w; ox++)
    txs[ox] = (uint32_t)((uint64_t)ox * w.pre_w / w.out_w);
  for (uint32_t oy = 0; oy < w.out_h; oy++) {
    int32_t py = w.y + (int32_t)oy;
    if (py < 0 || py >= (int32_t)kPanelH)
      continue;
    uint32_t ty = (uint32_t)((uint64_t)oy * w.pre_h / w.out_h);
    for (uint32_t ox = 0; ox < w.out_w; ox++) {
      int32_t px = w.x + (int32_t)ox;
      if (px < 0 || px >= (int32_t)kPanelW)
        continue;
      uint32_t tx = txs[ox];
      // (tx, ty) is post-turn; walk it back to the surface.
      uint32_t a = column ? ty : tx, b = column ? tx : ty;
      uint32_t sx = hflip ? w.src_w - 1 - a : a;
      uint32_t sy = vflip ? w.src_h - 1 - b : b;
      size_t off = fetch_off(w, sx, sy);
      if (off + w.bpp > w.len)
        continue;
      uint32_t s = depth_argb(w.mem + off, w.depth);
      uint32_t &d = panel[(size_t)py * kPanelW + px];
      if (!blend) {
        d = s | 0xFF000000u;
        continue;
      }
      uint32_t sa = s >> 24;
      uint32_t fs = src_factor(ssel, k1, sa), fd = dst_factor(dsel, k1, k2, sa);
      uint32_t o = 0xFF000000u;
      for (int sh = 0; sh < 24; sh += 8) {
        uint32_t c = (((s >> sh) & 0xFF) * fs + ((d >> sh) & 0xFF) * fd) / 255;
        o |= (c > 255 ? 255 : c) << sh;
      }
      d = o;
    }
  }
}

} // namespace

void sdl_display_update(EmuState *state, uc_engine *uc) {
  if (!renderer)
    return;

  // The windows on show, bottom first. Until the payload programs the DC,
  // the emulator's own framebuffer stands in as window A.
  std::vector<Win> wins;
  uint32_t boot_win[dcwin::kRegs] = {};
  boot_win[dcwin::OPTIONS] = dcwin::WIN_ENABLE;
  boot_win[dcwin::COLOR_DEPTH] = 0xC; // B8G8R8A8
  boot_win[dcwin::SIZE] = kPanelH << 16 | kPanelW;
  boot_win[dcwin::LINE_STRIDE] = kPanelW * 4;
  boot_win[dcwin::BLEND_LAYER] = dcwin::BLEND_LAYER_RESET;
  boot_win[dcwin::START_ADDR] = (uint32_t)FB_BASE;
  for (int i = 0; i < 4; i++) {
    const uint32_t *r = state->dc_programmed ? state->dc_win_active[i]
                                             : (i == 0 ? boot_win : nullptr);
    if (!r)
      continue;
    Win w;
    if (load_window(state, uc, i, r, g_swizzle_override, w))
      wins.push_back(std::move(w));
  }
  // Deeper layers first; equal depths stack A (bottom) to D (top).
  std::stable_sort(wins.begin(), wins.end(), [](const Win &l, const Win &r) {
    return l.layer > r.layer;
  });

  // The largest window decides how the picture is turned for the host.
  const Win *primary = nullptr;
  for (const Win &w : wins)
    if (!primary || (uint64_t)w.out_w * w.out_h >
                        (uint64_t)primary->out_w * primary->out_h ||
        ((uint64_t)w.out_w * w.out_h ==
             (uint64_t)primary->out_w * primary->out_h &&
         w.idx < primary->idx))
      primary = &w;
  int rot = 0;
  if (primary) {
    Turn t = turn_of(primary->xf);
    if (state->vic_out_addr && primary->addr == state->vic_out_addr)
      t = t * turn_of(state->vic_out_xform);
    rot = undo_rot(t);
  }
  state->last_auto_rot.store((uint32_t)rot);
  if (state->rotation_override != -1)
    rot = state->rotation_override & 3;

  uint32_t out_w = (rot & 1) ? kPanelH : kPanelW;
  uint32_t out_h = (rot & 1) ? kPanelW : kPanelH;
  // The SDL event handler inverts this view to map mouse -> panel coords.
  state->last_rot.store((uint32_t)rot);
  state->last_out_w.store(out_w);
  state->last_out_h.store(out_h);

  static uint32_t last_w = 0, last_h = 0;
  if (!texture || out_w != last_w || out_h != last_h) {
    if (texture)
      SDL_DestroyTexture(texture);
    // ARGB8888 matches the B8G8R8A8 bytes of a Tegra surface on a
    // little-endian host.
    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
                                SDL_TEXTUREACCESS_STREAMING, out_w, out_h);
    last_w = out_w;
    last_h = out_h;
    SDL_SetWindowSize(window, out_w, out_h);
  }

  // Skip the rebuild and the texture upload when neither the surfaces' bytes
  // nor the way they are laid out changed since the last frame - the common
  // case for a payload sitting on a menu or waiting on hardware.
  std::vector<uint64_t> shape = {(uint64_t)rot, wins.size()}, where;
  for (const Win &w : wins) {
    shape.insert(shape.end(),
                 {(uint64_t)w.idx, w.addr, w.len, w.depth, w.kind, w.gob_h,
                  w.stride, (uint64_t)w.pre_w << 32 | w.pre_h,
                  (uint64_t)w.out_w << 32 | w.out_h, w.xf, w.blend_layer,
                  w.blend_match});
    where.push_back((uint64_t)(uint32_t)w.x << 32 | (uint32_t)w.y);
  }
  static std::vector<uint64_t> last_shape, last_where;
  static std::vector<uint8_t> last_bytes;
  size_t total = 0;
  for (const Win &w : wins)
    total += w.len;
  bool same = shape == last_shape && where == last_where &&
              last_bytes.size() == total;
  for (size_t i = 0, pos = 0; same && i < wins.size(); pos += wins[i++].len)
    same = memcmp(last_bytes.data() + pos, wins[i].mem, wins[i].len) == 0;

  static bool snapshot_pending = true;
  if (!same) {
    // Log layout changes, but not moves: bdk slides its log window in.
    if (shape != last_shape)
      printf("[display] scan-out: %zu window(s), view %ux%u, rot %d (auto %u)\n",
             wins.size(), out_w, out_h, rot, state->last_auto_rot.load());
    last_shape = shape;
    last_where = where;
    last_bytes.resize(total);
    size_t pos = 0;
    for (const Win &w : wins) {
      memcpy(last_bytes.data() + pos, w.mem, w.len);
      pos += w.len;
    }
    snapshot_pending = true;

    static std::vector<uint32_t> panel;
    panel.assign((size_t)kPanelW * kPanelH, 0xFF000000u); // black background
    for (const Win &w : wins)
      draw_window(w, panel);

    // Turn the panel picture for the view: a panel pixel (sx, sy) of the
    // W x H picture lands at
    //   rot 1: (H-1-sy, sx)   2: (W-1-sx, H-1-sy)   3: (sy, W-1-sx)
    g_frame.resize((size_t)out_w * out_h);
    for (uint32_t sy = 0; sy < kPanelH; sy++) {
      const uint32_t *row = &panel[(size_t)sy * kPanelW];
      for (uint32_t sx = 0; sx < kPanelW; sx++) {
        uint32_t dx = sx, dy = sy;
        switch (rot) {
        case 1: dx = kPanelH - 1 - sy; dy = sx; break;
        case 2: dx = kPanelW - 1 - sx; dy = kPanelH - 1 - sy; break;
        case 3: dx = sy; dy = kPanelW - 1 - sx; break;
        default: break;
        }
        g_frame[(size_t)dy * out_w + dx] = row[sx];
      }
    }
    SDL_UpdateTexture(texture, nullptr, g_frame.data(), out_w * 4);
  }

  // Snapshot for PNG conversion - at most once a second (~60 frames), and
  // only when the frame has changed since the last one written, so an idle
  // payload does not rewrite a 3.5 MB file every second.
  static int snap_counter = 0;
  if (++snap_counter >= 60 && snapshot_pending && !g_frame.empty()) {
    snap_counter = 0;
    snapshot_pending = false;
    FILE *f = fopen("last_fb.rgba", "wb");
    if (f) {
      fwrite(g_frame.data(), 1, g_frame.size() * 4, f);
      fclose(f);
      FILE *fm = fopen("last_fb.meta", "w");
      if (fm) {
        fprintf(fm, "%u %u\n", out_w, out_h);
        fclose(fm);
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

  // With the auto-detected turn, the view shows the picture the way the
  // payload drew it (Nyx: landscape, as on a Switch held landscape). A
  // manual override (Ctrl+R / Ctrl+Shift+R) turns that view by a further
  // quarter-turn or two, so undo exactly that extra turn first: the render
  // maps a source pixel (sx, sy) of a W x H image to
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
          // button — reload the payload into IRAM, wipe DRAM, reset PC.
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