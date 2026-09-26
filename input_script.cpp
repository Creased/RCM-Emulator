// Scripted button input - see input_script.h.
//
// Syntax (identical for a file and for an inline spec; in a file one event per
// line, inline they're separated by ',' or ';'):
//
//     <when> <KEY> [hold_ms]
//
//   <when>    absolute time in ms ("3000"), or "+N" for N ms after the
//             previous event was released. Bare "+" means "+400".
//   <KEY>     POWER | PWR | P
//             VOL_DOWN | VOLDOWN | DOWN | D
//             VOL_UP   | VOLUP   | UP   | U
//   hold_ms   how long to hold the button, default 200 ms.
//
//     <when> TAP <x> <y> [hold_ms]
//
//   touches the screen at (x, y) in the 1280x720 landscape picture bdk's
//   touch driver reports (Nyx's coordinates), and lifts after hold_ms.
//
// '#' starts a comment. Example - walk 6 entries down a menu and select:
//
//     rcm_emu payload.bin --input-script "3000 D, + D, + D, + D, + D, + D, + P"
//
// or the same thing in a file:
//
//     # pick the 7th menu entry
//     3000 VOL_DOWN
//     +    VOL_DOWN
//     ...
//     +    POWER

#include "input_script.h"
#include "emu_state.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

enum class Key { Power, VolUp, VolDown, Uart, UartFile, Tap };

struct Event {
  uint64_t at_us = 0;   // press time
  uint64_t until_us = 0; // release time
  Key key = Key::Power;
  bool pressed = false;
  bool released = false;
  std::string data;      // UART / UARTFILE payload
  uint16_t x = 0, y = 0; // TAP: panel-raw coordinates
};

std::vector<Event> g_events;
bool g_active = false;
size_t g_done = 0;

constexpr uint64_t kDefaultHoldMs = 200;
constexpr uint64_t kDefaultGapMs = 400;

const char *key_name(Key k) {
  switch (k) {
  case Key::Power: return "POWER";
  case Key::VolUp: return "VOL+";
  case Key::Uart: return "UART";
  case Key::UartFile: return "UARTFILE";
  case Key::Tap: return "TAP";
  default: return "VOL-";
  }
}

bool parse_key(const std::string &tok, Key *out) {
  std::string s;
  for (char c : tok)
    s.push_back((char)toupper((unsigned char)c));
  if (s == "POWER" || s == "PWR" || s == "P") { *out = Key::Power; return true; }
  if (s == "VOL_DOWN" || s == "VOLDOWN" || s == "DOWN" || s == "D") {
    *out = Key::VolDown; return true;
  }
  if (s == "VOL_UP" || s == "VOLUP" || s == "UP" || s == "U") {
    *out = Key::VolUp; return true;
  }
  // Feed bytes into the UART_B receive FIFO rather than pressing a button.
  // UART pushes the rest of the line literally (with \n / \r escapes);
  // UARTFILE pushes a file's raw contents, for binary protocol work.
  if (s == "UART") { *out = Key::Uart; return true; }
  if (s == "UARTFILE") { *out = Key::UartFile; return true; }
  if (s == "TAP" || s == "T") { *out = Key::Tap; return true; }
  return false;
}

// Expand the small escape set we accept in a literal UART payload.
std::string unescape(const std::string &in) {
  std::string out;
  for (size_t i = 0; i < in.size(); i++) {
    if (in[i] == '\\' && i + 1 < in.size()) {
      char c = in[++i];
      if (c == 'n') { out.push_back('\n'); continue; }
      if (c == 'r') { out.push_back('\r'); continue; }
      if (c == 't') { out.push_back('\t'); continue; }
      if (c == '0') { out.push_back('\0'); continue; }
      out.push_back(c);
      continue;
    }
    out.push_back(in[i]);
  }
  return out;
}

// Split on newlines, ',' and ';'. Strip '#' comments.
std::vector<std::string> split_events(const std::string &text) {
  std::vector<std::string> out;
  std::string cur;
  bool in_comment = false;
  for (char c : text) {
    if (c == '#') in_comment = true;
    if (c == '\n') { in_comment = false; }
    if (in_comment) { if (c != '\n') continue; }
    if (c == '\n' || c == ',' || c == ';') {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(cur);
  return out;
}

bool parse_text(const std::string &text) {
  uint64_t prev_release_us = 0;
  int lineno = 0;
  for (const std::string &raw : split_events(text)) {
    lineno++;
    // Tokenize on whitespace.
    std::vector<std::string> tok;
    {
      std::string t;
      for (char c : raw) {
        if (isspace((unsigned char)c)) {
          if (!t.empty()) { tok.push_back(t); t.clear(); }
        } else {
          t.push_back(c);
        }
      }
      if (!t.empty()) tok.push_back(t);
    }
    if (tok.empty()) continue; // blank / comment-only

    if (tok.size() < 2) {
      fprintf(stderr, "[input-script] entry %d: expected '<when> <KEY> [hold_ms]', got '%s'\n",
              lineno, raw.c_str());
      return false;
    }

    Event ev;
    const std::string &when = tok[0];
    if (when[0] == '+') {
      uint64_t gap = (when.size() > 1) ? strtoull(when.c_str() + 1, nullptr, 10)
                                       : kDefaultGapMs;
      ev.at_us = prev_release_us + gap * 1000ULL;
    } else {
      if (!isdigit((unsigned char)when[0])) {
        fprintf(stderr, "[input-script] entry %d: bad time '%s'\n", lineno, when.c_str());
        return false;
      }
      ev.at_us = strtoull(when.c_str(), nullptr, 10) * 1000ULL;
    }

    if (!parse_key(tok[1], &ev.key)) {
      fprintf(stderr, "[input-script] entry %d: unknown key '%s'\n", lineno, tok[1].c_str());
      return false;
    }

    if (ev.key == Key::Uart || ev.key == Key::UartFile) {
      // Everything after the keyword is the payload; keep interior spacing.
      size_t kpos = raw.find(tok[1]);
      std::string rest = (kpos == std::string::npos)
                             ? std::string()
                             : raw.substr(kpos + tok[1].size());
      size_t b = rest.find_first_not_of(" \t");
      rest = (b == std::string::npos) ? std::string() : rest.substr(b);
      while (!rest.empty() && (rest.back() == ' ' || rest.back() == '\t' ||
                               rest.back() == '\r'))
        rest.pop_back();
      ev.data = (ev.key == Key::Uart) ? unescape(rest) : rest;
      ev.until_us = ev.at_us;   // instantaneous, nothing to release
      prev_release_us = ev.at_us;
      g_events.push_back(ev);
      continue;
    }

    size_t hold_tok = 2;
    if (ev.key == Key::Tap) {
      if (tok.size() < 4) {
        fprintf(stderr, "[input-script] entry %d: expected '<when> TAP <x> <y> [hold_ms]'\n",
                lineno);
        return false;
      }
      // bdk's touch.c clamps raw samples to 15..1264 and 15..704 and
      // stretches that span over 1280x720; invert it.
      uint32_t x = std::min<uint32_t>(strtoul(tok[2].c_str(), nullptr, 10), 1279);
      uint32_t y = std::min<uint32_t>(strtoul(tok[3].c_str(), nullptr, 10), 719);
      ev.x = (uint16_t)(15 + (x * (1264 - 15) + 1279) / 1280);
      ev.y = (uint16_t)(15 + (y * (704 - 15) + 719) / 720);
      hold_tok = 4;
    }
    uint64_t hold_ms = (tok.size() > hold_tok)
                           ? strtoull(tok[hold_tok].c_str(), nullptr, 10)
                           : kDefaultHoldMs;
    if (hold_ms == 0) hold_ms = kDefaultHoldMs;
    ev.until_us = ev.at_us + hold_ms * 1000ULL;
    prev_release_us = ev.until_us;
    g_events.push_back(ev);
  }
  return !g_events.empty();
}

} // namespace

bool input_script_load(const char *spec) {
  if (!spec || !*spec) return false;
  g_events.clear();
  g_done = 0;

  std::string text;
  if (FILE *f = fopen(spec, "rb")) {
    char buf[4096];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
      text.append(buf, n);
    fclose(f);
    printf("[input-script] loaded from file: %s\n", spec);
  } else {
    text = spec; // treat as an inline spec
    printf("[input-script] inline spec: %s\n", spec);
  }

  if (!parse_text(text)) {
    fprintf(stderr, "[input-script] no usable events; script disabled\n");
    g_events.clear();
    return false;
  }

  g_active = true;
  printf("[input-script] %zu event(s) queued\n", g_events.size());
  return true;
}

bool input_script_active() { return g_active; }

void input_script_tick(EmuState &state) {
  if (!g_active || g_done >= g_events.size()) return;

  uint64_t now = state.emu_usec;
  for (size_t i = g_done; i < g_events.size(); i++) {
    Event &ev = g_events[i];
    // Events are ordered by press time; stop at the first future one.
    if (!ev.pressed && now < ev.at_us) break;

    // UART events are one-shot: push the bytes into UART_B's receive FIFO
    // (index 1 = UART_B, the port payloads use for their debug console) and
    // retire the event immediately.
    if (ev.key == Key::Uart || ev.key == Key::UartFile) {
      if (now >= ev.at_us && !ev.pressed) {
        std::string bytes = ev.data;
        if (ev.key == Key::UartFile) {
          bytes.clear();
          if (FILE *f = fopen(ev.data.c_str(), "rb")) {
            char buf[4096];
            size_t n;
            while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
              bytes.append(buf, n);
            fclose(f);
          } else {
            fprintf(stderr, "[input-script] cannot open UARTFILE '%s'\n",
                    ev.data.c_str());
          }
        }
        for (unsigned char ch : bytes)
          state.uart_rx_fifo[1].push_back(ch);
        ev.pressed = ev.released = true;
        printf("[input-script] UART_B <- %zu byte(s) @%llu us (%zu/%zu)\n",
               bytes.size(), (unsigned long long)now, i + 1, g_events.size());
        fflush(stdout);
        if (i == g_done) {
          g_done++;
          if (g_done == g_events.size())
            printf("[input-script] sequence complete\n");
        }
        continue;
      }
      break;
    }

    if (ev.key == Key::Tap) {
      if (!ev.pressed && now >= ev.at_us) {
        state.tc_pressed.store(true);
        state.touch_post(0x03, ev.x, ev.y);     // FTS4 ENTER
        ev.pressed = true;
        printf("[input-script] TAP down panel=(%u,%u) @%llu us (%zu/%zu)\n",
               ev.x, ev.y, (unsigned long long)now, i + 1, g_events.size());
        fflush(stdout);
        break;
      }
      if (ev.pressed && !ev.released && now >= ev.until_us) {
        state.tc_pressed.store(false);
        state.touch_post(0x04, ev.x, ev.y);     // FTS4 LEAVE
        ev.released = true;
        if (i == g_done) {
          g_done++;
          if (g_done == g_events.size())
            printf("[input-script] sequence complete\n");
        }
      }
      if (!ev.released) break;
      continue;
    }

    std::atomic<bool> *btn = (ev.key == Key::Power)   ? &state.btn_power
                             : (ev.key == Key::VolUp) ? &state.btn_vol_up
                                                      : &state.btn_vol_down;
    if (!ev.pressed && now >= ev.at_us) {
      btn->store(true);
      ev.pressed = true;
      printf("[input-script] %s down @%llu us (%zu/%zu)\n", key_name(ev.key),
             (unsigned long long)now, i + 1, g_events.size());
      fflush(stdout);
      // Even if the release is already due - a FLOW_CTLR sleep can jump the
      // clock past it within one batch - the payload has to run with the
      // button down at least once, or it never sees the press.
      break;
    }
    if (ev.pressed && !ev.released && now >= ev.until_us) {
      btn->store(false);
      ev.released = true;
      if (i == g_done) {
        g_done++;
        if (g_done == g_events.size())
          printf("[input-script] sequence complete\n");
      }
    }
    if (!ev.released) break; // keep this event current until it's let go
  }
}

void input_script_restart(EmuState &state) {
  for (Event &ev : g_events)
    ev.pressed = ev.released = false;
  g_done = 0;
  state.btn_power = false;
  state.btn_vol_up = false;
  state.btn_vol_down = false;
}
