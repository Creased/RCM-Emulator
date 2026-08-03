// Scripted button input for headless / CI runs.
//
// Menu-driven payloads (hwtest-style test suites) can't be exercised without
// pressing buttons, which makes them impossible to verify from a script or a
// CI job. This module replays a deterministic button sequence against the
// emulated VOL+/VOL-/POWER lines, keyed off the emulator's own microsecond
// clock (emu_usec) so runs are reproducible regardless of host speed.
//
// Enabled with `--input-script <file-or-inline-spec>`. See input_script.cpp
// for the accepted syntax.

#pragma once

struct EmuState;

// Load a script from a file path, or - if the argument isn't a readable file -
// parse it as an inline spec. Returns false (and prints why) on a parse error.
bool input_script_load(const char *spec);

// True when a script was loaded successfully.
bool input_script_active();

// Apply any events that are due at the current state.emu_usec. Call once per
// emulator loop iteration; cheap no-op when no script is loaded.
void input_script_tick(EmuState &state);
