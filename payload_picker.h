#pragma once
#include <string>

// Shown when the emulator is started with no payload argument. Returns the
// chosen path, or an empty string if the user closed the window without
// picking one.
std::string payload_picker_run(void);
