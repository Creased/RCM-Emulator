# RCM Payload Emulator
# ARM32 emulator for Switch RCM payloads using Unicorn Engine + SDL2

SRC_DIR = . t210 display
SRCS = $(foreach dir,$(SRC_DIR),$(wildcard $(dir)/*.cpp))

# Vendored Dear ImGui (see third_party/imgui/). Listed explicitly so we don't
# accidentally pick up imgui_demo.cpp etc. if they're added later.
IMGUI_DIR = third_party/imgui
IMGUI_SRCS = \
    $(IMGUI_DIR)/imgui.cpp \
    $(IMGUI_DIR)/imgui_draw.cpp \
    $(IMGUI_DIR)/imgui_tables.cpp \
    $(IMGUI_DIR)/imgui_widgets.cpp \
    $(IMGUI_DIR)/backends/imgui_impl_sdl2.cpp \
    $(IMGUI_DIR)/backends/imgui_impl_sdlrenderer2.cpp

SRCS += $(IMGUI_SRCS)
OBJS = $(SRCS:.cpp=.o)

OUTPUT = rcm_emu

CXX = g++
# -MMD -MP emits .d sidecar files with each .o so a header change forces a
# rebuild of every TU that includes it. Avoids stale-layout bugs when EmuState
# (or any other shared header) gains a new field.
CXXFLAGS = -Wall -g -O2 -std=c++17 -MMD -MP -I. -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends
LDFLAGS =
LIBS = -lunicorn -lSDL2 -lpthread

# pkg-config for SDL2 if available
SDL2_CFLAGS := $(shell pkg-config --cflags sdl2 2>/dev/null)
SDL2_LIBS := $(shell pkg-config --libs sdl2 2>/dev/null)

ifneq ($(SDL2_CFLAGS),)
    CXXFLAGS += $(SDL2_CFLAGS)
    LIBS = -lunicorn $(SDL2_LIBS) -lpthread
endif

# ---- Windows cross-build ---------------------------------------------------
#
# `make windows` builds rcm_emu.exe with the MinGW toolchain. Both deps come
# from the cross sysroot the container prepares (see Dockerfile.windows):
# SDL2 from its official MinGW devel tree, unicorn built from source.
#
# Statically linked on purpose. A .exe that needs libstdc++, libgcc, winpthread
# and SDL2.dll beside it is not something anyone wants to download from a
# release page, and SDL2 is linked static here too so there is exactly one
# file to ship.
#
# SDL on Windows renames main to SDL_main and supplies its own entry point, so
# the link order -lmingw32 -lSDL2main -lSDL2 matters and SDL2main must come
# before SDL2. The trailing Windows libraries are what static SDL2 and unicorn
# pull in: without them the link fails on missing WinMM / OLE / socket symbols.
WIN_CXX  = x86_64-w64-mingw32-g++
WIN_OUT  = rcm_emu.exe
WIN_OBJS = $(SRCS:.cpp=.win.o)
WIN_CXXFLAGS = -Wall -g -O2 -std=c++17 -MMD -MP -I. -I$(IMGUI_DIR)                -I$(IMGUI_DIR)/backends -I/usr/x86_64-w64-mingw32/include/SDL2                -DSDL_MAIN_HANDLED
WIN_LIBS = -lmingw32 -lSDL2main -lSDL2 -lunicorn -lpthread            -lwinmm -limm32 -lole32 -loleaut32 -lversion -lsetupapi -lcfgmgr32            -lgdi32 -lrpcrt4 -lws2_32 -luuid -lshell32 -ladvapi32 -luser32
WIN_LDFLAGS = -static -static-libgcc -static-libstdc++

.PHONY: all clean windows

all: $(OUTPUT)

windows: $(WIN_OUT)

$(WIN_OUT): $(WIN_OBJS)
	$(WIN_CXX) $(WIN_LDFLAGS) -o $@ $^ $(WIN_LIBS)
	@echo "   built $@ ($$(stat -c%s $@) bytes)"

%.win.o: %.cpp
	$(WIN_CXX) $(WIN_CXXFLAGS) -c -o $@ $<

-include $(WIN_OBJS:.o=.d)

$(OUTPUT): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Pull in the header-dep .d sidecars produced by -MMD. Silenced on first build
# (or after `make clean`) since the files don't exist yet.
-include $(OBJS:.o=.d)

clean:
	rm -f $(OUTPUT) $(OBJS) $(OBJS:.o=.d) \n	      $(WIN_OUT) $(WIN_OBJS) $(WIN_OBJS:.o=.d)
