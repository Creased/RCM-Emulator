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
CXXFLAGS = -Wall -g -O2 -std=c++17 -I. -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends
LDFLAGS =
LIBS = -lunicorn -lSDL2 -lpthread

# pkg-config for SDL2 if available
SDL2_CFLAGS := $(shell pkg-config --cflags sdl2 2>/dev/null)
SDL2_LIBS := $(shell pkg-config --libs sdl2 2>/dev/null)

ifneq ($(SDL2_CFLAGS),)
    CXXFLAGS += $(SDL2_CFLAGS)
    LIBS = -lunicorn $(SDL2_LIBS) -lpthread
endif

.PHONY: all clean

all: $(OUTPUT)

$(OUTPUT): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)

# Header dependencies
main.o: main.cpp emu_state.h t210/memory_map.h t210/mmio.h display/sdl_display.h display/config_window.h
t210/mmio.o: t210/mmio.cpp t210/mmio.h t210/memory_map.h t210/tegra_bl.h emu_state.h
display/sdl_display.o: display/sdl_display.cpp display/sdl_display.h display/config_window.h emu_state.h t210/tegra_bl.h
display/config_window.o: display/config_window.cpp display/config_window.h emu_state.h

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OUTPUT) $(OBJS)
