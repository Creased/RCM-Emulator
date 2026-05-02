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

.PHONY: all clean

all: $(OUTPUT)

$(OUTPUT): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Pull in the header-dep .d sidecars produced by -MMD. Silenced on first build
# (or after `make clean`) since the files don't exist yet.
-include $(OBJS:.o=.d)

clean:
	rm -f $(OUTPUT) $(OBJS) $(OBJS:.o=.d)
