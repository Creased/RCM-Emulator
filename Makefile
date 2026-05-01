# RCM Payload Emulator
# ARM32 emulator for Switch RCM payloads using Unicorn Engine + SDL2

SRC_DIR = . t210 display
SRCS = $(foreach dir,$(SRC_DIR),$(wildcard $(dir)/*.cpp))
OBJS = $(SRCS:.cpp=.o)

OUTPUT = rcm_emu

CXX = g++
CXXFLAGS = -Wall -g -O2 -std=c++17 -I.
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
main.o: main.cpp emu_state.h t210/memory_map.h t210/mmio.h display/sdl_display.h
t210/mmio.o: t210/mmio.cpp t210/mmio.h t210/memory_map.h t210/tegra_bl.h emu_state.h
display/sdl_display.o: display/sdl_display.cpp display/sdl_display.h emu_state.h t210/tegra_bl.h

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OUTPUT) $(OBJS)
