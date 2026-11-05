#include "gic.h"

#include <cstdio>
#include <cstring>

/*
 * GIC-400 model. See gic.h. Register offsets and reset values: ARM IHI 0048B
 * (GICv2 architecture) ch.4, GIC-400 TRM (DDI 0471B) table 3-1 / 3-6.
 */

namespace {

struct Gic {
  uint32_t dctlr = 0;
  uint32_t cctlr = 0;
  uint32_t pmr = 0;
  uint32_t bpr = 2;
  uint32_t group[GIC_NR_IRQS / 32] = {};
  bool enabled[GIC_NR_IRQS] = {};
  bool latched[GIC_NR_IRQS] = {};   // edge, SGI or software-set pending
  bool line[GIC_NR_IRQS] = {};      // level input
  bool active[GIC_NR_IRQS] = {};
  uint8_t prio[GIC_NR_IRQS] = {};
  uint8_t target[GIC_NR_IRQS] = {};
  uint8_t cfg[GIC_NR_IRQS] = {};    // 2 bits each, bit 1 set = edge
  // Active priorities, most recent last: the running priority is the
  // highest (numerically lowest) of them.
  int stack[GIC_NR_IRQS] = {};
  int depth = 0;
};

Gic g;

// CPU0 asks "is an interrupt pending?" before every translated block, so the
// answer is cached and recomputed only after something changed.
bool g_dirty = true;
int g_best = 1023;
bool g_line_up = false;

bool is_edge(int id) { return id < 16 || (g.cfg[id] & 2); }

bool pending(int id) {
  if (g.latched[id])
    return true;
  return !is_edge(id) && g.line[id];
}

int running_prio() {
  int p = 0x100;
  for (int i = 0; i < g.depth; i++)
    if (g.prio[g.stack[i]] < p)
      p = g.prio[g.stack[i]];
  return p;
}

int best_scan() {
  int bestid = 1023, bestp = 0x100;
  for (int id = 0; id < GIC_NR_IRQS; id++) {
    if (!g.enabled[id] || g.active[id] || !pending(id))
      continue;
    if (g.prio[id] < bestp) {
      bestp = g.prio[id];
      bestid = id;
    }
  }
  return bestid;
}

int best() {
  if (g_dirty) {
    g_best = best_scan();
    g_line_up = false;
    if ((g.dctlr & 3) && (g.cctlr & 3) && g_best != 1023)
      g_line_up = g.prio[g_best] < g.pmr && g.prio[g_best] < running_prio();
    g_dirty = false;
  }
  return g_best;
}

uint32_t bits_read(const bool *v, uint32_t word) {
  uint32_t r = 0;
  for (int b = 0; b < 32; b++)
    if (v[word * 32 + b])
      r |= 1u << b;
  return r;
}

uint32_t pend_read(uint32_t word) {
  uint32_t r = 0;
  for (int b = 0; b < 32; b++)
    if (pending(word * 32 + b))
      r |= 1u << b;
  return r;
}

// Byte-array registers (priority, targets) accessed by byte or by word.
uint32_t bytes_read(const uint8_t *a, uint32_t off, unsigned size) {
  uint32_t r = 0;
  for (unsigned i = 0; i < size && off + i < GIC_NR_IRQS; i++)
    r |= (uint32_t)a[off + i] << (8 * i);
  return r;
}

void bytes_write(uint8_t *a, uint32_t off, unsigned size, uint32_t v,
                 uint8_t mask) {
  for (unsigned i = 0; i < size && off + i < GIC_NR_IRQS; i++)
    a[off + i] = (uint8_t)((v >> (8 * i)) & mask);
}

} // namespace

void gic_reset() {
  g = Gic();
  g_dirty = true;
  for (int id = 0; id < 32; id++)
    g.target[id] = 0x01;            // banked: reads this CPU's own bit
  for (int id = 0; id < 16; id++)
    g.enabled[id] = true;           // SGIs are always enabled on GIC-400
  for (int id = 16; id < 32; id++)
    g.cfg[id] = 0;                  // PPIs level
}

uint32_t gic_dist_read(uint32_t off, unsigned size) {
  if (off >= 0x400 && off < 0x400 + GIC_NR_IRQS)
    return bytes_read(g.prio, off - 0x400, size);
  if (off >= 0x800 && off < 0x800 + GIC_NR_IRQS) {
    uint32_t i = off - 0x800;
    if (i < 32)
      return bytes_read(g.target, i, size);
    return bytes_read(g.target, i, size);
  }
  uint32_t w = (off & 0x7F) >> 2;
  switch (off & ~0x7Fu) {
  case 0x080: return w < GIC_NR_IRQS / 32 ? g.group[w] : 0;
  case 0x100: case 0x180: return w < GIC_NR_IRQS / 32 ? bits_read(g.enabled, w) : 0;
  case 0x200: case 0x280: return w < GIC_NR_IRQS / 32 ? pend_read(w) : 0;
  case 0x300: case 0x380: return w < GIC_NR_IRQS / 32 ? bits_read(g.active, w) : 0;
  default: break;
  }
  if (off >= 0xC00 && off < 0xC00 + GIC_NR_IRQS / 4) {
    uint32_t base = (off - 0xC00) * 4, r = 0;
    for (int i = 0; i < 16; i++)
      r |= (uint32_t)(g.cfg[base + i] & 3) << (2 * i);
    return r;
  }
  switch (off) {
  case 0x000: return g.dctlr;
  case 0x004: return (1u << 10) | (3u << 5) | (GIC_NR_IRQS / 32 - 1); // SecExt, 4 CPUs
  case 0x008: return 0x0200143B;    // GIC-400, implementer ARM
  case 0xFE8: return 0x2B;          // PIDR2: GICv2
  case 0xFD0: return 0x04;
  case 0xFE0: return 0x90;
  case 0xFE4: return 0xB4;
  case 0xFEC: return 0x00;
  case 0xFF0: return 0x0D;
  case 0xFF4: return 0xF0;
  case 0xFF8: return 0x05;
  case 0xFFC: return 0xB1;
  default: return 0;
  }
}

void gic_dist_write(uint32_t off, unsigned size, uint32_t val) {
  g_dirty = true;
  if (off >= 0x400 && off < 0x400 + GIC_NR_IRQS) {
    bytes_write(g.prio, off - 0x400, size, val, 0xF8); // 5 priority bits
    return;
  }
  if (off >= 0x800 && off < 0x800 + GIC_NR_IRQS) {
    uint32_t i = off - 0x800;
    if (i >= 32)
      bytes_write(g.target, i, size, val, 0xFF);
    return;
  }
  if (off >= 0xC00 && off < 0xC00 + GIC_NR_IRQS / 4) {
    uint32_t base = (off - 0xC00) * 4;
    for (int i = 0; i < 16; i++)
      if (base + i >= 16)           // SGI configuration is read-only
        g.cfg[base + i] = (uint8_t)((val >> (2 * i)) & 3);
    return;
  }
  uint32_t w = (off & 0x7F) >> 2;
  bool inrange = w < GIC_NR_IRQS / 32;
  switch (off & ~0x7Fu) {
  case 0x080: if (inrange) g.group[w] = val; return;
  case 0x100:
    if (inrange) for (int b = 0; b < 32; b++) if (val & (1u << b)) g.enabled[w * 32 + b] = true;
    return;
  case 0x180:
    if (inrange) for (int b = 0; b < 32; b++)
      if ((val & (1u << b)) && w * 32 + b >= 16) g.enabled[w * 32 + b] = false;
    return;
  case 0x200:
    if (inrange) for (int b = 0; b < 32; b++) if (val & (1u << b)) g.latched[w * 32 + b] = true;
    return;
  case 0x280:
    if (inrange) for (int b = 0; b < 32; b++) if (val & (1u << b)) g.latched[w * 32 + b] = false;
    return;
  case 0x300:
    if (inrange) for (int b = 0; b < 32; b++) if (val & (1u << b)) g.active[w * 32 + b] = true;
    return;
  case 0x380:
    if (inrange) for (int b = 0; b < 32; b++) if (val & (1u << b)) g.active[w * 32 + b] = false;
    return;
  default: break;
  }
  switch (off) {
  case 0x000: g.dctlr = val & 3; return;
  case 0xF00: {                     // GICD_SGIR
    int id = val & 0xF;
    uint32_t filter = (val >> 24) & 3, targets = (val >> 16) & 0xFF;
    bool to_me = filter == 2 || (filter == 0 && (targets & 1));
    if (to_me)
      g.latched[id] = true;
    return;
  }
  default: return;
  }
}

uint32_t gic_cpu_read(uint32_t off, unsigned size) {
  (void)size;
  if (off == 0x0C || off == 0x20)
    g_dirty = true;
  switch (off) {
  case 0x00: return g.cctlr;
  case 0x04: return g.pmr;
  case 0x08: return g.bpr;
  case 0x0C: case 0x20: {           // IAR / AIAR: acknowledge
    if (!(g.dctlr & 3) || !(g.cctlr & 3))
      return 1023;
    int id = best();
    if (id == 1023 || g.prio[id] >= g.pmr || g.prio[id] >= running_prio())
      return 1023;
    g.active[id] = true;
    g.latched[id] = false;
    if (g.depth < GIC_NR_IRQS)
      g.stack[g.depth++] = id;
    return (uint32_t)id;            // SGI source CPU field: CPU0, i.e. 0
  }
  case 0x14: return (uint32_t)(running_prio() & 0xFF);
  case 0x18: case 0x28: return (uint32_t)best();
  case 0x1C: return g.bpr;
  case 0xFC: return 0x0202143B;     // GICC_IIDR, GIC-400
  default: return 0;
  }
}

void gic_cpu_write(uint32_t off, unsigned size, uint32_t val) {
  (void)size;
  g_dirty = true;
  switch (off) {
  case 0x00: g.cctlr = val & 0x3FF; return;
  case 0x04: g.pmr = val & 0xF8; return;
  case 0x08: case 0x1C: g.bpr = val & 7; return;
  case 0x10: case 0x24: case 0x1000: {  // EOIR / AEOIR / DIR
    int id = (int)(val & 0x3FF);
    if (id >= GIC_NR_IRQS)
      return;
    g.active[id] = false;
    for (int i = g.depth - 1; i >= 0; i--)
      if (g.stack[i] == id) {
        memmove(&g.stack[i], &g.stack[i + 1], (g.depth - i - 1) * sizeof(int));
        g.depth--;
        break;
      }
    return;
  }
  default: return;
  }
}

void gic_set_spi(int spi, bool level) {
  int id = 32 + spi;
  if (spi < 0 || id >= GIC_NR_IRQS)
    return;
  if (g.line[id] == level)
    return;
  if (level && is_edge(id))
    g.latched[id] = true;           // rising edge of an edge-configured line
  g.line[id] = level;
  g_dirty = true;
}

void gic_set_ppi(int id, bool level) {
  if (id < 16 || id >= 32 || g.line[id] == level)
    return;
  g.line[id] = level;
  g_dirty = true;
}

bool gic_irq_pending() {
  best();
  return g_line_up;
}

int gic_highest_pending() { return best(); }

void gic_describe(int id, char *buf, unsigned n) {
  if (id < 0 || id >= GIC_NR_IRQS) {
    snprintf(buf, n, "?");
    return;
  }
  snprintf(buf, n, "dctlr %u cctlr %u pmr %02X id%d en %d line %d latch %d act %d prio %02X depth %d",
           g.dctlr, g.cctlr, g.pmr, id, g.enabled[id], g.line[id], g.latched[id],
           g.active[id], g.prio[id], g.depth);
}
