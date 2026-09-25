#ifndef T210_REGCACHE_H
#define T210_REGCACHE_H

#include <cstdint>
#include <cstring>
#include <memory>
#include <unordered_map>

// Last-written value of every MMIO address, for the registers whose model is
// "read back what the payload wrote" (PINMUX, PWM, UART LCR, CLK_SOURCE_*...).
//
// This sits on the hottest path in the emulator - every MMIO write stores
// into it and a good share of reads consult it - and it used to be a
// std::map<uint64_t, uint32_t>: a red-black tree walk plus, for a new key, a
// heap allocation per access. Profiling a hwtest sweep put the tree lookups
// on par with the peripheral models themselves.
//
// Registers cluster into a few dozen 4 KiB pages, so this keeps one flat page
// per touched page with a presence bitmap, and caches the last page looked
// up. The interface is the subset of std::map the call sites use (count() and
// operator[]), so they did not have to change. Keys stay byte-granular: a
// byte store at 0x70006001 is its own entry, exactly as it was in the map.
class RegCache {
  struct Page {
    uint32_t val[4096];
    uint64_t present[4096 / 64];
  };

  std::unordered_map<uint64_t, std::unique_ptr<Page>> pages_;
  mutable uint64_t last_key_ = ~0ULL;
  mutable Page *last_ = nullptr;

  Page *find(uint64_t key) const {
    if (key == last_key_)
      return last_;
    auto it = pages_.find(key);
    if (it == pages_.end())
      return nullptr;
    last_key_ = key;
    last_ = it->second.get();
    return last_;
  }

public:
  size_t count(uint64_t addr) const {
    const Page *p = find(addr >> 12);
    if (!p)
      return 0;
    uint32_t o = (uint32_t)(addr & 0xFFF);
    return (p->present[o >> 6] >> (o & 63)) & 1;
  }

  uint32_t &operator[](uint64_t addr) {
    uint64_t key = addr >> 12;
    Page *p = find(key);
    if (!p) {
      auto page = std::make_unique<Page>();
      memset(page.get(), 0, sizeof(Page));
      p = page.get();
      pages_.emplace(key, std::move(page));
      last_key_ = key;
      last_ = p;
    }
    uint32_t o = (uint32_t)(addr & 0xFFF);
    p->present[o >> 6] |= 1ULL << (o & 63);
    return p->val[o];
  }

  // Value if ever written, else `dflt`. One lookup instead of the
  // count()-then-operator[] pair.
  uint32_t get(uint64_t addr, uint32_t dflt = 0) const {
    const Page *p = find(addr >> 12);
    if (!p)
      return dflt;
    uint32_t o = (uint32_t)(addr & 0xFFF);
    return ((p->present[o >> 6] >> (o & 63)) & 1) ? p->val[o] : dflt;
  }

  void clear() {
    pages_.clear();
    last_key_ = ~0ULL;
    last_ = nullptr;
  }
};

#endif // T210_REGCACHE_H
