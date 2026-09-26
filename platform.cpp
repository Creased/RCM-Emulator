#include "platform.h"

#include <cstdio>
#include <cstring>
#include <string>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#endif

// ---- Zero-filled memory, committed only where touched --------------------
//
// The emulated DRAM window is 2 GiB. calloc gets it from the OS as demand-
// zero pages, so only what a payload touches becomes resident - but a soft
// reboot used to memset() the whole block back to zero, which touched every
// page and left the process 2 GiB resident from the first reboot on. These
// allocate the same demand-zero memory directly, and hand back fresh zero
// pages on reset, at the same address (the engines hold host pointers into
// it), without touching any.

uint8_t *zeroed_alloc(size_t size) {
#ifdef _WIN32
  return (uint8_t *)VirtualAlloc(nullptr, size, MEM_RESERVE | MEM_COMMIT,
                                 PAGE_READWRITE);
#else
  void *p = mmap(nullptr, size, PROT_READ | PROT_WRITE,
                 MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
  return p == MAP_FAILED ? nullptr : (uint8_t *)p;
#endif
}

void zeroed_reset(uint8_t *p, size_t size) {
#ifdef _WIN32
  // Decommitted pages come back zero-filled when committed again.
  if (!VirtualFree(p, size, MEM_DECOMMIT) ||
      !VirtualAlloc(p, size, MEM_COMMIT, PAGE_READWRITE))
    memset(p, 0, size);
#else
  // A fresh anonymous mapping over the old one: zero pages, same address.
  if (mmap(p, size, PROT_READ | PROT_WRITE,
           MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE | MAP_FIXED, -1,
           0) == MAP_FAILED)
    memset(p, 0, size);
#endif
}

void zeroed_free(uint8_t *p, size_t size) {
  if (!p)
    return;
#ifdef _WIN32
  (void)size;
  VirtualFree(p, 0, MEM_RELEASE);
#else
  munmap(p, size);
#endif
}

// ---- Opening files by name -------------------------------------------------

FILE *platform_fopen(const char *path, const char *mode) {
  FILE *f = fopen(path, mode);
#ifdef _WIN32
  // Command-line paths arrive in the ANSI code page, which fopen expects;
  // SDL hands dropped files over as UTF-8, which it does not. Try the name
  // as UTF-8 before giving up.
  if (!f) {
    int n = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path, -1,
                                nullptr, 0);
    int m = MultiByteToWideChar(CP_UTF8, 0, mode, -1, nullptr, 0);
    if (n > 0 && m > 0) {
      std::wstring wpath(n, L'\0'), wmode(m, L'\0');
      MultiByteToWideChar(CP_UTF8, 0, path, -1, &wpath[0], n);
      MultiByteToWideChar(CP_UTF8, 0, mode, -1, &wmode[0], m);
      f = _wfopen(wpath.c_str(), wmode.c_str());
    }
  }
#endif
  return f;
}
