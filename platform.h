#ifndef RCM_PLATFORM_H
#define RCM_PLATFORM_H

#include <cstdint>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

// Small portability helpers for the disk-image code, which is the one place
// the emulator does raw POSIX file I/O.

// MinGW's open() defaults to TEXT mode: _read() turns every CR LF into LF
// and stops at the first 0x1A byte. On an SD card or eMMC image that is
// silent sector corruption, so every image fd must be opened O_BINARY.
// POSIX has no such flag (and no such mode), so it is a no-op there.
#ifndef O_BINARY
#define O_BINARY 0
#endif

// Size of an open file, in 64 bits. MinGW's plain struct stat carries a
// 32-bit st_size, which cannot describe the 4 GiB chunks Hekate splits a
// rawnand dump into; _fstati64 can.
static inline int64_t file_size64(int fd) {
#ifdef _WIN32
  struct _stati64 st;
  if (_fstati64(fd, &st) != 0)
    return -1;
  return (int64_t)st.st_size;
#else
  struct stat st;
  if (fstat(fd, &st) != 0)
    return -1;
  return (int64_t)st.st_size;
#endif
}

#endif // RCM_PLATFORM_H
