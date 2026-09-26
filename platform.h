#ifndef RCM_PLATFORM_H
#define RCM_PLATFORM_H

#include <cstddef>
#include <cstdint>
#include <cstdio>
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

// pread/pwrite are POSIX and MinGW does not have them. Seek-then-read is
// equivalent here: the emulated storage is touched only from the CPU thread,
// so the atomicity real pread() buys against a shared file offset is not in
// play.
#ifdef _WIN32
#include <io.h>
static inline ssize_t pread(int fd, void *buf, size_t n, long long off) {
  if (_lseeki64(fd, off, SEEK_SET) < 0)
    return -1;
  return _read(fd, buf, (unsigned int)n);
}
static inline ssize_t pwrite(int fd, const void *buf, size_t n, long long off) {
  if (_lseeki64(fd, off, SEEK_SET) < 0)
    return -1;
  return _write(fd, buf, (unsigned int)n);
}
#else
#include <unistd.h>
#endif

// Large zero-filled memory that only becomes resident where it is touched
// (platform.cpp). zeroed_reset() makes all of it zero again, at the same
// address, without touching it.
uint8_t *zeroed_alloc(size_t size);
void     zeroed_reset(uint8_t *p, size_t size);
void     zeroed_free(uint8_t *p, size_t size);

// fopen() that also accepts a UTF-8 name on Windows (a dropped file).
FILE    *platform_fopen(const char *path, const char *mode);

#endif // RCM_PLATFORM_H
