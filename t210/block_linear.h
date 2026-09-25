#pragma once

#include <cstddef>
#include <cstdint>

// Tegra X1 block-linear surfaces (TRM 20.1.2), as the display controller and
// VIC read and write them. A GOB is 64 bytes x 8 lines (512 bytes) of 16Bx2
// sectors; GOBs stack gob_h high into a block, and blocks run left to right,
// then down, gobs_per_row GOBs to a row.

// Byte offset of byte column xb (pixel x * bytes per pixel) on line y.
inline size_t bl_offset(uint32_t xb, uint32_t y, uint32_t gobs_per_row,
                        uint32_t gob_h) {
  uint32_t gob_y = y / 8;
  size_t gob = ((size_t)(gob_y / gob_h) * gobs_per_row + xb / 64) * gob_h +
               gob_y % gob_h;
  // Within the GOB, TRM Figure 47's sector order.
  uint32_t x = xb % 64, l = y % 8;
  return gob * 512 + (x / 32) * 256 + (l / 2) * 64 + ((x % 32) / 16) * 32 +
         (l % 2) * 16 + (x % 16);
}

// Bytes a surface h lines tall occupies (whole blocks).
inline size_t bl_size(uint32_t gobs_per_row, uint32_t gob_h, uint32_t h) {
  return (size_t)gobs_per_row * 512 * gob_h *
         ((h + 8 * gob_h - 1) / (8 * gob_h));
}
