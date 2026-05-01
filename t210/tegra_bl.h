#ifndef T210_TEGRA_BL_H
#define T210_TEGRA_BL_H

#include <cstdint>

// Tegra X1-style block-linear byte offset for 32 bpp (RGBA8888 / XRGB in RAM).
// Matches test_img.py "Block Linear / Tegra X1" mode.
inline uint32_t tegra_bl_byte_off_rgba8888(uint32_t sx, uint32_t sy,
                                           uint32_t width_gobs,
                                           uint32_t block_height_gobs) {
  uint32_t gob_x = sx / 16u;
  uint32_t pixel_x = sx % 16u;
  uint32_t gob_y = sy / 8u;
  uint32_t line_y = sy % 8u;
  uint32_t block_y = gob_y / block_height_gobs;
  uint32_t gob_in_block_y = gob_y % block_height_gobs;

  uint32_t idx =
      (block_y * width_gobs * block_height_gobs * 128u) +
      (gob_x * block_height_gobs * 128u) + (gob_in_block_y * 128u) +
      (pixel_x & 3u) + ((line_y & 1u) << 2) + ((pixel_x & 4u) << 1) +
      ((line_y & 2u) << 3) + ((pixel_x & 8u) << 2) + ((line_y & 4u) << 4);
  return idx * 4u;
}

#endif
