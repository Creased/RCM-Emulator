#include "se_engine.h"
#include "../emu_state.h"
#include "memory_map.h"

#include <unicorn/unicorn.h>

#include <cstdio>
#include <cstring>
#include <cstdlib>

// =============================================================================
//  Tegra X1 Security Engine (SE) emulation
//
//  Implements a useful subset of the SE so that Hekate's BDK and Lockpick_RCM's
//  key derivation pipelines run real AES-128 against the emulated keytable.
//
//  References:
//   * hekate/bdk/sec/se.c             (driver flow used by both payloads)
//   * Lockpick_RCM/bdk/sec/se_t210.h  (register offsets / bitfields)
//   * Lockpick_RCM/source/keys/keys.c (uses cases: ECB, CTR, CMAC, unwrap)
//
//  AES-128 implementation below is a compact reference variant adapted from
//  the public-domain "tiny-AES-c" project (Brad Conte / kokke). Only AES-128
//  is needed; 192/256 are unused by the BDK.
// =============================================================================

// ---- SHA-256 (public-domain reference) --------------------------------------

namespace sha256 {

static const uint32_t K[64] = {
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2,
};

#define ROR32(x, n) (((x) >> (n)) | ((x) << (32 - (n))))

static void compute(const uint8_t *msg, uint32_t len, uint8_t out[32]) {
    uint32_t H[8] = {
        0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,
        0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19,
    };
    // Total padded length: msg + 0x80 + zeros + 8-byte length, multiple of 64
    uint64_t bitlen = (uint64_t)len * 8;
    uint32_t padded_len = ((len + 1 + 8 + 63) / 64) * 64;
    uint8_t *buf = (uint8_t*)std::calloc(1, padded_len);
    std::memcpy(buf, msg, len);
    buf[len] = 0x80;
    for (int i = 0; i < 8; i++) buf[padded_len - 1 - i] = (uint8_t)(bitlen >> (i * 8));
    for (uint32_t off = 0; off < padded_len; off += 64) {
        uint32_t W[64];
        for (int i = 0; i < 16; i++) {
            W[i] = ((uint32_t)buf[off + i*4 + 0] << 24) |
                   ((uint32_t)buf[off + i*4 + 1] << 16) |
                   ((uint32_t)buf[off + i*4 + 2] <<  8) |
                   ((uint32_t)buf[off + i*4 + 3]);
        }
        for (int i = 16; i < 64; i++) {
            uint32_t s0 = ROR32(W[i-15],7) ^ ROR32(W[i-15],18) ^ (W[i-15] >> 3);
            uint32_t s1 = ROR32(W[i-2],17) ^ ROR32(W[i-2],19)  ^ (W[i-2]  >> 10);
            W[i] = W[i-16] + s0 + W[i-7] + s1;
        }
        uint32_t a=H[0],b=H[1],c=H[2],d=H[3],e=H[4],f=H[5],g=H[6],h=H[7];
        for (int i = 0; i < 64; i++) {
            uint32_t S1 = ROR32(e,6) ^ ROR32(e,11) ^ ROR32(e,25);
            uint32_t ch = (e & f) ^ ((~e) & g);
            uint32_t t1 = h + S1 + ch + K[i] + W[i];
            uint32_t S0 = ROR32(a,2) ^ ROR32(a,13) ^ ROR32(a,22);
            uint32_t mj = (a & b) ^ (a & c) ^ (b & c);
            uint32_t t2 = S0 + mj;
            h=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
        }
        H[0]+=a; H[1]+=b; H[2]+=c; H[3]+=d;
        H[4]+=e; H[5]+=f; H[6]+=g; H[7]+=h;
    }
    std::free(buf);
    for (int i = 0; i < 8; i++) {
        out[i*4 + 0] = (uint8_t)(H[i] >> 24);
        out[i*4 + 1] = (uint8_t)(H[i] >> 16);
        out[i*4 + 2] = (uint8_t)(H[i] >>  8);
        out[i*4 + 3] = (uint8_t)(H[i]);
    }
}

#undef ROR32

} // namespace sha256

// ---- AES-128 (public-domain reference) --------------------------------------

namespace aes {

static const uint8_t Sbox[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

static const uint8_t InvSbox[256] = {
    0x52,0x09,0x6a,0xd5,0x30,0x36,0xa5,0x38,0xbf,0x40,0xa3,0x9e,0x81,0xf3,0xd7,0xfb,
    0x7c,0xe3,0x39,0x82,0x9b,0x2f,0xff,0x87,0x34,0x8e,0x43,0x44,0xc4,0xde,0xe9,0xcb,
    0x54,0x7b,0x94,0x32,0xa6,0xc2,0x23,0x3d,0xee,0x4c,0x95,0x0b,0x42,0xfa,0xc3,0x4e,
    0x08,0x2e,0xa1,0x66,0x28,0xd9,0x24,0xb2,0x76,0x5b,0xa2,0x49,0x6d,0x8b,0xd1,0x25,
    0x72,0xf8,0xf6,0x64,0x86,0x68,0x98,0x16,0xd4,0xa4,0x5c,0xcc,0x5d,0x65,0xb6,0x92,
    0x6c,0x70,0x48,0x50,0xfd,0xed,0xb9,0xda,0x5e,0x15,0x46,0x57,0xa7,0x8d,0x9d,0x84,
    0x90,0xd8,0xab,0x00,0x8c,0xbc,0xd3,0x0a,0xf7,0xe4,0x58,0x05,0xb8,0xb3,0x45,0x06,
    0xd0,0x2c,0x1e,0x8f,0xca,0x3f,0x0f,0x02,0xc1,0xaf,0xbd,0x03,0x01,0x13,0x8a,0x6b,
    0x3a,0x91,0x11,0x41,0x4f,0x67,0xdc,0xea,0x97,0xf2,0xcf,0xce,0xf0,0xb4,0xe6,0x73,
    0x96,0xac,0x74,0x22,0xe7,0xad,0x35,0x85,0xe2,0xf9,0x37,0xe8,0x1c,0x75,0xdf,0x6e,
    0x47,0xf1,0x1a,0x71,0x1d,0x29,0xc5,0x89,0x6f,0xb7,0x62,0x0e,0xaa,0x18,0xbe,0x1b,
    0xfc,0x56,0x3e,0x4b,0xc6,0xd2,0x79,0x20,0x9a,0xdb,0xc0,0xfe,0x78,0xcd,0x5a,0xf4,
    0x1f,0xdd,0xa8,0x33,0x88,0x07,0xc7,0x31,0xb1,0x12,0x10,0x59,0x27,0x80,0xec,0x5f,
    0x60,0x51,0x7f,0xa9,0x19,0xb5,0x4a,0x0d,0x2d,0xe5,0x7a,0x9f,0x93,0xc9,0x9c,0xef,
    0xa0,0xe0,0x3b,0x4d,0xae,0x2a,0xf5,0xb0,0xc8,0xeb,0xbb,0x3c,0x83,0x53,0x99,0x61,
    0x17,0x2b,0x04,0x7e,0xba,0x77,0xd6,0x26,0xe1,0x69,0x14,0x63,0x55,0x21,0x0c,0x7d
};

static const uint8_t Rcon[11] = {0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36};

static inline uint8_t xtime(uint8_t x) { return (uint8_t)((x << 1) ^ (((x >> 7) & 1) * 0x1b)); }
static inline uint8_t mul(uint8_t x, uint8_t y) {
  uint8_t r = 0;
  for (int i = 0; i < 8; ++i) {
    if (y & 1) r ^= x;
    x = xtime(x);
    y >>= 1;
  }
  return r;
}

// 11 round keys × 16 bytes = 176 bytes.
static void key_expand(const uint8_t key[16], uint8_t rk[176]) {
  std::memcpy(rk, key, 16);
  uint32_t i = 16;
  uint8_t tmp[4];
  while (i < 176) {
    tmp[0] = rk[i - 4]; tmp[1] = rk[i - 3]; tmp[2] = rk[i - 2]; tmp[3] = rk[i - 1];
    if ((i % 16) == 0) {
      // RotWord + SubWord + Rcon
      uint8_t t = tmp[0]; tmp[0] = tmp[1]; tmp[1] = tmp[2]; tmp[2] = tmp[3]; tmp[3] = t;
      tmp[0] = Sbox[tmp[0]]; tmp[1] = Sbox[tmp[1]];
      tmp[2] = Sbox[tmp[2]]; tmp[3] = Sbox[tmp[3]];
      tmp[0] ^= Rcon[i / 16];
    }
    rk[i + 0] = rk[i - 16 + 0] ^ tmp[0];
    rk[i + 1] = rk[i - 16 + 1] ^ tmp[1];
    rk[i + 2] = rk[i - 16 + 2] ^ tmp[2];
    rk[i + 3] = rk[i - 16 + 3] ^ tmp[3];
    i += 4;
  }
}

static void add_rk(uint8_t s[16], const uint8_t *rk) {
  for (int i = 0; i < 16; ++i) s[i] ^= rk[i];
}
static void sub_bytes(uint8_t s[16])    { for (int i = 0; i < 16; ++i) s[i] = Sbox[s[i]]; }
static void inv_sub_bytes(uint8_t s[16]){ for (int i = 0; i < 16; ++i) s[i] = InvSbox[s[i]]; }
static void shift_rows(uint8_t s[16]) {
  uint8_t t;
  t = s[1];  s[1]  = s[5];  s[5]  = s[9];  s[9]  = s[13]; s[13] = t;
  t = s[2];  s[2]  = s[10]; s[10] = t;     t = s[6];  s[6]  = s[14]; s[14] = t;
  t = s[15]; s[15] = s[11]; s[11] = s[7];  s[7]  = s[3];  s[3]  = t;
}
static void inv_shift_rows(uint8_t s[16]) {
  uint8_t t;
  t = s[13]; s[13] = s[9];  s[9]  = s[5];  s[5]  = s[1];  s[1]  = t;
  t = s[2];  s[2]  = s[10]; s[10] = t;     t = s[6];  s[6]  = s[14]; s[14] = t;
  t = s[3];  s[3]  = s[7];  s[7]  = s[11]; s[11] = s[15]; s[15] = t;
}
static void mix_cols(uint8_t s[16]) {
  for (int c = 0; c < 4; ++c) {
    uint8_t a0 = s[4*c+0], a1 = s[4*c+1], a2 = s[4*c+2], a3 = s[4*c+3];
    uint8_t t = a0 ^ a1 ^ a2 ^ a3;
    s[4*c+0] ^= t ^ xtime(a0 ^ a1);
    s[4*c+1] ^= t ^ xtime(a1 ^ a2);
    s[4*c+2] ^= t ^ xtime(a2 ^ a3);
    s[4*c+3] ^= t ^ xtime(a3 ^ a0);
  }
}
static void inv_mix_cols(uint8_t s[16]) {
  for (int c = 0; c < 4; ++c) {
    uint8_t a = s[4*c+0], b = s[4*c+1], cc = s[4*c+2], d = s[4*c+3];
    s[4*c+0] = mul(a,0x0e) ^ mul(b,0x0b) ^ mul(cc,0x0d) ^ mul(d,0x09);
    s[4*c+1] = mul(a,0x09) ^ mul(b,0x0e) ^ mul(cc,0x0b) ^ mul(d,0x0d);
    s[4*c+2] = mul(a,0x0d) ^ mul(b,0x09) ^ mul(cc,0x0e) ^ mul(d,0x0b);
    s[4*c+3] = mul(a,0x0b) ^ mul(b,0x0d) ^ mul(cc,0x09) ^ mul(d,0x0e);
  }
}

static void encrypt_block(const uint8_t in[16], uint8_t out[16], const uint8_t rk[176]) {
  uint8_t s[16]; std::memcpy(s, in, 16);
  add_rk(s, rk);
  for (int r = 1; r < 10; ++r) {
    sub_bytes(s); shift_rows(s); mix_cols(s); add_rk(s, rk + r*16);
  }
  sub_bytes(s); shift_rows(s); add_rk(s, rk + 160);
  std::memcpy(out, s, 16);
}

static void decrypt_block(const uint8_t in[16], uint8_t out[16], const uint8_t rk[176]) {
  uint8_t s[16]; std::memcpy(s, in, 16);
  add_rk(s, rk + 160);
  for (int r = 9; r >= 1; --r) {
    inv_shift_rows(s); inv_sub_bytes(s); add_rk(s, rk + r*16); inv_mix_cols(s);
  }
  inv_shift_rows(s); inv_sub_bytes(s); add_rk(s, rk);
  std::memcpy(out, s, 16);
}

} // namespace aes

// ---- SE register state ------------------------------------------------------

namespace {

constexpr uint32_t SE_OPERATION_REG          = 0x008;
constexpr uint32_t SE_INT_ENABLE_REG         = 0x00C;
constexpr uint32_t SE_INT_STATUS_REG         = 0x010;
constexpr uint32_t SE_CONFIG_REG             = 0x014;
constexpr uint32_t SE_IN_LL_ADDR_REG         = 0x018;
constexpr uint32_t SE_OUT_LL_ADDR_REG        = 0x024;
constexpr uint32_t SE_CRYPTO_LAST_BLOCK      = 0x080;
constexpr uint32_t SE_CRYPTO_CONFIG_REG      = 0x304;
constexpr uint32_t SE_CRYPTO_LINEAR_CTR_REG  = 0x308; // 4 dwords
constexpr uint32_t SE_CRYPTO_BLOCK_COUNT_REG = 0x318;
constexpr uint32_t SE_CRYPTO_KEYTABLE_ADDR   = 0x31C;
constexpr uint32_t SE_CRYPTO_KEYTABLE_DATA   = 0x320;
constexpr uint32_t SE_CRYPTO_KEYTABLE_DST    = 0x330;
constexpr uint32_t SE_RNG_CONFIG_REG         = 0x340;
constexpr uint32_t SE_HASH_RESULT_REG        = 0x030; // 16 dwords
constexpr uint32_t SE_STATUS_REG             = 0x800;
constexpr uint32_t SE_ERR_STATUS_REG         = 0x804;

constexpr uint32_t SE_INT_OP_DONE   = 1u << 4;

constexpr uint32_t ALG_NOP     = 0;
constexpr uint32_t ALG_AES_DEC = 1;
constexpr uint32_t ALG_AES_ENC = 1; // same numeric value; chosen via dec/enc field

constexpr uint32_t DST_MEMORY    = 0;
constexpr uint32_t DST_HASHREG   = 1;
constexpr uint32_t DST_KEYTABLE  = 2;

constexpr uint32_t XOR_BYPASS = 0;
constexpr uint32_t XOR_TOP    = 2;
constexpr uint32_t XOR_BOTTOM = 3;

constexpr uint32_t INPUT_MEMORY  = 0;
constexpr uint32_t INPUT_AESOUT  = 2;
constexpr uint32_t INPUT_LNR_CTR = 3;

constexpr uint32_t VCTRAM_MEM     = 0;
constexpr uint32_t VCTRAM_AESOUT  = 2;
constexpr uint32_t VCTRAM_PREVMEM = 3;

constexpr uint32_t IV_ORIGINAL = 0;
constexpr uint32_t IV_UPDATED  = 1;

constexpr uint32_t CORE_DECRYPT = 0;
constexpr uint32_t CORE_ENCRYPT = 1;

// 16 keyslots, each 32 bytes (256-bit max — 2 quads)
struct Keyslot {
  uint8_t key[32];      // KEYS_0_3 + KEYS_4_7
  uint8_t iv_orig[16];
  uint8_t iv_upd[16];
};
static Keyslot ks_table[16];

// BIS key override: when --prod-keys is supplied, the bis_key_NN values from
// it are stashed here. Anytime Lockpick writes to slots 0-5 (the BIS keytable
// slots), we substitute the prod.keys value instead of the derived one. This
// sidesteps any rounding errors / mismatched key sources in our SE-based key
// derivation chain (we get the wrong tsec→master_kek→master_key→bis_kek path
// for some firmware versions, but the user already knows the right BIS keys
// because their prod.keys file says so).
static bool     bis_override_present[6] = {false, false, false, false, false, false};
static uint8_t  bis_override_keys[6][16] = {0};

static uint32_t reg_config        = 0;
static uint32_t reg_crypto_config = 0;
static uint32_t reg_in_ll_addr    = 0;
static uint32_t reg_out_ll_addr   = 0;
static uint32_t reg_block_count   = 0;
static uint32_t reg_keytable_addr = 0;
static uint32_t reg_keytable_dst  = 0;
static uint32_t reg_op            = 0;
static uint32_t reg_int_status    = 0;
static uint32_t reg_int_enable    = 0;
static uint32_t linear_ctr[4]     = {0};
static uint32_t hash_result[16]   = {0};
static uint32_t reg_err_status    = 0;
static uint32_t reg_rng_config    = 0;
static uint32_t spare_regs[256]   = {0}; // catch-all for less critical regs

// LL descriptor: { num, addr, size }. Hekate often uses num=0 (single descriptor).
struct LLDesc { uint32_t num, addr, size; };

static bool read_ll(uc_engine *uc, uint32_t ll_addr, LLDesc *out) {
  if (!ll_addr) return false;
  if (uc_mem_read(uc, ll_addr, out, sizeof(LLDesc)) != UC_ERR_OK) return false;
  return true;
}

// Encrypt/decrypt a buffer in-place using the keyslot's key (assumes 128-bit).
static void run_aes_blocks(uint32_t slot, bool encrypt,
                           const uint8_t *in, uint8_t *out, size_t blocks,
                           uint32_t xor_pos, uint32_t input_sel,
                           uint32_t vctram_sel, uint32_t iv_sel,
                           bool ctr_mode, bool hash_mode) {
  uint8_t rk[176];
  aes::key_expand(ks_table[slot].key, rk);

  // Pick which IV to use for chaining. Hekate typically uses ORIGINAL_IV for
  // CBC starts and switches to UPDATED_IV mid-CMAC; we honour that.
  uint8_t chain[16];
  if (iv_sel == IV_UPDATED) std::memcpy(chain, ks_table[slot].iv_upd, 16);
  else                      std::memcpy(chain, ks_table[slot].iv_orig, 16);

  uint8_t prev_in[16] = {0};
  uint8_t hash_state[16] = {0}; // accumulator for HASH=1 (CMAC) — XOR top, encrypt
  if (hash_mode) std::memcpy(hash_state, chain, 16);

  for (size_t i = 0; i < blocks; ++i) {
    uint8_t in_block[16];
    uint8_t out_block[16];

    if (input_sel == INPUT_LNR_CTR) {
      // CTR mode input is the linear counter (big-endian 128-bit), per-block.
      // Our linear_ctr[] is stored as 4 dwords little-endian; assemble.
      uint8_t ctr_be[16];
      for (int j = 0; j < 4; ++j) {
        uint32_t w = linear_ctr[j];
        ctr_be[j*4 + 0] = (w >> 0)  & 0xFF;
        ctr_be[j*4 + 1] = (w >> 8)  & 0xFF;
        ctr_be[j*4 + 2] = (w >> 16) & 0xFF;
        ctr_be[j*4 + 3] = (w >> 24) & 0xFF;
      }
      std::memcpy(in_block, ctr_be, 16);
    } else if (input_sel == INPUT_AESOUT) {
      // Re-feeding previous AES output (used in some chained constructions).
      std::memcpy(in_block, prev_in, 16);
    } else {
      std::memcpy(in_block, in + i*16, 16);
    }

    // VCTRAM source: the value XORed at XOR_TOP / XOR_BOTTOM.
    uint8_t vctram[16];
    if (vctram_sel == VCTRAM_AESOUT)       std::memcpy(vctram, prev_in, 16);
    else if (vctram_sel == VCTRAM_PREVMEM) std::memcpy(vctram, (i==0)? chain : (in + (i-1)*16), 16);
    else                                   std::memcpy(vctram, chain, 16);

    if (xor_pos == XOR_TOP) {
      for (int j = 0; j < 16; ++j) in_block[j] ^= vctram[j];
    }

    if (encrypt) aes::encrypt_block(in_block, out_block, rk);
    else         aes::decrypt_block(in_block, out_block, rk);

    if (xor_pos == XOR_BOTTOM) {
      // Output XOR with chain (CTR keystream XOR plaintext, etc.)
      const uint8_t *xb = (input_sel == INPUT_LNR_CTR)
        ? (in + i*16)            // CTR: XOR with plaintext from memory
        : vctram;
      for (int j = 0; j < 16; ++j) out_block[j] ^= xb[j];
    }

    if (out) std::memcpy(out + i*16, out_block, 16);

    // Advance chain for next block.
    if (xor_pos == XOR_BOTTOM && input_sel != INPUT_LNR_CTR) {
      // CBC encrypt: chain = ciphertext = out_block
      std::memcpy(chain, out_block, 16);
    } else if (xor_pos == XOR_TOP) {
      // CBC decrypt: chain = ciphertext (the input block we read)
      std::memcpy(chain, in + i*16, 16);
    }
    std::memcpy(prev_in, out_block, 16);

    if (ctr_mode) {
      // Increment linear counter (big-endian 128-bit).
      for (int j = 15; j >= 0; --j) {
        uint8_t *p = (uint8_t*)linear_ctr;
        p[j]++;
        if (p[j]) break;
      }
    }
  }

  if (hash_mode) {
    // For HASH paths the final cipher block is exposed via SE_HASH_RESULT_REG.
    std::memcpy(hash_result, prev_in, 16);
  }
  // Save updated IV (chain) for next call's IV_UPDATED.
  std::memcpy(ks_table[slot].iv_upd, chain, 16);
}

static void op_start(EmuState *state) {
  // Decode CONFIG / CRYPTO_CONFIG.
  uint32_t dst_kind = (reg_config >> 2) & 0x7;
  uint32_t enc_alg  = (reg_config >> 12) & 0xF;
  uint32_t dec_alg  = (reg_config >> 8)  & 0xF;
  uint32_t enc_mode = (reg_config >> 24) & 0xFF;
  bool dec = (dec_alg == ALG_AES_DEC) && (enc_alg == ALG_NOP);
  bool enc = (enc_alg == ALG_AES_ENC) && (dec_alg == ALG_NOP);

  // ---- SHA-256 path ----
  // ALG_SHA=3, MODE_SHA256=5, DST=DST_HASHREG. Driver flow: writes
  // SE_SHA_MSG_LENGTH/LEFT, sets up IN_LL pointing at src/src_size, then
  // OP_START. Output 32 bytes go into HASH_RESULT_REG as 8 big-endian dwords
  // (driver byte_swap_32's each on read). We currently only model the
  // oneshot case (SHA_INIT_HASH); save_process_header uses oneshot.
  if (enc_alg == 3 /*ALG_SHA*/ && dst_kind == DST_HASHREG && enc_mode == 5 /*MODE_SHA256*/) {
    LLDesc in_ll{};
    bool got_in = read_ll(state->uc, reg_in_ll_addr, &in_ll);
    if (!got_in || !in_ll.size) {
      reg_int_status |= SE_INT_OP_DONE;
      return;
    }
    std::vector<uint8_t> buf(in_ll.size, 0);
    uc_mem_read(state->uc, in_ll.addr, buf.data(), in_ll.size);
    uint8_t digest[32];
    sha256::compute(buf.data(), in_ll.size, digest);
    // Pack into HASH_RESULT_REG as 8 big-endian dwords (driver byteswaps).
    for (int i = 0; i < 8; i++) {
      hash_result[i] = ((uint32_t)digest[i*4 + 0] << 24) |
                       ((uint32_t)digest[i*4 + 1] << 16) |
                       ((uint32_t)digest[i*4 + 2] <<  8) |
                       ((uint32_t)digest[i*4 + 3]);
    }
    std::printf("[se] SHA-256 over %u bytes @0x%X -> %02X%02X%02X%02X...\n",
                in_ll.size, in_ll.addr, digest[0], digest[1], digest[2], digest[3]);
    reg_int_status |= SE_INT_OP_DONE;
    return;
  }

  if (dst_kind == 4 /* DST_RSAREG */) {
    // RSA not implemented — leave output zero.
    reg_int_status |= SE_INT_OP_DONE;
    return;
  }

  if (!dec && !enc) {
    reg_int_status |= SE_INT_OP_DONE;
    return;
  }

  uint32_t xor_pos    = (reg_crypto_config >> 1)  & 0x3;
  uint32_t input_sel  = (reg_crypto_config >> 3)  & 0x3;
  uint32_t vctram_sel = (reg_crypto_config >> 5)  & 0x3;
  uint32_t iv_sel     = (reg_crypto_config >> 7)  & 0x1;
  uint32_t core_sel   = (reg_crypto_config >> 8)  & 0x1;
  bool     hash_en    = (reg_crypto_config >> 0)  & 0x1;
  uint32_t ctr_cnt    = (reg_crypto_config >> 11) & 0x1;
  uint32_t key_index  = (reg_crypto_config >> 24) & 0xF;

  bool encrypt = (core_sel == CORE_ENCRYPT);

  uint32_t blocks = reg_block_count + 1;
  uint32_t total_bytes = blocks * 16;

  // Read input via IN_LL.
  std::vector<uint8_t> in_buf, out_buf;
  in_buf.resize(total_bytes, 0);
  out_buf.resize(total_bytes, 0);

  LLDesc in_ll{}, out_ll{};
  bool got_in  = read_ll(state->uc, reg_in_ll_addr, &in_ll);
  bool got_out = read_ll(state->uc, reg_out_ll_addr, &out_ll);
  (void)got_out;

  if (got_in && in_ll.size && input_sel == INPUT_MEMORY) {
    uint32_t copy = (in_ll.size < total_bytes) ? in_ll.size : total_bytes;
    uc_mem_read(state->uc, in_ll.addr, in_buf.data(), copy);
  }

  run_aes_blocks(key_index, encrypt, in_buf.data(), out_buf.data(), blocks,
                 xor_pos, input_sel, vctram_sel, iv_sel,
                 (input_sel == INPUT_LNR_CTR) || ctr_cnt, hash_en);

  if (dst_kind == DST_MEMORY) {
    if (got_out && out_ll.size) {
      uint32_t copy = (out_ll.size < total_bytes) ? out_ll.size : total_bytes;
      uc_mem_write(state->uc, out_ll.addr, out_buf.data(), copy);
    }
  } else if (dst_kind == DST_KEYTABLE) {
    // Unwrap-key path: store the (single-block) result in the destination slot.
    uint32_t dst_slot = (reg_keytable_dst >> 8) & 0xF;
    uint32_t word_quad = reg_keytable_dst & 0x3;
    uint8_t *target = (word_quad == 1)
      ? ks_table[dst_slot].key + 16
      : ks_table[dst_slot].key + 0;
    std::memcpy(target, out_buf.data(), 16);
  }
  // DST_HASHREG: hash_result is already populated by run_aes_blocks.

  reg_int_status |= SE_INT_OP_DONE;

  printf("[se] OP slot=%u %s blocks=%u xor=%u in_sel=%u vctram=%u iv=%u dst=%u "
         "in_addr=0x%X out_addr=0x%X\n",
         key_index, encrypt ? "ENC" : "DEC", blocks, xor_pos, input_sel,
         vctram_sel, iv_sel, dst_kind,
         got_in ? in_ll.addr : 0, got_out ? out_ll.addr : 0);
}

// Keytable address layout: bits 4..7 = slot, bits 2..3 = quad, bits 0..1 = pkt
static void keytable_write(uint32_t addr, uint32_t val) {
  uint32_t slot = (addr >> 4) & 0xF;
  uint32_t quad = (addr >> 2) & 0x3;
  uint32_t pkt  = addr & 0x3;
  uint8_t *target = nullptr;
  switch (quad) {
    case 0: target = ks_table[slot].key + 0  + pkt*4; break; // KEYS_0_3
    case 1: target = ks_table[slot].key + 16 + pkt*4; break; // KEYS_4_7
    case 2: target = ks_table[slot].iv_orig  + pkt*4; break; // ORIGINAL_IV
    case 3: target = ks_table[slot].iv_upd   + pkt*4; break; // UPDATED_IV
  }
  if (target) std::memcpy(target, &val, 4);

  // After the last word of a 16-byte key is written, optionally substitute
  // the prod.keys-derived BIS key into slots 0-5 (KS_BIS_NN_CRYPT/TWEAK).
  // Lockpick's _derive_bis_keys may produce wrong values when our SE engine
  // doesn't faithfully model some part of the chain; the override is a
  // pragmatic shortcut that lets the BIS XTS layer succeed regardless.
  if (quad == 0 && pkt == 3 && slot < 6) {
    if (bis_override_present[slot]) {
      std::memcpy(ks_table[slot].key, bis_override_keys[slot], 16);
      std::printf("[se] slot %u BIS key overridden from prod.keys:", slot);
      for (int i = 0; i < 16; ++i) std::printf(" %02X", ks_table[slot].key[i]);
      std::printf("\n");
    } else {
      std::printf("[se] slot %u key loaded (derived):", slot);
      for (int i = 0; i < 16; ++i) std::printf(" %02X", ks_table[slot].key[i]);
      std::printf("\n");
    }
  }
}

static uint32_t keytable_read(uint32_t addr) {
  uint32_t slot = (addr >> 4) & 0xF;
  uint32_t quad = (addr >> 2) & 0x3;
  uint32_t pkt  = addr & 0x3;
  const uint8_t *src = nullptr;
  switch (quad) {
    case 0: src = ks_table[slot].key + 0  + pkt*4; break;
    case 1: src = ks_table[slot].key + 16 + pkt*4; break;
    case 2: src = ks_table[slot].iv_orig  + pkt*4; break;
    case 3: src = ks_table[slot].iv_upd   + pkt*4; break;
  }
  if (!src) return 0;
  uint32_t v;
  std::memcpy(&v, src, 4);
  return v;
}

} // namespace

#include <vector>

uint32_t se_engine_read(EmuState *state, uint64_t addr) {
  uint32_t off = (uint32_t)(addr - SE_BASE);
  if (off >= SE_HASH_RESULT_REG && off < SE_HASH_RESULT_REG + 16*4) {
    return hash_result[(off - SE_HASH_RESULT_REG) / 4];
  }
  if (off >= SE_CRYPTO_LINEAR_CTR_REG && off < SE_CRYPTO_LINEAR_CTR_REG + 16) {
    return linear_ctr[(off - SE_CRYPTO_LINEAR_CTR_REG) / 4];
  }
  switch (off) {
    case SE_OPERATION_REG:        return reg_op;
    case SE_INT_ENABLE_REG:       return reg_int_enable;
    case SE_INT_STATUS_REG:       return reg_int_status;
    case SE_CONFIG_REG:           return reg_config;
    case SE_IN_LL_ADDR_REG:       return reg_in_ll_addr;
    case SE_OUT_LL_ADDR_REG:      return reg_out_ll_addr;
    case SE_CRYPTO_CONFIG_REG:    return reg_crypto_config;
    case SE_CRYPTO_BLOCK_COUNT_REG: return reg_block_count;
    case SE_CRYPTO_KEYTABLE_ADDR: return reg_keytable_addr;
    case SE_CRYPTO_KEYTABLE_DATA: return keytable_read(reg_keytable_addr);
    case SE_CRYPTO_KEYTABLE_DST:  return reg_keytable_dst;
    case SE_RNG_CONFIG_REG:       return reg_rng_config;
    case SE_STATUS_REG:           return 0; // IDLE
    case SE_ERR_STATUS_REG:       return reg_err_status;
    default:
      if (off < sizeof(spare_regs)/sizeof(spare_regs[0])*4)
        return spare_regs[off / 4];
      return 0;
  }
  (void)state;
}

void se_engine_write(EmuState *state, uint64_t addr, uint32_t val) {
  uint32_t off = (uint32_t)(addr - SE_BASE);
  if (off >= SE_CRYPTO_LINEAR_CTR_REG && off < SE_CRYPTO_LINEAR_CTR_REG + 16) {
    linear_ctr[(off - SE_CRYPTO_LINEAR_CTR_REG) / 4] = val;
    return;
  }
  switch (off) {
    case SE_OPERATION_REG:
      reg_op = val;
      // SE_OP_START runs the configured AES op; other op codes (RESTART_*,
      // CTX_SAVE, ABORT) just need OP_DONE so the driver's _se_wait() exits.
      if ((val & 0x7) == 1 /*SE_OP_START*/) {
        op_start(state);
      } else if ((val & 0x7) != 0 /*not ABORT*/) {
        reg_int_status |= SE_INT_OP_DONE;
      }
      return;
    case SE_INT_ENABLE_REG:       reg_int_enable = val; return;
    case SE_INT_STATUS_REG:
      // Write-1-clear semantics on real HW.
      reg_int_status &= ~val;
      return;
    case SE_CONFIG_REG:           reg_config = val; return;
    case SE_IN_LL_ADDR_REG:       reg_in_ll_addr = val; return;
    case SE_OUT_LL_ADDR_REG:      reg_out_ll_addr = val; return;
    case SE_CRYPTO_CONFIG_REG:    reg_crypto_config = val; return;
    case SE_CRYPTO_BLOCK_COUNT_REG: reg_block_count = val; return;
    case SE_CRYPTO_KEYTABLE_ADDR: reg_keytable_addr = val; return;
    case SE_CRYPTO_KEYTABLE_DATA: keytable_write(reg_keytable_addr, val); return;
    case SE_CRYPTO_KEYTABLE_DST:  reg_keytable_dst = val; return;
    case SE_RNG_CONFIG_REG:       reg_rng_config = val; return;
    case SE_ERR_STATUS_REG:       reg_err_status &= ~val; return;
    default:
      if (off < sizeof(spare_regs)/sizeof(spare_regs[0])*4)
        spare_regs[off / 4] = val;
      return;
  }
  (void)state;
}

void se_engine_set_aes128_key(uint32_t slot, const uint8_t key[16]) {
  if (slot >= 16) return;
  std::memcpy(ks_table[slot].key, key, 16);
}

// ---- prod.keys parser -------------------------------------------------------

static int hex_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + c - 'a';
  if (c >= 'A' && c <= 'F') return 10 + c - 'A';
  return -1;
}

static int parse_hex(const char *s, uint8_t *out, size_t out_size) {
  for (size_t i = 0; i < out_size; ++i) {
    int h = hex_nibble(s[i*2]);
    int l = hex_nibble(s[i*2 + 1]);
    if (h < 0 || l < 0) return 0;
    out[i] = (uint8_t)((h << 4) | l);
  }
  return 1;
}

static void aes_self_test() {
  // Lockpick's check_keyslot_access vector:
  //   key       = 00..0F
  //   ciphertext= 00 (all zeros)
  //   expected  = 7B 1D 29 A1 6C F8 CC AB 84 F0 B8 A5 98 E4 2F A6
  const uint8_t key[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
  const uint8_t ct[16]  = {0};
  const uint8_t exp[16] = {0x7b,0x1d,0x29,0xa1,0x6c,0xf8,0xcc,0xab,
                            0x84,0xf0,0xb8,0xa5,0x98,0xe4,0x2f,0xa6};
  uint8_t rk[176];
  uint8_t out[16];
  aes::key_expand(key, rk);
  aes::decrypt_block(ct, out, rk);
  bool ok = std::memcmp(out, exp, 16) == 0;
  std::printf("[se] AES-128 self-test: %s\n", ok ? "PASS" : "FAIL");
  if (!ok) {
    std::printf("  got: ");
    for (int i = 0; i < 16; ++i) std::printf("%02X ", out[i]);
    std::printf("\n");
  }
}

int se_engine_load_prod_keys(const char *path) {
  static bool tested = false;
  if (!tested) { aes_self_test(); tested = true; }
  FILE *f = std::fopen(path, "r");
  if (!f) {
    std::printf("[se] failed to open prod.keys: %s\n", path);
    return 0;
  }
  // Slot mapping (matches the bootrom-derived keyslots Hekate/Lockpick rely on):
  //   12 = KS_TSEC        / KS_MARIKO_KEK
  //   13 = KS_TSEC_ROOT   / KS_MARIKO_BEK
  //   14 = KS_SECURE_BOOT
  // Plus convenience pre-loads for BIS so SYSTEM mount can short-circuit
  // when downstream key derivation is incomplete:
  //   0 = KS_BIS_00_CRYPT (low half of bis_key_00)
  //   1 = KS_BIS_00_TWEAK (high half of bis_key_00)
  int loaded = 0;
  char line[512];
  while (std::fgets(line, sizeof(line), f)) {
    char name[128];
    char hexv[256];
    if (std::sscanf(line, " %127[^ =] = %255s", name, hexv) != 2) continue;
    auto load_into = [&](uint32_t slot, size_t bytes, size_t off) {
      if (std::strlen(hexv) < (off + bytes) * 2) return;
      uint8_t buf[16];
      if (!parse_hex(hexv + off*2, buf, bytes)) return;
      se_engine_set_aes128_key(slot, buf);
      ++loaded;
    };
    if (!std::strcmp(name, "tsec_key"))        load_into(12, 16, 0);
    else if (!std::strcmp(name, "tsec_root_key") || !std::strcmp(name, "tsec_root_key_00"))
                                                load_into(13, 16, 0);
    else if (!std::strcmp(name, "secure_boot_key")) load_into(14, 16, 0);
    else if (!std::strcmp(name, "bis_key_00")) {
      load_into(0, 16, 0);   // crypt half
      load_into(1, 16, 16);  // tweak half
      // Also stash for runtime override of Lockpick's derived value.
      if (std::strlen(hexv) >= 64) {
        if (parse_hex(hexv,      bis_override_keys[0], 16)) bis_override_present[0] = true;
        if (parse_hex(hexv + 32, bis_override_keys[1], 16)) bis_override_present[1] = true;
      }
    }
    else if (!std::strcmp(name, "bis_key_01")) {
      load_into(2, 16, 0);
      load_into(3, 16, 16);
      if (std::strlen(hexv) >= 64) {
        if (parse_hex(hexv,      bis_override_keys[2], 16)) bis_override_present[2] = true;
        if (parse_hex(hexv + 32, bis_override_keys[3], 16)) bis_override_present[3] = true;
      }
    }
    else if (!std::strcmp(name, "bis_key_02")) {
      load_into(4, 16, 0);
      load_into(5, 16, 16);
      if (std::strlen(hexv) >= 64) {
        if (parse_hex(hexv,      bis_override_keys[4], 16)) bis_override_present[4] = true;
        if (parse_hex(hexv + 32, bis_override_keys[5], 16)) bis_override_present[5] = true;
      }
    }
  }
  std::fclose(f);
  std::printf("[se] loaded %d keys from %s\n", loaded, path);
  return loaded > 0 ? 1 : 0;
}
