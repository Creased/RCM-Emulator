#ifndef T210_SE_ENGINE_H
#define T210_SE_ENGINE_H

#include <cstdint>
#include <cstddef>

struct EmuState;

// Tegra X1 Security Engine (SE) — software emulation that runs real AES-128
// operations against an emulated 16-slot keytable.
//
// Currently implemented:
//   * AES-128 ECB encrypt/decrypt (single + multi-block, DMA via LL descriptors)
//   * AES-128 CBC encrypt/decrypt
//   * AES-128 CTR mode
//   * AES "unwrap" (decrypt and store result in destination keyslot)
//   * Keyslot programming (CRYPTO_KEYTABLE_ADDR/DATA)
//   * Pre-population of keyslots from a Switch prod.keys file
//
// Not implemented (returns zero / no-op):
//   * AES-CMAC (HASH+VCTRAM_AESOUT path)
//   * SHA / RNG / RSA
// Each unimplemented op still completes (sets OP_DONE) so the boot doesn't hang.

uint32_t se_engine_read (EmuState *state, uint64_t addr);
void     se_engine_write(EmuState *state, uint64_t addr, uint32_t val);

// Load 16-byte AES key into the given slot's KEYS_0_3 quad. For 256-bit keys
// (BIS), call with `slot+0` for the low half and `slot+1`-style write into
// the second quad — but for our purposes the bdk drivers always use 128-bit
// AES, so this is enough.
void se_engine_set_aes128_key(uint32_t slot, const uint8_t key[16]);

// Parse a switch-style "prod.keys" file (lines of `name = hex_value`) and
// pre-populate the well-known bootrom-derived keyslots (KS_TSEC=12,
// KS_SECURE_BOOT=14, KS_TSEC_ROOT=13) so Lockpick / Hekate's derivation
// chain bootstraps without TSEC firmware emulation. Returns 1 on success, 0
// on any parse / file error.
int  se_engine_load_prod_keys(const char *path);

#endif
