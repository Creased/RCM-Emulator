#ifndef T210_SE_ENGINE_H
#define T210_SE_ENGINE_H

#include <cstdint>
#include <cstddef>

struct EmuState;

// Tegra X1 Security Engine (SE) — software emulation that runs real AES-128
// operations against an emulated 16-slot keytable.
//
// Currently implemented:
//   * AES-128 ECB, CBC, OFB and CTR, single and multi-block, with the IV
//     chained through UPDATED_IV the way bdk's partial-block path expects
//   * AES-CMAC (HASH_ENABLE into SE_HASH_RESULT)
//   * AES "unwrap" (decrypt and store result in destination keyslot)
//   * SHA-256, one-shot and in parts (SHA_INIT_HASH / SHA_CONTINUE)
//   * Context save of AES keyslots under a secure random key, as bdk's
//     se_aes_ctx_get_keys() reads keys back
//   * Keyslot programming (CRYPTO_KEYTABLE_ADDR/DATA)
//   * Pre-population of keyslots from a Switch prod.keys file
//
// Not implemented: RNG output, RSA, AES-192/256 key sizes. Those operations
// still complete (set OP_DONE) so a payload does not hang on them.

uint32_t se_engine_read (EmuState *state, uint64_t addr);
void     se_engine_write(EmuState *state, uint64_t addr, uint32_t val);

// Load 16-byte AES key into the given slot's KEYS_0_3 quad. For 256-bit keys
// (BIS), call with `slot+0` for the low half and `slot+1`-style write into
// the second quad — but for our purposes the bdk drivers always use 128-bit
// AES, so this is enough.
void se_engine_set_aes128_key(uint32_t slot, const uint8_t key[16]);

// Soft reboot: registers back to reset, keyslots back to what prod.keys
// pre-loaded.
void se_engine_reset();

// Parse a switch-style "prod.keys" file (lines of `name = hex_value`) and
// pre-populate the well-known bootrom-derived keyslots (KS_TSEC=12,
// KS_SECURE_BOOT=14, KS_TSEC_ROOT=13) so Lockpick / Hekate's derivation
// chain bootstraps without TSEC firmware emulation. Returns 1 on success, 0
// on any parse / file error.
int  se_engine_load_prod_keys(const char *path);

#endif
