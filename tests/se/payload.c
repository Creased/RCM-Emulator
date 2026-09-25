/*
 * Security Engine regression payload.
 *
 * A freestanding BPMP payload that drives the emulator's SE model with the
 * register sequences bdk uses, checks every result against vectors computed
 * outside the emulator (vectors.h, from gen_vectors.py), and reports on
 * UART-B. tests/se/run.sh checks the log.
 *
 *   rsa   se_rsa_key_set() / se_rsa_exp_mod() as Lockpick's bdk has them:
 *         a 2048-bit and a 1024-bit key in the two slots, each signing with
 *         its private exponent and verifying with e = 65537
 *   sha   SHA-256 one-shot, and in two parts (INIT, then CONTINUE)
 *   aes   AES-128 ECB, Lockpick's keyslot-access vector
 *   rng   se_rng_pseudo(): output that is not all zeros, differs from call
 *         to call, and stops at the size asked for
 */

typedef unsigned int u32;
typedef unsigned char u8;

#define REG(a) (*(volatile u32 *)(a))

#define UARTB_THR 0x70006040u
#define I2C5      0x7000D000u
#define SE        0x70012000u
#define SER(o)    REG(SE + (o))

#define SE_OPERATION      0x008
#define SE_INT_STATUS     0x010
#define SE_CONFIG         0x014
#define SE_IN_LL_ADDR     0x018
#define SE_OUT_LL_ADDR    0x024
#define SE_HASH_RESULT    0x030
#define SE_SHA_CONFIG     0x200
#define SE_SHA_MSG_LENGTH 0x204
#define SE_SHA_MSG_LEFT   0x214
#define SE_CRYPTO_CONFIG  0x304
#define SE_LAST_BLOCK     0x318
#define SE_KEYTABLE_ADDR  0x31C
#define SE_KEYTABLE_DATA  0x320
#define SE_RNG_CONFIG     0x340
#define SE_RSA_CONFIG     0x400
#define SE_RSA_KEY_SIZE   0x404
#define SE_RSA_EXP_SIZE   0x408
#define SE_RSA_KT_ADDR    0x420
#define SE_RSA_KT_DATA    0x424
#define SE_RSA_OUTPUT     0x428
#define SE_ERR_STATUS     0x804

#define OP_DONE  (1u << 4)
#define ERR_STAT (1u << 16)

#define DST(x)      ((x) << 2)
#define DEC_ALG(x)  ((x) << 8)
#define ENC_ALG(x)  ((x) << 12)
#define ENC_MODE(x) ((x) << 24)
#define DST_MEMORY  0
#define DST_HASHREG 1
#define DST_RSAREG  4
#define ALG_AES     1
#define ALG_RNG     2
#define ALG_SHA     3
#define ALG_RSA     4
#define MODE_SHA256 5

#include "vectors.h"

static void puts_(const char *s) {
    while (*s)
        REG(UARTB_THR) = (u32)(unsigned char)*s++;
}

static void i2c5_write(u32 dev, u32 reg, u32 val) {
    REG(I2C5 + 0x04) = dev << 1;              /* CMD_ADDR0 */
    REG(I2C5 + 0x0C) = reg | (val << 8);      /* CMD_DATA1 */
    REG(I2C5 + 0x00) = (1u << 1) | (1u << 9); /* CNFG: 2 bytes, write, GO */
}

static int failed;

static void check(int ok, const char *what) {
    puts_(ok ? "se test: ok    " : "se test: FAIL  ");
    puts_(what);
    puts_("\n");
    if (!ok)
        failed = 1;
}

static int same(const u8 *a, const u8 *b, u32 n) {
    for (u32 i = 0; i < n; i++)
        if (a[i] != b[i])
            return 0;
    return 1;
}

static u32 be32(const u8 *p) {
    return (u32)p[0] << 24 | (u32)p[1] << 16 | (u32)p[2] << 8 | p[3];
}

static void put_be32(u8 *p, u32 v) {
    p[0] = v >> 24;
    p[1] = v >> 16;
    p[2] = v >> 8;
    p[3] = v;
}

/* bdk's _se_execute_oneshot(): one linked-list entry each way. */
static volatile u32 ll_in[3], ll_out[3];

static int se_exec(void *dst, u32 dst_size, const void *src, u32 src_size) {
    ll_in[0] = 0;
    ll_in[1] = (u32)src;
    ll_in[2] = src_size;
    ll_out[0] = 0;
    ll_out[1] = (u32)dst;
    ll_out[2] = dst_size;
    SER(SE_IN_LL_ADDR) = src ? (u32)ll_in : 0;
    SER(SE_OUT_LL_ADDR) = dst ? (u32)ll_out : 0;
    SER(SE_ERR_STATUS) = SER(SE_ERR_STATUS);
    SER(SE_INT_STATUS) = SER(SE_INT_STATUS);
    SER(SE_OPERATION) = 1; /* START */
    while (!(SER(SE_INT_STATUS) & OP_DONE))
        ;
    return (SER(SE_INT_STATUS) & ERR_STAT) || SER(SE_ERR_STATUS);
}

/* se_rsa_key_set(): the big-endian key goes in backwards, word by word. */
static void rsa_key_set(u32 ks, const u8 *mod, u32 mod_size, const u8 *exp,
                        u32 exp_size) {
    for (u32 i = 0; i < mod_size / 4; i++) {
        SER(SE_RSA_KT_ADDR) = ks << 7 | 1u << 6 | i; /* MOD */
        SER(SE_RSA_KT_DATA) = be32(mod + mod_size - 4 - 4 * i);
    }
    for (u32 i = 0; i < exp_size / 4; i++) {
        SER(SE_RSA_KT_ADDR) = ks << 7 | i;           /* EXP */
        SER(SE_RSA_KT_DATA) = be32(exp + exp_size - 4 - 4 * i);
    }
}

/* se_rsa_exp_mod(): the input reversed into a little-endian number, the
 * output read back from SE_RSA_OUTPUT into big-endian bytes. */
static int rsa_exp_mod(u32 ks, u8 *dst, const u8 *src, u32 size,
                       u32 exp_size) {
    static u8 buf[256];
    for (u32 i = 0; i < size; i++)
        buf[i] = src[size - 1 - i];
    SER(SE_CONFIG) = ENC_ALG(ALG_RSA) | DST(DST_RSAREG);
    SER(SE_RSA_CONFIG) = ks << 24;
    SER(SE_RSA_KEY_SIZE) = (size >> 6) - 1;
    SER(SE_RSA_EXP_SIZE) = exp_size >> 2;
    int res = se_exec(0, 0, buf, size);
    for (u32 i = 0; i < size / 4; i++)
        put_be32(dst + size - 4 - 4 * i, SER(SE_RSA_OUTPUT + 4 * i));
    return res;
}

/* _se_sha_hash_256(): total 0 or equal to the part means INIT; a part
 * that is not the last says one byte more is left than it carries. */
static int sha256(u8 *hash, u32 total, const u8 *src, u32 size) {
    u32 left = total < size ? size + 1 : size;
    SER(SE_CONFIG) = ENC_MODE(MODE_SHA256) | ENC_ALG(ALG_SHA) | DST(DST_HASHREG);
    SER(SE_SHA_MSG_LENGTH) = total << 3;
    SER(SE_SHA_MSG_LENGTH + 4) = total >> 29;
    SER(SE_SHA_MSG_LENGTH + 8) = 0;
    SER(SE_SHA_MSG_LENGTH + 12) = 0;
    SER(SE_SHA_MSG_LEFT) = left << 3;
    SER(SE_SHA_MSG_LEFT + 4) = left >> 29;
    SER(SE_SHA_MSG_LEFT + 8) = 0;
    SER(SE_SHA_MSG_LEFT + 12) = 0;
    SER(SE_SHA_CONFIG) = (total == size || !total) ? 1 : 0;
    int res = se_exec(0, 0, src, size);
    for (u32 i = 0; i < 8; i++)
        put_be32(hash + 4 * i, SER(SE_HASH_RESULT + 4 * i));
    return res;
}

/* se_rng_pseudo() for a whole number of blocks. */
static int rng(u8 *dst, u32 size) {
    SER(SE_CONFIG) = ENC_ALG(ALG_RNG) | DST(DST_MEMORY);
    SER(SE_CRYPTO_CONFIG) = 1u << 3; /* INPUT_RANDOM */
    SER(SE_RNG_CONFIG) = 1u << 2;    /* SRC_ENTROPY, MODE_NORMAL */
    SER(SE_LAST_BLOCK) = (size >> 4) - 1;
    return se_exec(dst, size, 0, 0);
}

static const u8 e65537[4] = {0x00, 0x01, 0x00, 0x01};

__attribute__((section(".text.start"), noreturn))
void _start(void) {
    static u8 out[256], hash[32], blk[16], a[48], b[48];

    puts_("se test: start\n");

    /* RSA: both slots loaded before either is used. */
    rsa_key_set(0, k2048_mod, 256, k2048_exp, 256);
    rsa_key_set(1, k1024_mod, 128, k1024_exp, 128);
    check(!rsa_exp_mod(0, out, k2048_msg, 256, 256) &&
          same(out, k2048_sig, 256), "RSA-2048 m^d mod n");
    check(!rsa_exp_mod(1, out, k1024_msg, 128, 128) &&
          same(out, k1024_sig, 128), "RSA-1024 m^d mod n in slot 1");
    rsa_key_set(0, k2048_mod, 256, e65537, 4);
    check(!rsa_exp_mod(0, out, k2048_sig, 256, 4) &&
          same(out, k2048_msg, 256), "RSA-2048 s^e mod n gives m back");
    rsa_key_set(1, k1024_mod, 128, e65537, 4);
    check(!rsa_exp_mod(1, out, k1024_sig, 128, 4) &&
          same(out, k1024_msg, 128), "RSA-1024 s^e mod n gives m back");

    /* SHA-256. */
    static const u8 abc[3] = {'a', 'b', 'c'};
    check(!sha256(hash, 3, abc, 3) && same(hash, sha_abc_digest, 32),
          "SHA-256 one-shot");
    check(!sha256(hash, 0, sha_msg, 128) &&
          !sha256(hash, 200, sha_msg + 128, 72) &&
          same(hash, sha_msg_digest, 32), "SHA-256 in two parts");

    /* AES-128 ECB decrypt of a zero block under 00..0F. */
    static const u8 aes_exp[16] = {0x7B, 0x1D, 0x29, 0xA1, 0x6C, 0xF8,
                                   0xCC, 0xAB, 0x84, 0xF0, 0xB8, 0xA5,
                                   0x98, 0xE4, 0x2F, 0xA6};
    for (u32 i = 0; i < 4; i++) {
        SER(SE_KEYTABLE_ADDR) = 5u << 4 | i; /* slot 5, KEYS_0_3 */
        SER(SE_KEYTABLE_DATA) = (4 * i) | (4 * i + 1) << 8 |
                                (4 * i + 2) << 16 | (u32)(4 * i + 3) << 24;
    }
    for (u32 i = 0; i < 16; i++)
        blk[i] = 0;
    SER(SE_CONFIG) = DEC_ALG(ALG_AES) | DST(DST_MEMORY);
    SER(SE_CRYPTO_CONFIG) = 5u << 24; /* KEY_INDEX 5, CORE_DECRYPT */
    SER(SE_LAST_BLOCK) = 0;
    check(!se_exec(blk, 16, blk, 16) && same(blk, aes_exp, 16),
          "AES-128 ECB decrypt");

    /* RNG: 32 bytes twice, into zeroed buffers with a sentinel tail. */
    for (u32 i = 0; i < 48; i++)
        a[i] = b[i] = i < 32 ? 0 : 0x5A;
    int res = rng(a, 32) | rng(b, 32);
    int zero = 1, tail = 1;
    for (u32 i = 0; i < 32; i++)
        if (a[i])
            zero = 0;
    for (u32 i = 32; i < 48; i++)
        if (a[i] != 0x5A || b[i] != 0x5A)
            tail = 0;
    check(!res && !zero, "RNG fills the buffer");
    check(!same(a, b, 32), "RNG output changes between calls");
    check(tail, "RNG stops at the size asked for");

    puts_(failed ? "se test: done, with failures\n" : "se test: done\n");
    i2c5_write(0x3C, 0x41, 0x02); /* MAX77620 ONOFFCNFG1: PWR_OFF */
    for (;;)
        ;
}
