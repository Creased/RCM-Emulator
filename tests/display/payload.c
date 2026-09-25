/*
 * Display regression payload.
 *
 * A freestanding BPMP payload that drives the emulator's display controller
 * and VIC models the way bdk does - window registers through
 * DISPLAY_WINDOW_HEADER and STATE_CONTROL, VIC through its Falcon register
 * window and a vic_config_t in DRAM - then parks. tests/display/run.sh
 * compares the frame the emulator shows (last_fb.rgba) with the picture the
 * payload drew. Built once per scenario with -DSCENARIO=n:
 *
 *   1  a pitch window (hekate's TUI setup)
 *   2  VIC turns a 1280x720 picture 270 degrees into a pitch surface that
 *      window A shows (Nyx's setup); the view undoes the turn
 *   3  as 2, but VIC writes a block-linear surface (16-GOB blocks)
 *   4  VIC copies the picture into a block-linear surface, and window A
 *      shows it turned by the DC itself: SCAN_COLUMN | H_DIRECTION with
 *      ADDR_H_OFFSET at the top-right pixel's last byte (HOS's setup)
 *   5  as 2, but VIC reads a block-linear source surface
 *   6  window A mirrored top-bottom (V_DIRECTION, ADDR_V_OFFSET at the
 *      last line), with window D blended over it at K1 = 128
 *
 * Every surface holds pat(x, y), which differs between neighbouring
 * pixels in both directions, so a wrong stride, turn, flip or offset shows
 * up almost everywhere.
 */

#ifndef SCENARIO
#define SCENARIO 1
#endif

typedef unsigned int u32;

#define REG(a) (*(volatile u32 *)(a))

#define UARTB_THR 0x70006040u
#define DC        0x54200000u
#define VIC       0x54340000u

#define DCR(i)    REG(DC + (i) * 4u)
/* bdk's _vic_write_priv() for registers with no low address bits. */
#define VICP(r)   REG(VIC + 0x1000u + ((r) >> 6))

#define SRC_L  0xC0000000u /* 1280x720 landscape picture, pitch */
#define SURF_B 0xC0400000u /* block-linear intermediate */
#define DST    0xC0800000u /* what window A scans out */
#define SRC_P  0xC0C00000u /* 720x1280 portrait picture, pitch */
#define SRC_Q  0xC1000000u /* 200x300 overlay for window D */
#define CFG    0xC1400000u /* vic_config_t, 256-byte aligned */

/* DC window registers (word indices) and bits. */
#define WINDOW_HEADER 0x042u
#define STATE_CONTROL 0x041u
#define WIN_OPTIONS   0x700u
#define COLOR_DEPTH   0x703u
#define POSITION      0x704u
#define SIZE          0x705u
#define PRESCALED     0x706u
#define LINE_STRIDE   0x70Au
#define BLEND_LAYER   0x716u
#define BLEND_MATCH   0x717u
#define START_ADDR    0x800u
#define ADDR_H_OFFSET 0x806u
#define ADDR_V_OFFSET 0x808u
#define SURFACE_KIND  0x80Bu

#define H_DIRECTION BIT(0)
#define V_DIRECTION BIT(2)
#define SCAN_COLUMN BIT(4)
#define WIN_ENABLE  BIT(30)
#define BIT(n)      (1u << (n))

#define KIND_PITCH  0u
#define KIND_BLOCK  (2u | (4u << 4)) /* block linear, 16-GOB blocks */

static void puts_(const char *s) {
    while (*s)
        REG(UARTB_THR) = (u32)(unsigned char)*s++;
}

static u32 pat(u32 x, u32 y) {
    return 0xFF000000u | ((x * 7u) & 0xFFu) << 16 | ((y * 3u) & 0xFFu) << 8 |
           ((x ^ y) & 0xFFu);
}

static u32 patq(u32 x, u32 y) {
    return 0xFF000000u | ((x * 5u + y) & 0xFFu) << 16 | 0x4000u |
           ((y * 9u) & 0xFFu);
}

static void fill(u32 base, u32 w, u32 h, u32 (*f)(u32, u32)) {
    volatile u32 *p = (volatile u32 *)base;
    for (u32 y = 0; y < h; y++)
        for (u32 x = 0; x < w; x++)
            p[y * w + x] = f(x, y);
}

/* One u64 bitfield word of vic_config_t, written as two halves. */
static void cfg64(u32 off, u32 lo, u32 hi) {
    REG(CFG + off) = lo;
    REG(CFG + off + 4) = hi;
}

/*
 * One VIC compose of a sw x sh X8R8G8B8 surface (slot 0) into a target of
 * the same size, with bdk's vic_set_surface() field layout. kind: 0 pitch,
 * 1 block linear; gob: log2 GOBs per block.
 */
static void vic_compose(u32 src, u32 skind, u32 sgob, u32 dst, u32 okind,
                        u32 ogob, u32 sw, u32 sh, u32 flip_x, u32 transpose) {
    for (u32 i = 0; i < 0x610; i += 4)
        REG(CFG + i) = 0;
    /* OutputConfig: FlipX b48, Transpose b50; TargetRect R b16, B b48. */
    cfg64(0x10, 0, (flip_x ? BIT(16) : 0) | (transpose ? BIT(18) : 0));
    cfg64(0x18, (sw - 1) << 16, (sh - 1) << 16);
    /* OutputSurfaceConfig: X8R8G8B8 (36), BlkKind b11, BlkHeight b15,
     * width - 1 b32, height - 1 b46. */
    cfg64(0x20, 36u | okind << 11 | ogob << 15, (sw - 1) | (sh - 1) << 14);
    /* Slot 0: enable; SourceRect in 16.16; DestRect. */
    cfg64(0x90, 1, 0);
    cfg64(0xB0, 0, (sw - 1) << 16);
    cfg64(0xB8, 0, (sh - 1) << 16);
    cfg64(0xC0, (sw - 1) << 16, (sh - 1) << 16);
    cfg64(0xD0, 36u | skind << 11 | sgob << 15, (sw - 1) | (sh - 1) << 14);

    VICP(0x14000) = CFG >> 8;  /* VIC_SC_PRAMBASE */
    VICP(0x14300) = src >> 8;  /* VIC_SC_SFC0_BASE_LUMA(0) */
    VICP(0x22000) = dst >> 8;  /* VIC_BL_TARGET_BASADR */
    VICP(0x10000) = 1;         /* VIC_FC_COMPOSE */
}

/* Program window w (0 = A .. 3 = D) and activate it. */
static void window(u32 w, u32 opts, u32 pos, u32 width, u32 height,
                   u32 stride, u32 kind, u32 addr, u32 h_off, u32 v_off,
                   u32 blend_layer, u32 blend_match) {
    DCR(WINDOW_HEADER) = BIT(4 + w);
    DCR(WIN_OPTIONS)   = 0;
    DCR(COLOR_DEPTH)   = 0xC; /* B8G8R8A8 */
    DCR(POSITION)      = pos;
    DCR(SIZE)          = height << 16 | width;
    DCR(PRESCALED)     = height << 16 | width * 4;
    DCR(LINE_STRIDE)   = stride;
    DCR(SURFACE_KIND)  = kind;
    DCR(START_ADDR)    = addr;
    DCR(ADDR_H_OFFSET) = h_off;
    DCR(ADDR_V_OFFSET) = v_off;
    DCR(BLEND_LAYER)   = blend_layer;
    DCR(BLEND_MATCH)   = blend_match;
    DCR(WIN_OPTIONS)   = opts | WIN_ENABLE;
    DCR(STATE_CONTROL) = BIT(8) | BIT(9 + w); /* GENERAL + WIN_x_UPDATE */
    DCR(STATE_CONTROL) = BIT(0) | BIT(1 + w); /* GENERAL + WIN_x_ACT_REQ */
}

#define BYPASS 0x01000000u /* BLEND_LAYER_CONTROL reset: blending bypassed */

__attribute__((section(".text.start"), noreturn))
void _start(void) {
#if SCENARIO == 1
    fill(SRC_P, 720, 1280, pat);
    window(0, 0, 0, 720, 1280, 720 * 4, KIND_PITCH, SRC_P, 0, 0, BYPASS, 0);
#elif SCENARIO == 2
    fill(SRC_L, 1280, 720, pat);
    vic_compose(SRC_L, 0, 0, DST, 0, 0, 1280, 720, 1, 1);
    window(0, 0, 0, 720, 1280, 720 * 4, KIND_PITCH, DST, 0, 0, BYPASS, 0);
#elif SCENARIO == 3
    fill(SRC_L, 1280, 720, pat);
    vic_compose(SRC_L, 0, 0, DST, 1, 4, 1280, 720, 1, 1);
    window(0, 0, 0, 720, 1280, 720 * 4, KIND_BLOCK, DST, 0, 0, BYPASS, 0);
#elif SCENARIO == 4
    fill(SRC_L, 1280, 720, pat);
    vic_compose(SRC_L, 0, 0, SURF_B, 1, 4, 1280, 720, 0, 0);
    window(0, SCAN_COLUMN | H_DIRECTION, 0, 720, 1280, 1280 * 4, KIND_BLOCK,
           SURF_B, 1280 * 4 - 1, 0, BYPASS, 0);
#elif SCENARIO == 5
    fill(SRC_L, 1280, 720, pat);
    vic_compose(SRC_L, 0, 0, SURF_B, 1, 4, 1280, 720, 0, 0);
    vic_compose(SURF_B, 1, 4, DST, 0, 0, 1280, 720, 1, 1);
    window(0, 0, 0, 720, 1280, 720 * 4, KIND_PITCH, DST, 0, 0, BYPASS, 0);
#elif SCENARIO == 6
    fill(SRC_P, 720, 1280, pat);
    fill(SRC_Q, 200, 300, patq);
    window(0, V_DIRECTION, 0, 720, 1280, 720 * 4, KIND_PITCH, SRC_P, 0, 1279,
           BYPASS, 0);
    /* K1 = 128, blending on, depth 0; SRC x K1 + DST x (1 - K1). */
    window(3, 0, 200u << 16 | 100u, 200, 300, 200 * 4, KIND_PITCH, SRC_Q,
           0, 0, 128u << 8, 0x72u);
#endif
    puts_("display test: ready\n");
    for (;;)
        ;
}
