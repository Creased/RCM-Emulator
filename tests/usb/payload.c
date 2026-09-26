/*
 * USB device mode regression payload.
 *
 * A freestanding BPMP payload that brings up a small USB mass storage gadget
 * - one LUN, a 128-block RAM disk - on one of the two device controllers,
 * with the register sequences bdk's drivers use, and serves whatever the
 * emulator's --usb-host PC asks. Built once per scenario with -DSCENARIO=n:
 *
 *   1  the USB2 controller USB1 (bdk usbd.c, Erista): queue heads in the
 *      controller at +0x1000, transfer descriptors in DRAM
 *   2  the XUSB device controller (bdk xusbd.c, Mariko): event ring,
 *      transfer rings and endpoint contexts in IRAM
 *   3  the USB2 controller again, as a HID gadget: after the report
 *      descriptor and SET_IDLE it sends three interrupt IN reports
 *
 * Either way the gadget answers the standard requests and GET_MAX_LUN, then
 * the Bulk-Only Transport: INQUIRY, TEST UNIT READY, REQUEST SENSE, READ
 * CAPACITY, READ(10), PREVENT ALLOW MEDIUM REMOVAL and START STOP UNIT. When
 * the host ejects, it stops the controller and powers off. It prints the
 * CRC32 of the disk ranges the host reads, which tests/usb/run.sh compares
 * with what the host logs. Run without --usb-host, scenario 1 sees no bus
 * reset and says so.
 */

#ifndef SCENARIO
#define SCENARIO 1
#endif

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

#define REG(a) (*(volatile u32 *)(a))
#define B(a)   (*(volatile u8 *)(a))

#define TIMERUS   0x60005010u
#define UARTB_THR 0x70006040u
#define I2C5      0x7000D000u

#define DISK       0xC0000000u /* 128 blocks of 512 bytes */
#define DISK_LBAS  128u
#define EP0BUF     0xC0110000u
#define CMDBUF     0xC0111000u /* CBW / CSW / small replies */

static void putc_(char c) { REG(UARTB_THR) = (u32)(unsigned char)c; }

static void puts_(const char *s) {
    while (*s)
        putc_(*s++);
}

static void puthex(u32 v) {
    for (int i = 28; i >= 0; i -= 4)
        putc_("0123456789ABCDEF"[(v >> i) & 0xF]);
}

static void i2c5_write(u32 dev, u32 reg, u32 val) {
    REG(I2C5 + 0x04) = dev << 1;
    REG(I2C5 + 0x0C) = reg | (val << 8);
    REG(I2C5 + 0x00) = (1u << 1) | (1u << 9);
}

static __attribute__((noreturn)) void finish(const char *msg) {
    puts_(msg);
    i2c5_write(0x3C, 0x41, 0x02); /* MAX77620 ONOFFCNFG1: PWR_OFF */
    for (;;)
        ;
}

static void copy(u32 dst, const u8 *src, u32 n) {
    for (u32 i = 0; i < n; i++)
        B(dst + i) = src[i];
}

#if SCENARIO != 3
static void zero(u32 dst, u32 n) {
    for (u32 i = 0; i < n; i += 4)
        REG(dst + i) = 0;
}
#endif

static u32 crc32(u32 addr, u32 n) {
    u32 c = 0xFFFFFFFFu;
    for (u32 i = 0; i < n; i++) {
        c ^= B(addr + i);
        for (int k = 0; k < 8; k++)
            c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1)));
    }
    return ~c;
}

/* ---- Descriptors --------------------------------------------------------- */

static const u8 dev_desc[18] = {
    18, 1, 0x00, 0x02, 0, 0, 0, 64,
    0x09, 0x12, 0x57, 0x7E,  /* 1209:7E57 */
    0x00, 0x01, 1, 2, 3, 1,
};

#if SCENARIO == 3
static const u8 hid_report[21] = {
    0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01,   /* vendor page, application */
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x08,
    0x09, 0x01, 0x81, 0x02, 0xC0,               /* 8 bytes of input */
};

static const u8 cfg_desc[34] = {
    9, 2, 34, 0, 1, 1, 0, 0x80, 50,
    9, 4, 0, 0, 1, 0x03, 0x00, 0x00, 0,         /* HID */
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, 21, 0,     /* HID descriptor */
    7, 5, 0x81, 3, 8, 0, 4,                     /* interrupt IN, 8 bytes */
};
#else
static const u8 cfg_desc[32] = {
    9, 2, 32, 0, 1, 1, 0, 0x80, 50,
    9, 4, 0, 0, 2, 0x08, 0x06, 0x50, 0,         /* mass storage, SCSI, BOT */
    7, 5, 0x81, 2, 0x00, 0x02, 0,               /* bulk IN, 512 */
    7, 5, 0x01, 2, 0x00, 0x02, 0,               /* bulk OUT, 512 */
};
#endif

static const u8 str_lang[4] = {4, 3, 0x09, 0x04};
static const char *const strs[3] = {"RCM-Emulator", "USB test disk", "0001"};

#if SCENARIO != 3
static const u8 inquiry[36] = {
    0x00, 0x80, 0x02, 0x02, 31, 0, 0, 0,
    'R', 'C', 'M', 'E', 'M', 'U', ' ', ' ',
    'T', 'e', 's', 't', ' ', 'd', 'i', 's', 'k', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
    '1', '.', '0', '0',
};
#endif

/* ---- Transport ------------------------------------------------------------ */
/*
 * What the gadget needs from a controller:
 *   poll_setup(pkt)       a SETUP packet, if one came (1) or not (0)
 *   ep0_in(len)           send EP0BUF[0..len), then the status stage
 *   ep0_status()          a no-data request's status stage
 *   ep0_stall()           reject the request
 *   set_address(a)        after its status stage
 *   configure()           bring up the bulk endpoints
 *   bulk(in, buf, len)    one bulk transfer; returns bytes moved
 *   stop()                detach
 */

#if SCENARIO != 2
/* ---- USB2 controller USB1 (bdk usbd.c) ---- */

#define USB     0x7D000000u
#define U(o)    REG(USB + (o))
#define USBCMD  0x130
#define USBSTS  0x134
#define DEVADDR 0x144
#define EPLIST  0x148
#define USBMODE 0x1F8
#define SETUPST 0x208
#define PRIME   0x20C
#define FLUSH   0x210
#define STATUS  0x214
#define COMPL   0x218
#define EPCTRL(n) (0x21C + 4 * (n))
#define SUSP    0x400
#define QHB     (USB + 0x1000)
#define QH(i)   (QHB + 64 * (i))
#define DTD     0xC0100000u /* 4 dTDs per queue head, 32 bytes each */

static void init(void) {
    U(SUSP) |= (1u << 12) | (1u << 11);         /* UTMIP_PHY_ENB, UTMIP_RESET */
    U(SUSP) &= ~(1u << 11);
    while (!(U(SUSP) & (1u << 7)))              /* PHY_CLK_VALID */
        ;
    U(USBCMD) |= 1u << 1;                       /* RESET */
    while (U(USBCMD) & (1u << 1))
        ;
    U(USBMODE) = 2;                             /* device */
    for (u32 i = 0; i < 4; i++) {
        for (u32 w = 0; w < 16; w++)
            REG(QH(i) + 4 * w) = 0;
        REG(QH(i) + 8) = 1;                     /* no dTD */
    }
    REG(QH(0)) = 64u << 16 | 1u << 15;          /* EP0 OUT: 64, IOS */
    REG(QH(1)) = 64u << 16;
    U(EPLIST) = QHB;
    U(USBCMD) |= 1;                             /* RUN: attach */
}

/* One transfer on queue head idx (2 * ep + in), in up to four 16 KiB dTDs as
 * bdk builds them. Returns the bytes moved, or -1 if it never completed. */
static int xfer(u32 idx, u32 buf, u32 len) {
    u32 base = DTD + idx * 4 * 32, off = 0, n = 0;
    do {
        u32 d = base + n * 32, sz = len - off > 16384 ? 16384 : len - off;
        REG(d) = 1;
        REG(d + 4) = sz << 16 | 0x80;           /* bytes, ACTIVE */
        for (u32 p = 0; p < 5; p++)
            REG(d + 8 + 4 * p) = buf ? buf + off + 4096 * p : 0;
        if (n)
            REG(base + (n - 1) * 32) = d;
        off += sz;
        n++;
    } while (off < len && n < 4);
    REG(QH(idx) + 8) = base;
    REG(QH(idx) + 0xC) = 0;
    u32 bit = idx & 1 ? 1u << (16 + idx / 2) : 1u << (idx / 2);
    U(PRIME) = bit;
    u32 start = REG(TIMERUS);
    while ((U(PRIME) | U(STATUS)) & bit)
        if (REG(TIMERUS) - start > 5000000)
            return -1;
    U(COMPL) = bit;
    u32 left = 0;
    for (u32 i = 0; i < n; i++)
        left += (REG(base + i * 32 + 4) >> 16) & 0x7FFF;
    return (int)(len - left);
}

static int poll_setup(u8 *pkt) {
    u32 sts = U(USBSTS);
    U(USBSTS) = sts;
    if (sts & (1u << 6)) {                      /* bus reset */
        U(DEVADDR) = 0;
        U(SETUPST) = U(SETUPST);
        U(COMPL) = U(COMPL);
        U(FLUSH) = 0xFFFFFFFFu;
    }
    if (!(U(SETUPST) & 1))
        return 0;
    U(SETUPST) = 1;
    u32 lo = REG(QH(0) + 0x28), hi = REG(QH(0) + 0x2C);
    for (int i = 0; i < 4; i++) {
        pkt[i] = lo >> (8 * i);
        pkt[4 + i] = hi >> (8 * i);
    }
    return 1;
}

static void ep0_in(u32 len) {
    xfer(1, EP0BUF, len);
    xfer(0, 0, 0);
}

static void ep0_status(void) { xfer(1, 0, 0); }

static void ep0_stall(void) { U(EPCTRL(0)) |= 1u | 1u << 16; }

static void set_address(u32 a) { U(DEVADDR) = a << 25; }

static void configure(void) {
    for (u32 i = 2; i < 4; i++) {
        for (u32 w = 0; w < 16; w++)
            REG(QH(i) + 4 * w) = 0;
        REG(QH(i)) = 512u << 16;
        REG(QH(i) + 8) = 1;
    }
    U(EPCTRL(1)) = 2u << 2 | 1u << 7 | 2u << 18 | 1u << 23; /* bulk RX/TX on */
}

static int bulk(int in, u32 buf, u32 len) { return xfer(in ? 3 : 2, buf, len); }

static void stop(void) {
    U(USBCMD) &= ~1u;
    (void)U(USBCMD);                            /* let the host see it go */
}

/* Without a host nothing resets the bus: say so after half a second. */
static int bus_reset_seen(void) {
    u32 start = REG(TIMERUS);
    while (REG(TIMERUS) - start < 500000)
        if (U(USBSTS) & (1u << 6))
            return 1;
    return 0;
}

#else
/* ---- XUSB device controller (bdk xusbd.c) ---- */

#define XU      0x700D0000u
#define X(o)    REG(XU + (o))
#define X_DB     0x04
#define X_ERSTSZ 0x08
#define X_ERST0  0x10
#define X_ERST1  0x18
#define X_ERDP   0x20
#define X_EREP   0x28
#define X_CTRL   0x30
#define X_ST     0x34
#define X_IMOD   0x38
#define X_PORTSC 0x3C
#define X_ECP    0x40
#define X_HALT   0x50
#define X_RELOAD 0x58

#define XMEM  0x40030000u
#define SEG0  (XMEM)
#define SEG1  (XMEM + 0x100)
#define CTX   (XMEM + 0x500) /* four 64-byte endpoint contexts */

struct ring { u32 base, idx, pcs; };
static struct ring rings[4]; /* by context index: 0 control, 2 OUT, 3 IN */
static u32 evt_deq, ccs, seq;

static void ring_init(u32 dci, u32 base) {
    struct ring *r = &rings[dci];
    r->base = base;
    r->idx = 0;
    r->pcs = 1;
    zero(base, 16 * 16);
    REG(base + 15 * 16) = base;                 /* Link TRB back to the start */
    REG(base + 15 * 16 + 12) = 6u << 10 | 2;    /* type LINK, toggle cycle */
    u32 ctx = CTX + 64 * dci;
    zero(ctx, 64);
    REG(ctx) = 1;                               /* EP_RUNNING */
    REG(ctx + 8) = base | 1;                    /* dequeue pointer, DCS */
}

static void queue(u32 dci, u32 w0, u32 w2, u32 w3) {
    struct ring *r = &rings[dci];
    u32 t = r->base + r->idx * 16;
    REG(t) = w0;
    REG(t + 4) = 0;
    REG(t + 8) = w2;
    REG(t + 12) = w3 | r->pcs;
    if (++r->idx == 15) {
        REG(r->base + 15 * 16 + 12) = 6u << 10 | 2 | r->pcs;
        r->idx = 0;
        r->pcs ^= 1;
    }
    X(X_DB) = dci << 8 | (dci ? 0 : seq << 16);
}

static void init(void) {
    zero(XMEM, 0x600);
    X(X_ERST0) = SEG0;
    X(X_ERST1) = SEG1;
    X(X_ERSTSZ) = 16u << 16 | 16;
    X(X_EREP) = SEG0 | 1;
    X(X_ERDP) = SEG0;
    evt_deq = SEG0;
    ccs = 1;
    ring_init(0, XMEM + 0x200);
    X(X_CTRL) |= 1u << 4 | 1u << 1;             /* IE, LSE */
    X(X_ECP) = CTX;
    X(X_IMOD) = 0;
    X(X_PORTSC) = (X(X_PORTSC) & ~(0xFu << 5)) | 1u << 16 | 5u << 5; /* RxDetect */
    X(X_CTRL) |= 1u << 31;                      /* ENABLE */
}

/* The next event, if any. Polling ST lets the host run. */
static int next_event(u32 *ev) {
    (void)X(X_ST);
    if ((REG(evt_deq + 12) & 1) != ccs)
        return 0;
    for (int i = 0; i < 4; i++)
        ev[i] = REG(evt_deq + 4 * i);
    if (evt_deq == SEG0 + 15 * 16)
        evt_deq = SEG1;
    else if (evt_deq == SEG1 + 15 * 16) {
        evt_deq = SEG0;
        ccs ^= 1;
    } else
        evt_deq += 16;
    X(X_ERDP) = evt_deq;
    X(X_ST) = 1u << 4;                          /* IP, write 1 to clear */
    return 1;
}

static void port_change(void) {
    u32 ps = X(X_PORTSC);
    X(X_PORTSC) = ps;                           /* write the change bits back */
}

/* Wait for the Transfer event of endpoint dci; the bytes it left over. */
static int wait_xfer(u32 dci) {
    u32 ev[4], start = REG(TIMERUS);
    for (;;) {
        if (REG(TIMERUS) - start > 5000000)
            return -1;
        if (!next_event(ev))
            continue;
        u32 type = ev[3] >> 10 & 0x3F;
        if (type == 34)
            port_change();
        else if (type == 32 && (ev[3] >> 16 & 0x1F) == dci)
            return (int)(ev[2] & 0xFFFFFF);
    }
}

static int poll_setup(u8 *pkt) {
    u32 ev[4];
    if (!next_event(ev))
        return 0;
    u32 type = ev[3] >> 10 & 0x3F;
    if (type == 34) {
        port_change();
        return 0;
    }
    if (type != 63)
        return 0;
    for (int i = 0; i < 4; i++) {
        pkt[i] = ev[0] >> (8 * i);
        pkt[4 + i] = ev[1] >> (8 * i);
    }
    seq = ev[2] & 0xFFFF;
    return 1;
}

static void ep0_in(u32 len) {
    queue(0, EP0BUF, len, 3u << 10 | 1u << 16 | 1u << 5); /* Data IN, IOC */
    wait_xfer(0);
    queue(0, 0, 0, 4u << 10 | 1u << 5);                   /* Status OUT */
    wait_xfer(0);
}

static void ep0_status(void) {
    queue(0, 0, 0, 4u << 10 | 1u << 16 | 1u << 5);        /* Status IN */
    wait_xfer(0);
}

static void ep0_stall(void) { X(X_HALT) |= 1; }

static void set_address(u32 a) { X(X_CTRL) = (X(X_CTRL) & 0x80FFFFFFu) | a << 24; }

static void configure(void) {
    ring_init(2, XMEM + 0x300);
    ring_init(3, XMEM + 0x400);
    X(X_RELOAD) = 0xC;
    while (X(X_RELOAD) & 0xC)
        ;
    X(X_HALT) &= ~0xCu;
    X(X_CTRL) |= 1;                             /* RUN */
}

static int bulk(int in, u32 buf, u32 len) {
    u32 dci = in ? 3 : 2;
    queue(dci, buf, len, 1u << 10 | 1u << 5 | 1u << 2); /* Normal, IOC, ISP */
    int left = wait_xfer(dci);
    return left < 0 ? -1 : (int)(len - left);
}

static void stop(void) {
    X(X_CTRL) = 0;
    (void)X(X_CTRL);
}

static int bus_reset_seen(void) { return 1; }

#endif

/* ---- The gadget ----------------------------------------------------------- */

static int configured, lun_asked, report_asked, idle_set;

static void control(const u8 *p) {
    u32 bm = p[0], req = p[1], val = p[2] | p[3] << 8, len = p[6] | p[7] << 8;
    u32 n = 0;
    if (bm == 0x80 && req == 6) {               /* GET_DESCRIPTOR */
        u32 type = val >> 8, idx = val & 0xFF;
        if (type == 1) {
            copy(EP0BUF, dev_desc, n = 18);
        } else if (type == 2) {
            copy(EP0BUF, cfg_desc, n = sizeof(cfg_desc));
        } else if (type == 3 && idx == 0) {
            copy(EP0BUF, str_lang, n = 4);
        } else if (type == 3 && idx <= 3) {
            const char *s = strs[idx - 1];
            n = 2;
            while (*s) {
                B(EP0BUF + n) = *s++;
                B(EP0BUF + n + 1) = 0;
                n += 2;
            }
            B(EP0BUF) = n;
            B(EP0BUF + 1) = 3;
        } else {
            ep0_stall();
            return;
        }
        ep0_in(n < len ? n : len);
    } else if (bm == 0x00 && req == 5) {        /* SET_ADDRESS */
        ep0_status();
        set_address(val & 0x7F);
    } else if (bm == 0x00 && req == 9) {        /* SET_CONFIGURATION */
        configure();
        ep0_status();
        configured = 1;
        puts_("usb test: configured\n");
    } else if (bm == 0xA1 && req == 0xFE) {     /* GET_MAX_LUN */
        B(EP0BUF) = 0;
        ep0_in(1);
        lun_asked = 1;
#if SCENARIO == 3
    } else if (bm == 0x81 && req == 6 && (val >> 8) == 0x22) { /* HID report */
        copy(EP0BUF, hid_report, n = sizeof(hid_report));
        ep0_in(n < len ? n : len);
        report_asked = 1;
    } else if (bm == 0x21 && req == 0x0A) {     /* SET_IDLE */
        ep0_status();
        idle_set = 1;
#endif
    } else {
        ep0_stall();
    }
}

#if SCENARIO != 3
static void put_be32(u32 a, u32 v) {
    B(a) = v >> 24;
    B(a + 1) = v >> 16;
    B(a + 2) = v >> 8;
    B(a + 3) = v;
}

/* Serve Bulk-Only Transport commands until the host ejects the disk. */
static void bot(void) {
    u32 sense = 6, asc = 0x29; /* a unit attention first, like a real disk */
    for (;;) {
        int n = bulk(0, CMDBUF, 512);
        if (n != 31 || REG(CMDBUF) != 0x43425355u) /* "USBC" */
            finish("usb test: FAIL bad CBW\n");
        u32 tag = REG(CMDBUF + 4), want = REG(CMDBUF + 8);
        u8 op = B(CMDBUF + 15);
        u32 cdb = CMDBUF + 15, status = 0, sent = 0, eject = 0;
        switch (op) {
        case 0x12:                              /* INQUIRY */
            copy(CMDBUF + 0x100, inquiry, 36);
            sent = bulk(1, CMDBUF + 0x100, want < 36 ? want : 36);
            break;
        case 0x00:                              /* TEST UNIT READY */
            status = sense ? 1 : 0;
            break;
        case 0x03:                              /* REQUEST SENSE */
            zero(CMDBUF + 0x100, 20);
            B(CMDBUF + 0x100) = 0x70;
            B(CMDBUF + 0x102) = sense;
            B(CMDBUF + 0x107) = 10;
            B(CMDBUF + 0x10C) = asc;
            sense = asc = 0;
            sent = bulk(1, CMDBUF + 0x100, want < 18 ? want : 18);
            break;
        case 0x25:                              /* READ CAPACITY(10) */
            put_be32(CMDBUF + 0x100, DISK_LBAS - 1);
            put_be32(CMDBUF + 0x104, 512);
            sent = bulk(1, CMDBUF + 0x100, 8);
            break;
        case 0x28: {                            /* READ(10) */
            u32 lba = (u32)B(cdb + 2) << 24 | B(cdb + 3) << 16 | B(cdb + 4) << 8 |
                      B(cdb + 5);
            u32 cnt = B(cdb + 7) << 8 | B(cdb + 8);
            if (lba + cnt > DISK_LBAS)
                status = 1;
            else
                sent = bulk(1, DISK + lba * 512, cnt * 512);
            break;
        }
        case 0x1E:                              /* PREVENT ALLOW MEDIUM REMOVAL */
            break;
        case 0x1B:                              /* START STOP UNIT */
            eject = B(cdb + 4) & 2;
            break;
        default:
            status = 1;
            break;
        }
        REG(CMDBUF + 0x200) = 0x53425355u;      /* "USBS" */
        REG(CMDBUF + 0x204) = tag;
        REG(CMDBUF + 0x208) = want - sent;
        B(CMDBUF + 0x20C) = status;
        bulk(1, CMDBUF + 0x200, 13);
        if (eject)
            return;
    }
}
#endif

__attribute__((section(".text.start"), noreturn))
void _start(void) {
    for (u32 i = 0; i < DISK_LBAS * 512; i++)
        B(DISK + i) = (u8)((i * 7) ^ (i >> 9));
    puts_("usb test: disk crc32 first ");
    puthex(crc32(DISK, 64 * 512));
    puts_(" last ");
    puthex(crc32(DISK + (DISK_LBAS - 8) * 512, 8 * 512));
    putc_('\n');

    init();
    if (!bus_reset_seen()) {
        stop();
        finish("usb test: no bus reset, nothing on the cable\n");
    }
    u8 pkt[8];
    u32 start = REG(TIMERUS);
    while (!(configured && (SCENARIO == 3 ? report_asked && idle_set : lun_asked))) {
        if (poll_setup(pkt))
            control(pkt);
        if (REG(TIMERUS) - start > 10000000)
            finish("usb test: FAIL enumeration timed out\n");
    }
#if SCENARIO == 3
    for (u32 k = 1; k <= 3; k++) {
        for (u32 i = 0; i < 8; i++)
            B(CMDBUF + i) = (u8)(k * 0x10 + i);
        if (bulk(1, CMDBUF, 8) != 8)
            finish("usb test: FAIL report not taken\n");
    }
    puts_("usb test: hid reports sent\n");
#else
    bot();
    puts_("usb test: ejected\n");
#endif
    stop();
    finish("usb test: done\n");
}
