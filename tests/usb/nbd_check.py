#!/usr/bin/env python3
"""NBD client for tests/usb/run.sh: talks to the disk the emulator's
--usb-host nbd serves and checks what comes back.

Usage: nbd_check.py <port> <log> rw|ro

Waits for the host to say it is serving, negotiates with NBD_OPT_GO, reads
the whole disk (tests/usb/payload.c fills it with a known pattern), checks
that bad requests are refused, then - on a writable disk - writes two
sectors, reads them back and flushes, or - on a write-protected one - checks
that the write is refused. Two reads go out back to back to check that
pipelined requests are answered in turn. Prints the CRC32 the disk should
have once the client is done, then disconnects.
"""

import socket
import struct
import sys
import time
import zlib

NBDMAGIC = 0x4E42444D41474943
IHAVEOPT = 0x49484156454F5054
REPLY_MAGIC = 0x0003E889045565A9
OPT_GO, OPT_LIST = 7, 3
REP_ACK, REP_SERVER, REP_INFO = 1, 2, 3
CMD_READ, CMD_WRITE, CMD_DISC, CMD_FLUSH = 0, 1, 2, 3
FLAG_READ_ONLY, FLAG_SEND_FLUSH = 1 << 1, 1 << 2
EPERM, EINVAL = 1, 22

LBAS = 384
failed = False


def check(ok, what):
    global failed
    print(("  ok    " if ok else "  FAIL  ") + what)
    if not ok:
        failed = True


def pattern():
    return bytes(((i * 7) ^ (i >> 9)) & 0xFF for i in range(LBAS * 512))


def recv_all(s, n):
    b = b""
    while len(b) < n:
        c = s.recv(n - len(b))
        if not c:
            raise EOFError("server closed the connection")
        b += c
    return b


def option(s, opt, data=b""):
    s.sendall(struct.pack(">QII", IHAVEOPT, opt, len(data)) + data)
    replies = []
    while True:
        magic, ropt, rtype, rlen = struct.unpack(">QIII", recv_all(s, 20))
        if magic != REPLY_MAGIC or ropt != opt:
            raise ValueError("bad option reply")
        replies.append((rtype, recv_all(s, rlen)))
        if rtype == REP_ACK or rtype & 0x80000000:
            return replies


def request(s, cmd, handle, offset, length, data=b""):
    s.sendall(struct.pack(">IHHQQI", 0x25609513, 0, cmd, handle, offset,
                          length) + data)


def reply(s, want_handle, length=0):
    magic, err, handle = struct.unpack(">IIQ", recv_all(s, 16))
    if magic != 0x67446698 or handle != want_handle:
        raise ValueError("bad reply (magic %08X, handle %d)" % (magic, handle))
    return err, (recv_all(s, length) if not err and length else b"")


def main():
    port, log, mode = int(sys.argv[1]), sys.argv[2], sys.argv[3]
    deadline = time.time() + 60
    while "serving the disk over NBD" not in open(log, errors="replace").read():
        if time.time() > deadline:
            check(False, "the host serves the disk over NBD")
            return 1
        time.sleep(0.1)

    s = socket.create_connection(("127.0.0.1", port), timeout=30)
    magic, opt_magic, hflags = struct.unpack(">QQH", recv_all(s, 18))
    check(magic == NBDMAGIC and opt_magic == IHAVEOPT and hflags & 1,
          "NBD %s: fixed newstyle greeting" % mode)
    s.sendall(struct.pack(">I", 3))            # fixed newstyle, no zeroes

    listing = option(s, OPT_LIST)
    check([r[0] for r in listing] == [REP_SERVER, REP_ACK],
          "NBD %s: NBD_OPT_LIST names one export" % mode)

    go = option(s, OPT_GO, struct.pack(">IH", 0, 0))
    size = flags = None
    for rtype, data in go:
        if rtype == REP_INFO and struct.unpack(">H", data[:2])[0] == 0:
            size, flags = struct.unpack(">QH", data[2:12])
    check(go[-1][0] == REP_ACK and size == LBAS * 512,
          "NBD %s: NBD_OPT_GO, export of %s bytes" % (mode, size))
    check(bool(flags & FLAG_READ_ONLY) == (mode == "ro") and
          bool(flags & FLAG_SEND_FLUSH),
          "NBD %s: transmission flags %s" % (mode, flags and hex(flags)))

    disk = bytearray(pattern())
    request(s, CMD_READ, 1, 0, len(disk))
    err, data = reply(s, 1, len(disk))
    check(err == 0 and data == disk,
          "NBD %s: reading the whole disk returns it" % mode)

    request(s, CMD_READ, 2, 100, 512)
    check(reply(s, 2)[0] == EINVAL, "NBD %s: an unaligned read is refused" % mode)
    request(s, CMD_READ, 3, len(disk), 512)
    check(reply(s, 3)[0] == EINVAL, "NBD %s: a read past the end is refused" % mode)

    new = bytes((i * 13 + 5) & 0xFF for i in range(1024))
    request(s, CMD_WRITE, 4, 10 * 512, len(new), new)
    err = reply(s, 4)[0]
    if mode == "rw":
        check(err == 0, "NBD rw: writing blocks 10-11")
        disk[10 * 512:12 * 512] = new
    else:
        check(err == EPERM, "NBD ro: a write is refused")

    request(s, CMD_READ, 5, 8 * 512, 4 * 512)
    request(s, CMD_READ, 6, (LBAS - 1) * 512, 512)
    e5, d5 = reply(s, 5, 4 * 512)
    e6, d6 = reply(s, 6, 512)
    check(e5 == 0 and d5 == disk[8 * 512:12 * 512] and
          e6 == 0 and d6 == disk[(LBAS - 1) * 512:],
          "NBD %s: two pipelined reads%s" %
          (mode, ", the first seeing the write" if mode == "rw" else ""))

    request(s, CMD_FLUSH, 7, 0, 0)
    check(reply(s, 7)[0] == 0, "NBD %s: flush" % mode)

    request(s, CMD_DISC, 8, 0, 0)
    s.close()
    print("nbd check: disk crc32 %08X" % (zlib.crc32(disk) & 0xFFFFFFFF))
    return 1 if failed else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, EOFError, ValueError) as e:
        print("  FAIL  NBD client: %s" % e)
        sys.exit(1)
