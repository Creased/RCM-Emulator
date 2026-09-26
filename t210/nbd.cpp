// A one-client NBD server for the emulated USB host (see nbd.h).
//
// The handshake is the fixed newstyle one every current client speaks
// (nbd-client, qemu-nbd, nbdfuse, libnbd):
//
//   server  "NBDMAGIC" "IHAVEOPT" handshake flags
//   client  its flags, then options: "IHAVEOPT", option, length, data
//   server  answers each option; EXPORT_NAME or GO hands over the disk
//
// then requests (magic 0x25609513, flags, type, handle, offset, length,
// and a WRITE's data) each answered by a simple reply (magic 0x67446698,
// error, handle, and a READ's data). The export has one name - any name
// the client asks for - and takes READ, WRITE, FLUSH and DISC; structured
// replies and the other extensions are refused, which clients handle.

#include "nbd.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace {

constexpr uint64_t NBDMAGIC = 0x4E42444D41474943ull;       // "NBDMAGIC"
constexpr uint64_t IHAVEOPT = 0x49484156454F5054ull;       // "IHAVEOPT"
constexpr uint64_t REPLY_MAGIC = 0x0003E889045565A9ull;    // option replies
constexpr uint32_t REQUEST_MAGIC = 0x25609513;
constexpr uint32_t SIMPLE_REPLY_MAGIC = 0x67446698;

constexpr uint16_t FLAG_FIXED_NEWSTYLE = 1 << 0, FLAG_NO_ZEROES = 1 << 1;
constexpr uint16_t FLAG_HAS_FLAGS = 1 << 0, FLAG_READ_ONLY = 1 << 1,
                   FLAG_SEND_FLUSH = 1 << 2;

constexpr uint32_t OPT_EXPORT_NAME = 1, OPT_ABORT = 2, OPT_LIST = 3,
                   OPT_INFO = 6, OPT_GO = 7;
constexpr uint32_t REP_ACK = 1, REP_SERVER = 2, REP_INFO = 3,
                   REP_ERR_UNSUP = 0x80000001, REP_ERR_INVALID = 0x80000003;
constexpr uint16_t INFO_EXPORT = 0, INFO_BLOCK_SIZE = 3;

#ifdef _WIN32
using sock_t = SOCKET;
const sock_t BAD_SOCK = INVALID_SOCKET;
bool net_init() {
  static bool ok = [] {
    WSADATA d;
    return WSAStartup(MAKEWORD(2, 2), &d) == 0;
  }();
  return ok;
}
void close_sock(sock_t s) { closesocket(s); }
bool would_block() { return WSAGetLastError() == WSAEWOULDBLOCK; }
bool set_nonblocking(sock_t s) {
  u_long on = 1;
  return ioctlsocket(s, FIONBIO, &on) == 0;
}
const char *sock_error() {
  static char buf[32];
  snprintf(buf, sizeof(buf), "winsock error %d", WSAGetLastError());
  return buf;
}
#else
using sock_t = int;
const sock_t BAD_SOCK = -1;
bool net_init() { return true; }
void close_sock(sock_t s) { ::close(s); }
bool would_block() {
#if EWOULDBLOCK != EAGAIN
  if (errno == EWOULDBLOCK)
    return true;
#endif
  return errno == EAGAIN || errno == EINTR;
}
bool set_nonblocking(sock_t s) {
  int fl = fcntl(s, F_GETFL, 0);
  return fl >= 0 && fcntl(s, F_SETFL, fl | O_NONBLOCK) == 0;
}
const char *sock_error() { return strerror(errno); }
#endif

#ifdef MSG_NOSIGNAL
constexpr int SEND_FLAGS = MSG_NOSIGNAL; // a gone client is an error, not SIGPIPE
#else
constexpr int SEND_FLAGS = 0;
#endif

sock_t sock(intptr_t s) { return (sock_t)s; }

// Wait up to 10 s for room to send.
bool wait_writable(sock_t s) {
  fd_set w;
  FD_ZERO(&w);
  FD_SET(s, &w);
  timeval tv = {10, 0};
  return select((int)s + 1, nullptr, &w, nullptr, &tv) > 0;
}

// Everything on the wire is big-endian.
void put16(uint8_t *p, uint16_t v) {
  p[0] = v >> 8;
  p[1] = (uint8_t)v;
}
void put32(uint8_t *p, uint32_t v) {
  put16(p, v >> 16);
  put16(p + 2, (uint16_t)v);
}
void put64(uint8_t *p, uint64_t v) {
  put32(p, v >> 32);
  put32(p + 4, (uint32_t)v);
}
uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }
uint32_t get32(const uint8_t *p) {
  return (uint32_t)get16(p) << 16 | get16(p + 2);
}
uint64_t get64(const uint8_t *p) {
  return (uint64_t)get32(p) << 32 | get32(p + 4);
}

} // namespace

bool NbdServer::listen(uint16_t port, uint64_t size, bool read_only) {
  close();
  if (!net_init()) {
    printf("[usb-host] NBD: no sockets (%s)\n", sock_error());
    return false;
  }
  sock_t s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s == BAD_SOCK) {
    printf("[usb-host] NBD: no socket (%s)\n", sock_error());
    return false;
  }
#ifndef _WIN32
  // Windows' SO_REUSEADDR lets another process take a bound port; the
  // POSIX one only skips TIME_WAIT, so a restarted run gets its port back.
  int one = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
#endif
  sockaddr_in a;
  memset(&a, 0, sizeof(a));
  a.sin_family = AF_INET;
  a.sin_port = htons(port);
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(s, (sockaddr *)&a, sizeof(a)) != 0 || ::listen(s, 1) != 0 ||
      !set_nonblocking(s)) {
    printf("[usb-host] NBD: cannot listen on 127.0.0.1:%u (%s)\n", port,
           sock_error());
    close_sock(s);
    return false;
  }
  listen_ = (intptr_t)s;
  size_ = size;
  read_only_ = read_only;
  state_ = Listening;
  return true;
}

void NbdServer::close() {
  drop_client();
  if (listen_ != -1)
    close_sock(sock(listen_));
  listen_ = -1;
  state_ = Idle;
}

void NbdServer::drop_client() {
  if (client_ != -1)
    close_sock(sock(client_));
  client_ = -1;
  in_.clear();
  peer_closed_ = false;
  state_ = listen_ != -1 ? Listening : Idle;
}

uint16_t NbdServer::transmission_flags() const {
  return FLAG_HAS_FLAGS | FLAG_SEND_FLUSH | (read_only_ ? FLAG_READ_ONLY : 0);
}

// Read what the client has sent. False once it has closed its end.
bool NbdServer::fill() {
  static uint8_t buf[65536];
  // Room for the largest WRITE; a client pipelining more waits in the socket.
  while (in_.size() < NBD_MAX_PAYLOAD + sizeof(buf)) {
    int n = recv(sock(client_), (char *)buf, sizeof(buf), 0);
    if (n > 0) {
      in_.insert(in_.end(), buf, buf + n);
      continue;
    }
    return n < 0 && would_block();
  }
  return true;
}

bool NbdServer::send_all(const void *p, size_t n) {
  const char *c = (const char *)p;
  while (n) {
    int k = send(sock(client_), c, (int)std::min<size_t>(n, 1u << 20),
                 SEND_FLAGS);
    if (k > 0) {
      c += k;
      n -= (size_t)k;
    } else if (k < 0 && would_block()) {
      if (!wait_writable(sock(client_)))
        return false;
    } else {
      return false;
    }
  }
  return true;
}

bool NbdServer::option_reply(uint32_t opt, uint32_t type,
                             const std::vector<uint8_t> &data) {
  uint8_t h[20];
  put64(h, REPLY_MAGIC);
  put32(h + 8, opt);
  put32(h + 12, type);
  put32(h + 16, (uint32_t)data.size());
  return send_all(h, sizeof(h)) &&
         (data.empty() || send_all(data.data(), data.size()));
}

// One handshake option. False drops the client.
bool NbdServer::option(uint32_t opt, const std::vector<uint8_t> &d) {
  switch (opt) {
  case OPT_EXPORT_NAME: {
    // No reply header: the export's size and flags, then (unless the
    // client opted out) 124 zero bytes, and the disk is the client's.
    uint8_t r[10 + 124] = {};
    put64(r, size_);
    put16(r + 8, transmission_flags());
    if (!send_all(r, no_zeroes_ ? 10 : sizeof(r)))
      return false;
    state_ = Transmission;
    break;
  }
  case OPT_ABORT:
    option_reply(opt, REP_ACK);
    return false;
  case OPT_LIST: {
    if (!d.empty())
      return option_reply(opt, REP_ERR_INVALID);
    std::vector<uint8_t> unnamed(4, 0);   // one export, the default name
    return option_reply(opt, REP_SERVER, unnamed) && option_reply(opt, REP_ACK);
  }
  case OPT_INFO:
  case OPT_GO: {
    // Export name length, the name, the count of info requests, the
    // requests. Whatever the client asks for, it gets the size and flags
    // and the block sizes: 512-byte sectors, 4 KiB preferred.
    if (d.size() < 6)
      return option_reply(opt, REP_ERR_INVALID);
    uint32_t name_len = get32(&d[0]);
    if (name_len > d.size() - 6 ||
        d.size() != 6 + name_len + 2 * (size_t)get16(&d[4 + name_len]))
      return option_reply(opt, REP_ERR_INVALID);
    std::vector<uint8_t> e(12), b(14);
    put16(&e[0], INFO_EXPORT);
    put64(&e[2], size_);
    put16(&e[10], transmission_flags());
    put16(&b[0], INFO_BLOCK_SIZE);
    put32(&b[2], 512);
    put32(&b[6], 4096);
    put32(&b[10], NBD_MAX_PAYLOAD);
    if (!option_reply(opt, REP_INFO, e) || !option_reply(opt, REP_INFO, b) ||
        !option_reply(opt, REP_ACK))
      return false;
    if (opt == OPT_GO)
      state_ = Transmission;
    break;
  }
  default:
    return option_reply(opt, REP_ERR_UNSUP);
  }
  if (state_ == Transmission)
    printf("[usb-host] NBD client has the disk\n");
  return true;
}

NbdEvent NbdServer::poll(NbdRequest *req) {
  if (state_ == Idle)
    return NbdEvent::None;
  if (state_ == Listening) {
    sock_t c = accept(sock(listen_), nullptr, nullptr);
    if (c == BAD_SOCK)
      return NbdEvent::None;
    set_nonblocking(c);
    int one = 1;
    setsockopt(c, IPPROTO_TCP, TCP_NODELAY, (const char *)&one, sizeof(one));
#ifdef SO_NOSIGPIPE
    setsockopt(c, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
    client_ = (intptr_t)c;
    in_.clear();
    peer_closed_ = no_zeroes_ = false;
    uint8_t hello[18];
    put64(hello, NBDMAGIC);
    put64(hello + 8, IHAVEOPT);
    put16(hello + 16, FLAG_FIXED_NEWSTYLE | FLAG_NO_ZEROES);
    if (!send_all(hello, sizeof(hello))) {
      drop_client();
      return NbdEvent::None;
    }
    state_ = ClientFlags;
  }
  if (!fill())
    peer_closed_ = true;

  for (;;) {
    size_t n = in_.size();
    if (state_ == ClientFlags) {
      if (n < 4)
        break;
      uint32_t f = get32(&in_[0]);
      in_.erase(in_.begin(), in_.begin() + 4);
      no_zeroes_ = (f & FLAG_FIXED_NEWSTYLE) && (f & FLAG_NO_ZEROES);
      state_ = Options;
    } else if (state_ == Options) {
      if (n < 16)
        break;
      uint32_t opt = get32(&in_[8]), len = get32(&in_[12]);
      if (get64(&in_[0]) != IHAVEOPT || len > 65536) {
        printf("[usb-host] NBD: not an NBD client; dropped\n");
        drop_client();
        return NbdEvent::None;
      }
      if (n < 16 + (size_t)len)
        break;
      std::vector<uint8_t> data(in_.begin() + 16, in_.begin() + 16 + len);
      in_.erase(in_.begin(), in_.begin() + 16 + len);
      if (!option(opt, data)) {
        drop_client();
        return NbdEvent::None;
      }
    } else {
      if (n < 28)
        break;
      uint16_t type = get16(&in_[6]);
      uint32_t len = get32(&in_[24]);
      if (get32(&in_[0]) != REQUEST_MAGIC ||
          (type == NBD_CMD_WRITE && len > NBD_MAX_PAYLOAD)) {
        printf("[usb-host] NBD: malformed request; client dropped\n");
        drop_client();
        return NbdEvent::Closed;
      }
      size_t need = 28 + (type == NBD_CMD_WRITE ? len : 0);
      if (n < need)
        break;
      req->flags = get16(&in_[4]);
      req->type = type;
      req->handle = get64(&in_[8]);
      req->offset = get64(&in_[16]);
      req->len = len;
      req->data.assign(in_.begin() + 28, in_.begin() + need);
      in_.erase(in_.begin(), in_.begin() + need);
      if (type == NBD_CMD_DISC) {
        printf("[usb-host] NBD client disconnected\n");
        drop_client();
        return NbdEvent::Closed;
      }
      return NbdEvent::Request;
    }
  }

  if (peer_closed_) {
    bool had_disk = state_ == Transmission;
    printf("[usb-host] NBD client %s\n",
           had_disk ? "went away without disconnecting" : "left");
    drop_client();
    return had_disk ? NbdEvent::Closed : NbdEvent::None;
  }
  return NbdEvent::None;
}

void NbdServer::reply(uint64_t handle, uint32_t error, const uint8_t *data,
                      size_t len) {
  if (state_ != Transmission)
    return;
  uint8_t h[16];
  put32(h, SIMPLE_REPLY_MAGIC);
  put32(h + 4, error);
  put64(h + 8, handle);
  if (!send_all(h, sizeof(h)) || (!error && len && !send_all(data, len)))
    peer_closed_ = true; // the next poll reports it gone
}
