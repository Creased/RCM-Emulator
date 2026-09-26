#pragma once

#include <cstdint>
#include <vector>

// A one-client NBD server - the network block device protocol, fixed
// newstyle handshake - that the emulated USB host (usb.cpp) serves a mass
// storage gadget's disk with, so the disk can be mounted on the machine the
// emulator runs on. It listens on 127.0.0.1 only. Single-threaded and
// non-blocking: poll() does whatever the socket allows and returns.

struct NbdRequest {
  uint16_t type = 0;           // NBD_CMD_*
  uint16_t flags = 0;
  uint64_t handle = 0, offset = 0;
  uint32_t len = 0;
  std::vector<uint8_t> data;   // what a WRITE carries
};

constexpr uint16_t NBD_CMD_READ = 0, NBD_CMD_WRITE = 1, NBD_CMD_DISC = 2,
                   NBD_CMD_FLUSH = 3;
constexpr uint32_t NBD_EPERM = 1, NBD_EIO = 5, NBD_EINVAL = 22;

// The largest READ or WRITE a client may send (the protocol's default).
constexpr uint32_t NBD_MAX_PAYLOAD = 32u << 20;

enum class NbdEvent {
  None,     // nothing to do yet
  Request,  // *req holds the next command
  Closed,   // the client that was using the disk disconnected
};

class NbdServer {
public:
  ~NbdServer() { close(); }

  // Listen on 127.0.0.1:port for a client, exporting `size` bytes. False
  // (and logged) when the port cannot be had.
  bool listen(uint16_t port, uint64_t size, bool read_only);
  void close();
  bool active() const { return state_ != Idle; }

  // Accept, run the handshake, read the next request. A client that goes
  // away before it has the disk (a listing, a failed negotiation) is not a
  // Closed event: the server just waits for the next one.
  NbdEvent poll(NbdRequest *req);

  // Answer a request; a READ's data goes with a zero error.
  void reply(uint64_t handle, uint32_t error, const uint8_t *data = nullptr,
             size_t len = 0);

private:
  enum State { Idle, Listening, ClientFlags, Options, Transmission };
  State state_ = Idle;
  intptr_t listen_ = -1, client_ = -1;  // sockets
  std::vector<uint8_t> in_;
  bool peer_closed_ = false, no_zeroes_ = false, read_only_ = false;
  uint64_t size_ = 0;

  bool fill();
  bool send_all(const void *p, size_t n);
  bool option(uint32_t opt, const std::vector<uint8_t> &data);
  bool option_reply(uint32_t opt, uint32_t type,
                    const std::vector<uint8_t> &data = {});
  void drop_client();
  uint16_t transmission_flags() const;
};
