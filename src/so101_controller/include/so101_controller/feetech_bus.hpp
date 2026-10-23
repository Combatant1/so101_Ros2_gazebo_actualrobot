// Feetech STS3215 serial-bus protocol (SCS/STS "half-duplex" packet format).
//
// This header is deliberately free of ROS and libserial: it talks to a tiny
// ByteTransport interface so that it can be unit-tested against a simulated bus.
//
// Packet formats (all little-endian):
//   instruction: FF FF <id> <len> <inst> <params...> <chk>        len = n_params + 2
//   status     : FF FF <id> <len> <err>  <params...> <chk>        len = n_params + 2
//   chk = ~(id + len + inst/err + sum(params)) & 0xFF
#ifndef SO101_CONTROLLER__FEETECH_BUS_HPP_
#define SO101_CONTROLLER__FEETECH_BUS_HPP_

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace so101_controller
{

namespace sts3215
{
// --- Instructions
constexpr uint8_t INST_PING = 0x01;
constexpr uint8_t INST_READ = 0x02;
constexpr uint8_t INST_WRITE = 0x03;
constexpr uint8_t INST_SYNC_READ = 0x82;
constexpr uint8_t INST_SYNC_WRITE = 0x83;
constexpr uint8_t BROADCAST_ID = 0xFE;

// --- Control-table addresses (SRAM unless noted)
constexpr uint8_t REG_OPERATING_MODE = 33;    // EEPROM: 0 = position servo
constexpr uint8_t REG_TORQUE_ENABLE = 40;
constexpr uint8_t REG_ACCELERATION = 41;      // 1 byte, units of 100 steps/s^2
constexpr uint8_t REG_GOAL_POSITION = 42;     // 2 bytes
constexpr uint8_t REG_GOAL_TIME = 44;         // 2 bytes
constexpr uint8_t REG_GOAL_SPEED = 46;        // 2 bytes, 0 = no limit
constexpr uint8_t REG_TORQUE_LIMIT = 48;      // 2 bytes, 0..1000
constexpr uint8_t REG_PRESENT_POSITION = 56;  // 2 bytes

// Block read starting at REG_PRESENT_POSITION:
//   [0..1] position  [2..3] speed  [4..5] load  [6] voltage (0.1 V)  [7] temperature (C)
constexpr uint8_t PRESENT_BLOCK_LEN = 8;
}  // namespace sts3215

// Minimal byte-stream abstraction so the protocol can be tested without hardware.
class ByteTransport
{
public:
  virtual ~ByteTransport() = default;
  // Write all bytes (throws std::exception on failure).
  virtual void write(const uint8_t * data, size_t n) = 0;
  // Wait up to timeout_ms for at least one byte, then return everything that is
  // immediately available (at most max). Returns 0 on timeout.
  virtual size_t read(uint8_t * buf, size_t max, int timeout_ms) = 0;
  // Drop any unread input.
  virtual void flush_input() = 0;
};

struct StatusPacket
{
  uint8_t id = 0;
  uint8_t error = 0;
  std::vector<uint8_t> params;
};

struct ReadResult
{
  bool ok = false;
  uint8_t error = 0;  // servo error flags (overload, over-temp, ...)
  std::vector<uint8_t> data;
};

class FeetechBus
{
public:
  using Clock = std::chrono::steady_clock;

  // timeout_ms: how long to wait for replies to one transaction.
  // echo: set true if your adapter echoes transmitted bytes back to the receiver.
  explicit FeetechBus(ByteTransport & transport, int timeout_ms = 8, bool echo = false)
  : transport_(transport), timeout_ms_(timeout_ms), echo_(echo)
  {
  }

  // ---- packet helpers (static so tests can use them) ---------------------------------
  static uint8_t checksum(const uint8_t * p, size_t n)
  {
    uint8_t sum = 0;
    for (size_t i = 0; i < n; ++i) {sum = static_cast<uint8_t>(sum + p[i]);}
    return static_cast<uint8_t>(~sum);
  }

  static std::vector<uint8_t> buildPacket(
    uint8_t id, uint8_t inst, const std::vector<uint8_t> & params)
  {
    std::vector<uint8_t> p;
    p.reserve(params.size() + 6);
    p.push_back(0xFF);
    p.push_back(0xFF);
    p.push_back(id);
    p.push_back(static_cast<uint8_t>(params.size() + 2));
    p.push_back(inst);
    p.insert(p.end(), params.begin(), params.end());
    p.push_back(checksum(p.data() + 2, p.size() - 2));
    return p;
  }

  // Pull one valid status packet out of rx (resynchronising on garbage). Returns false if
  // more bytes are needed.
  static bool extractPacket(std::vector<uint8_t> & rx, StatusPacket & out)
  {
    for (;;) {
      size_t i = 0;
      while (i + 1 < rx.size() && !(rx[i] == 0xFF && rx[i + 1] == 0xFF)) {++i;}
      if (i > 0) {rx.erase(rx.begin(), rx.begin() + static_cast<std::ptrdiff_t>(i));}
      if (rx.size() < 4) {return false;}
      if (rx[2] == 0xFF) {  // extra 0xFF padding before the real header
        rx.erase(rx.begin());
        continue;
      }
      const uint8_t len = rx[3];
      if (len < 2) {
        rx.erase(rx.begin());
        continue;
      }
      const size_t total = static_cast<size_t>(len) + 4;
      if (rx.size() < total) {return false;}
      if (checksum(rx.data() + 2, total - 3) != rx[total - 1]) {
        rx.erase(rx.begin());  // corrupt: skip one byte and re-sync
        continue;
      }
      out.id = rx[2];
      out.error = rx[4];
      out.params.assign(rx.begin() + 5, rx.begin() + static_cast<std::ptrdiff_t>(total) - 1);
      rx.erase(rx.begin(), rx.begin() + static_cast<std::ptrdiff_t>(total));
      return true;
    }
  }

  // ---- transactions -------------------------------------------------------------------
  bool ping(uint8_t id)
  {
    transmit(buildPacket(id, sts3215::INST_PING, {}));
    StatusPacket sp;
    return waitStatus(id, Clock::now() + ms(timeout_ms_), sp);
  }

  bool readRegister(uint8_t id, uint8_t addr, uint8_t len, std::vector<uint8_t> & data,
    uint8_t * error = nullptr)
  {
    transmit(buildPacket(id, sts3215::INST_READ, {addr, len}));
    StatusPacket sp;
    if (!waitStatus(id, Clock::now() + ms(timeout_ms_), sp) || sp.params.size() != len) {
      return false;
    }
    data = sp.params;
    if (error) {*error = sp.error;}
    return true;
  }

  // Write to one servo and wait for its acknowledgement.
  bool writeRegister(uint8_t id, uint8_t addr, const std::vector<uint8_t> & data)
  {
    std::vector<uint8_t> params{addr};
    params.insert(params.end(), data.begin(), data.end());
    transmit(buildPacket(id, sts3215::INST_WRITE, params));
    StatusPacket sp;
    return waitStatus(id, Clock::now() + ms(timeout_ms_), sp);
  }

  // Write the same register block on several servos with one broadcast packet (no reply).
  void syncWrite(uint8_t addr, uint8_t len,
    const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & items)
  {
    std::vector<uint8_t> params{addr, len};
    for (const auto & item : items) {
      params.push_back(item.first);
      params.insert(params.end(), item.second.begin(), item.second.end());
    }
    transmit(buildPacket(sts3215::BROADCAST_ID, sts3215::INST_SYNC_WRITE, params), false);
  }

  // Read the same register block from several servos with one request. out[i] corresponds
  // to ids[i]. Returns the number of servos that answered correctly.
  size_t syncRead(const std::vector<uint8_t> & ids, uint8_t addr, uint8_t len,
    std::vector<ReadResult> & out)
  {
    out.assign(ids.size(), ReadResult{});
    std::vector<uint8_t> params{addr, len};
    params.insert(params.end(), ids.begin(), ids.end());
    transmit(buildPacket(sts3215::BROADCAST_ID, sts3215::INST_SYNC_READ, params));

    size_t got = 0;
    const auto deadline = Clock::now() + ms(timeout_ms_);
    StatusPacket sp;
    while (got < ids.size()) {
      while (extractPacket(rx_, sp)) {
        for (size_t i = 0; i < ids.size(); ++i) {
          if (ids[i] == sp.id && !out[i].ok && sp.params.size() == len) {
            out[i].ok = true;
            out[i].error = sp.error;
            out[i].data = sp.params;
            ++got;
            break;
          }
        }
      }
      if (got >= ids.size() || !pump(deadline)) {break;}
    }
    return got;
  }

private:
  static Clock::duration ms(int v) {return std::chrono::milliseconds(v);}

  void transmit(const std::vector<uint8_t> & pkt, bool expect_reply = true)
  {
    transport_.flush_input();
    rx_.clear();
    transport_.write(pkt.data(), pkt.size());
    if (echo_ && expect_reply) {discardEcho(pkt.size());}
  }

  void discardEcho(size_t n)
  {
    const auto deadline = Clock::now() + ms(timeout_ms_);
    uint8_t buf[256];
    while (n > 0) {
      const auto now = Clock::now();
      if (now >= deadline) {return;}
      int wait = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
      if (wait < 1) {wait = 1;}
      const size_t got = transport_.read(buf, std::min(n, sizeof(buf)), wait);
      if (got == 0) {return;}
      n -= got;
    }
  }

  // Read more bytes into rx_. Returns false on timeout.
  bool pump(Clock::time_point deadline)
  {
    const auto now = Clock::now();
    if (now >= deadline) {return false;}
    int wait = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
    if (wait < 1) {wait = 1;}
    uint8_t buf[256];
    const size_t n = transport_.read(buf, sizeof(buf), wait);
    if (n == 0) {return false;}
    rx_.insert(rx_.end(), buf, buf + n);
    return true;
  }

  bool waitStatus(uint8_t id, Clock::time_point deadline, StatusPacket & sp)
  {
    for (;;) {
      while (extractPacket(rx_, sp)) {
        if (sp.id == id) {return true;}
      }
      if (!pump(deadline)) {return false;}
    }
  }

  ByteTransport & transport_;
  int timeout_ms_;
  bool echo_;
  std::vector<uint8_t> rx_;
};

}  // namespace so101_controller

#endif  // SO101_CONTROLLER__FEETECH_BUS_HPP_
