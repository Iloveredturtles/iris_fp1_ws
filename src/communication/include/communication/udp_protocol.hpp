// udp_protocol.hpp — wire format for talking to the simulator over UDP.
//
// Header-only and free of ROS, so it can be unit tested with plain g++.
//
// Packet layout (little endian, which is what x86 already uses natively):
//
//   command  (ROS -> sim)   12 bytes : "IRIS" | motor_pwm | steering_deg
//   feedback (sim -> ROS)   20 bytes : "IRIS" | left_ticks | right_ticks
//                                             | vel_x_cms  | vel_y_cms
//
// VERIFY THESE AGAINST THE MODULE before trusting them. If the byte layout is
// wrong, nothing downstream works and the failure looks like "the car ignores
// me", which is miserable to debug.

#ifndef COMMUNICATION_UDP_PROTOCOL_HPP
#define COMMUNICATION_UDP_PROTOCOL_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#if defined(__linux__)
#include <endian.h>
#else
#define htole32(x) (x)
#define le32toh(x) (x)
#endif

namespace communication {

inline constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
inline constexpr std::size_t kHeaderSize = 4;

inline constexpr std::size_t kCommandPacketSize = kHeaderSize + 2 * sizeof(int32_t);   // 12
inline constexpr std::size_t kFeedbackPacketSize = kHeaderSize + 4 * sizeof(int32_t);  // 20

// The simulator ignores values outside these, so clamp before sending rather
// than letting a runaway controller push nonsense onto the wire.
inline constexpr int32_t kPwmLimit = 255;
inline constexpr int32_t kSteerLimitDeg = 90;

struct DriveCommand {
  int32_t motor_pwm = 0;     // -255 .. +255, sign = direction
  int32_t steering_deg = 0;  //  -90 ..  +90, sign = left/right
};

struct Feedback {
  int32_t left_ticks = 0;
  int32_t right_ticks = 0;
  int32_t vel_x_cms = 0;
  int32_t vel_y_cms = 0;
};

inline int32_t clampInt(int32_t value, int32_t lo, int32_t hi) {
  return std::max(lo, std::min(hi, value));
}

// memcpy rather than a cast: a reinterpret_cast to int32_t* on an unaligned
// byte buffer is undefined behaviour, and the optimiser is allowed to assume
// it never happens.
inline void putInt32(unsigned char* dst, int32_t value) {
  const uint32_t le = htole32(static_cast<uint32_t>(value));
  std::memcpy(dst, &le, sizeof(le));
}

inline int32_t getInt32(const unsigned char* src) {
  uint32_t le = 0;
  std::memcpy(&le, src, sizeof(le));
  return static_cast<int32_t>(le32toh(le));
}

// Writes exactly kCommandPacketSize bytes into buf and returns that count.
inline std::size_t buildCommandPacket(unsigned char* buf, const DriveCommand& cmd) {
  std::memcpy(buf, kHeader, kHeaderSize);
  putInt32(buf + kHeaderSize + 0, clampInt(cmd.motor_pwm, -kPwmLimit, kPwmLimit));
  putInt32(buf + kHeaderSize + 4, clampInt(cmd.steering_deg, -kSteerLimitDeg, kSteerLimitDeg));
  return kCommandPacketSize;
}

// False means "not a packet for us" — short, or the header did not match.
// Dropping silently is correct here: UDP lets anything arrive on the port.
inline bool parseFeedbackPacket(const unsigned char* buf, std::size_t len, Feedback& out) {
  if (len < kFeedbackPacketSize) {
    return false;
  }
  if (std::memcmp(buf, kHeader, kHeaderSize) != 0) {
    return false;
  }
  out.left_ticks = getInt32(buf + 4);
  out.right_ticks = getInt32(buf + 8);
  out.vel_x_cms = getInt32(buf + 12);
  out.vel_y_cms = getInt32(buf + 16);
  return true;
}

}  // namespace communication

#endif  // COMMUNICATION_UDP_PROTOCOL_HPP
