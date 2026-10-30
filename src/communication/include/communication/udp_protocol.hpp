// udp_protocol.hpp
// how we pack and unpack the bytes we send to the car

#ifndef UDP_PROTOCOL_HPP
#define UDP_PROTOCOL_HPP

#include <cstdint>
#include <cstring>
#include <endian.h>

namespace communication {

// every packet starts with this so we can ignore junk arriving on the port
const char PACKET_HEADER[4] = {'I', 'R', 'I', 'S'};
const int HEADER_BYTES = 4;

const int COMMAND_PACKET_BYTES  = 12;   // header + motor + steering
const int FEEDBACK_PACKET_BYTES = 20;   // header + 4 numbers

const int MAX_MOTOR_PWM   = 255;
const int MAX_STEERING_DEG = 90;

// what we send to the car
struct DriveCommand {
    int motor_pwm = 0;      // -255 back, +255 forward
    int steering_deg = 0;   // -90 right, +90 left
};

// what the car sends back
struct Feedback {
    int left_wheel_ticks = 0;
    int right_wheel_ticks = 0;
    int forward_speed_cms = 0;
    int sideways_speed_cms = 0;
};

inline int clamp_int(int value, int lowest, int highest) {
    if (value < lowest)  return lowest;
    if (value > highest) return highest;
    return value;
}

// write one 4 byte number into the buffer
inline void put_int(unsigned char* buffer, int value) {
    uint32_t little_endian = htole32((uint32_t)value);
    memcpy(buffer, &little_endian, 4);
}

// read one 4 byte number back out
inline int get_int(const unsigned char* buffer) {
    uint32_t little_endian = 0;
    memcpy(&little_endian, buffer, 4);
    return (int)le32toh(little_endian);
}

// build the 12 byte packet we send every tick
inline void build_command(unsigned char* buffer, const DriveCommand& command) {
    memcpy(buffer, PACKET_HEADER, HEADER_BYTES);
    put_int(buffer + 4, clamp_int(command.motor_pwm, -MAX_MOTOR_PWM, MAX_MOTOR_PWM));
    put_int(buffer + 8, clamp_int(command.steering_deg, -MAX_STEERING_DEG, MAX_STEERING_DEG));
}

// false means the packet is too short or not ours, so just drop it
inline bool parse_feedback(const unsigned char* buffer, int length, Feedback& out) {
    if (length < FEEDBACK_PACKET_BYTES) return false;
    if (memcmp(buffer, PACKET_HEADER, HEADER_BYTES) != 0) return false;

    out.left_wheel_ticks    = get_int(buffer + 4);
    out.right_wheel_ticks   = get_int(buffer + 8);
    out.forward_speed_cms   = get_int(buffer + 12);
    out.sideways_speed_cms  = get_int(buffer + 16);
    return true;
}

}  // namespace communication

#endif
