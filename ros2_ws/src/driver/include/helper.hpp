#pragma once
// C++ Standard Library
#include <csignal>
#include <cstring>
#include <string>
#include <stdexcept>
#include <cstdint>
#include <array>
#include <iostream>
// Linux System Headers
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

// CAN Bus Headers
#include <linux/can.h>

namespace timr {

namespace driver {

using dof_size_t = uint8_t;
using scalar_t = double;
constexpr dof_size_t DOF = 6;
constexpr bool DEBUG = false;
constexpr scalar_t HOMING_VEL = 0.5;
constexpr scalar_t HOMING_ACC = 0.1;
constexpr bool TEST_WITH_REAL_DRIVER = true;

constexpr uint8_t crc8_table[256] = {
0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35
};

inline uint8_t crc8_checksum(const uint8_t* data, size_t size) {        
    uint8_t crc8 = data[0];
    for(uint8_t i = 1; i < size; i++) {
        crc8 = crc8_table[crc8 ^ data[i]];
    }
    return crc8;
}

template<std::size_t size>
inline void print_serial_data(const std::array<uint8_t, size>& data) {
    for (size_t i = 0; i < data.size(); i++) {
        std::cout << std::hex << (int)data[i] << " ";
    }
    std::cout << "\n";
}

template<std::size_t size>
inline void print_can_frames(const std::array<can_frame, size>& frames) {
    for (size_t i = 0; i < frames.size(); i++) {
        std::cout << "Frame " << i + 1 << "/" << frames.size() << "\n";
        std::cout << "ID: 0x" << std::hex << (frames[i].can_id & ~CAN_EFF_FLAG) << "\n";
        std::cout << "Length: " << std::dec << (int)frames[i].can_dlc << "\n";
        std::cout << "data: ";
        for (size_t j = 0; j < frames[i].can_dlc; j++) {
            std::cout << std::hex << (int)frames[i].data[j] << " ";
        }
        std::cout << "\n";
    }
}

template<std::size_t serialSize>
inline std::array<can_frame, (serialSize + 4) / 7> serial_data_to_can_frames(std::array<uint8_t, serialSize>& data) {

    // Calculate number of frames needed (excluding the ID byte and common first byte)
    size_t total_data_size = serialSize - 2;  // Subtract 2 bytes (ID and common first byte)
    constexpr size_t num_frames = (serialSize + 4) / 7;  // Match return type size
    std::array<can_frame, num_frames> frames{};

    for (size_t frame_idx = 0; frame_idx < num_frames; frame_idx++) {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        // Sequential IDs: 0x0100, 0x0101, 0x0102, etc.
        frame.can_id = data[0] << 8 | frame_idx;
        frame.can_id |= CAN_EFF_FLAG;

        // Calculate data length for this frame
        size_t remaining_bytes = total_data_size - (frame_idx * 7);
        frame.can_dlc = (remaining_bytes > 7) ? 8 : remaining_bytes + 1;  // +1 for data[1]

        // First byte is always data[1]
        frame.data[0] = data[1];

        // Copy remaining data for this frame
        size_t data_offset = 2 + (frame_idx * 7);  // 2 for ID and common byte, then 7 bytes per previous frame
        memcpy(frame.data + 1, data.data() + data_offset, frame.can_dlc - 1);
        frames[frame_idx] = frame;
    }
    return frames;
}

template<std::size_t expectedSerialDataSize, std::size_t frameCount>
inline std::array<uint8_t, expectedSerialDataSize> can_frames_to_serial_data(const std::array<can_frame, frameCount>& frames) {
    std::array<uint8_t, expectedSerialDataSize> data;
    
    // First byte is the ID from the CAN frame
    data[0] = (frames[0].can_id >> 8) & 0xFF;
    // Second byte is the common first byte from frame data
    data[1] = frames[0].data[0];
    
    // Copy remaining data from each frame
    for (size_t frame_idx = 0; frame_idx < frameCount; frame_idx++) {
        size_t data_offset = 2 + (frame_idx * 7);  // 2 for ID and common byte, then 7 bytes per frame
        size_t bytes_to_copy = frames[frame_idx].can_dlc - 1;  // -1 because we skip the common first byte
        memcpy(data.data() + data_offset, frames[frame_idx].data + 1, bytes_to_copy);
    }
    
    return data;
}

int bind_can_socket(const std::string& can_interface) {
    // Create CAN socket
    int can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket == -1) {
        throw std::runtime_error("Failed to create socket: " + std::string(strerror(errno)));
    }

    // Set up CAN interface
    struct sockaddr_can addr;
    struct ifreq ifr;
    
    strcpy(ifr.ifr_name, can_interface.c_str());
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        close(can_socket);
        throw std::runtime_error("Failed to get interface index: " + std::string(strerror(errno)));
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(can_socket);
        throw std::runtime_error("Failed to bind socket: " + std::string(strerror(errno)));
    }
    return can_socket;
}

} // namespace driver

} // namespace timr