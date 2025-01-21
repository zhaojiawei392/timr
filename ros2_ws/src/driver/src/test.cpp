#include "driver.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <linux/can/j1939.h>
#include <linux/can/bcm.h>
#include <linux/can/netlink.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

// Add this function before main()
void send_can_frame(const std::vector<uint8_t>& data, int fd) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));

    // Set CAN ID (just use the first byte as ID)
    frame.can_id = data[0] << 8 | CAN_EFF_FLAG;  // Using extended frame format

    // Set data length (up to 8 bytes for regular CAN)
    size_t len = data.size() - 1;
    if (len > 8) {
        throw std::runtime_error("Data too long for CAN (max 8 bytes)");
    }

    frame.can_dlc = len;  // Use can_dlc instead of len

    // Zero out the entire data field first
    memset(frame.data, 0, sizeof(frame.data));

    // Copy data (skipping first byte which was used for ID)
    for (size_t i = 1; i < data.size(); i++) {
        frame.data[i - 1] = data[i];
    }

    // Print frame details before sending
    std::cout << "Sending CAN frame:" << std::endl;
    std::cout << "ID: 0x" << std::hex << (frame.can_id & ~CAN_EFF_FLAG) << std::endl;
    std::cout << "Length: " << std::dec << (int)frame.can_dlc << std::endl;
    std::cout << "Data: ";
    for (size_t i = 0; i < frame.can_dlc; i++) {
        std::cout << std::hex << (int)frame.data[i] << " ";
    }
    std::cout << std::endl;

    // Write with CAN_MTU
    ssize_t nbytes_sent = write(fd, &frame, CAN_MTU);
    std::cout << "nbytes_sent: " << nbytes_sent << std::endl;
    if (nbytes_sent != CAN_MTU) {
        std::cout << "Expected bytes: " << CAN_MTU << std::endl;
        std::cout << "Actual bytes sent: " << nbytes_sent << std::endl;
        throw std::runtime_error("Failed to write CAN frame");
    }
    std::cout << "Frame sent successfully" << std::endl;

}

void receive_can_frame(int fd) {
    struct can_frame frame_recv;
    // Set receive timeout
    struct timeval timeout;
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        perror("setsockopt failed");
        return ;
    }   
    ssize_t nbytes = ::read(fd, &frame_recv, sizeof(frame_recv));
    if (nbytes < 0) {
        perror("read failed");
        return ;
    }

    std::cout << "Frame received successfully" << std::endl;
    std::cout << "Frame ID: 0x" << std::hex << (frame_recv.can_id & ~CAN_EFF_FLAG) << std::endl;
    std::cout << "Frame Length: " << std::dec << (int)frame_recv.can_dlc << std::endl;
    std::cout << "Frame data: ";
    for (int i = 0; i < frame_recv.can_dlc; i++) {
        std::cout << std::hex << (int)frame_recv.data[i] << " ";
    }
    std::cout << std::endl;
}

void send_canfd_frame(const std::vector<uint8_t>& data, int fd) {
    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame));

    // Set CAN ID (just use the first byte as ID)
    frame.can_id = data[0] << 8 | CAN_EFF_FLAG;  // Using extended frame format

    // Set data length (up to 64 bytes for CAN FD)
    size_t len = data.size() - 1;
    if (len > CANFD_MAX_DLEN) {
        throw std::runtime_error("Data too long for CAN FD (max 64 bytes)");
    }

    frame.len = len;
    frame.flags = 0;  // Add this line to explicitly set flags to 0
                      // You can use CANFD_BRS for bit rate switching if needed

    // Zero out the entire data field first
    memset(frame.data, 0, sizeof(frame.data));

    // Copy data (skipping first byte which was used for ID)
    for (size_t i = 1; i < data.size(); i++) {
        frame.data[i - 1] = data[i];
    }

    // Print frame details before sending
    std::cout << "Sending CAN FD frame:" << std::endl;
    std::cout << "ID: 0x" << std::hex << (frame.can_id & ~CAN_EFF_FLAG) << std::endl;
    std::cout << "Length: " << std::dec << (int)frame.len << std::endl;
    std::cout << "Data: ";
    for (size_t i = 0; i < frame.len; i++) {
        std::cout << std::hex << (int)frame.data[i] << " ";
    }
    std::cout << std::endl;

    // Enable CAN FD mode
    int enable_canfd = 1;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
        perror("setsockopt failed");  // Add error logging
        throw std::runtime_error("Error enabling CAN FD frames");
    }

    // Write with CANFD_MTU
    ssize_t nbytes_sent = write(fd, &frame, CANFD_MTU);
    std::cout << "nbytes_sent: " << nbytes_sent << std::endl;
    if (nbytes_sent != CANFD_MTU) {
        std::cout << "Expected bytes: " << CANFD_MTU << std::endl;
        std::cout << "Actual bytes sent: " << nbytes_sent << std::endl;
        perror("Write failed");
        throw std::runtime_error("Failed to write CAN FD frame");
    }
    std::cout << "Frame sent successfully" << std::endl;

}

int main() {
    YAML::Node config = YAML::LoadFile("/home/kai/Projects/timr/ros2_ws/src/driver/config/test.yaml");
    std::string can_interface = config["can_interface"].as<std::string>();
    std::vector<uint8_t> data;
    for (const auto& value : config["data"]) {
        data.push_back(value.as<uint8_t>());
    }

    // Create CAN socket (removed FD reference)
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        perror("socket");
        return 1;
    }

    struct sockaddr_can addr;
    struct ifreq ifr;
    
    strcpy(ifr.ifr_name, can_interface.c_str());
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        return 1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return 1;
    }

    try {
        send_canfd_frame(data, fd);
        receive_can_frame(fd);
    } catch (const std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    close(fd);
    return 0;
}