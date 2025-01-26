#include "helper.hpp"

#include <gtest/gtest.h>

namespace timr {

namespace driver {

class HelperTest : public ::testing::Test {
protected:

public:

    void SetUp() override {


    }    

    void TearDown() override {

    }
};

TEST_F(HelperTest, TEST_CRC8_CHECKSUM) {
    // Test case 1
    std::array<uint8_t, 5> data1 = {0x01, 0x02, 0x03, 0x04, 0x05};
    EXPECT_EQ(crc8_checksum(data1.data(), data1.size()), 0x68);

    // Test case 2 
    std::array<uint8_t, 4> data2 = {0xFF, 0xAA, 0x55, 0x00};
    EXPECT_EQ(crc8_checksum(data2.data(), data2.size()), 0xc6);

    // Test case 3
    std::array<uint8_t, 6> data3 = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
    EXPECT_EQ(crc8_checksum(data3.data(), data3.size()), 0xca);
}

TEST_F(HelperTest, TEST_PRINT_FUNCTIONS) {
    // Test print_can_frames
    std::array<can_frame, 2> frames;
    frames[0].can_id = 0x123 | CAN_EFF_FLAG;
    frames[0].can_dlc = 8;
    for(int i = 0; i < 8; i++) frames[0].data[i] = i;
    
    frames[1].can_id = 0x456 | CAN_EFF_FLAG; 
    frames[1].can_dlc = 4;
    for(int i = 0; i < 4; i++) frames[1].data[i] = i + 8;

    testing::internal::CaptureStdout();
    print_can_frames(frames);
    std::string output1 = testing::internal::GetCapturedStdout();
    std::string expected_output1 = "Frame 1/2\nID: 0x123\nLength: 8\ndata: 0 1 2 3 4 5 6 7 \nFrame 2/2\nID: 0x456\nLength: 4\ndata: 8 9 a b \n";
    EXPECT_EQ(output1, expected_output1);

    // Test print_serial_data
    std::array<uint8_t, 5> data = {0x01, 0x02, 0x03, 0x04, 0x05};
    
    testing::internal::CaptureStdout();
    print_serial_data(data);
    std::string output2 = testing::internal::GetCapturedStdout();
    std::string expected_output2 = "1 2 3 4 5 \n";
    EXPECT_EQ(output2, expected_output2);
}


TEST_F(HelperTest, TEST_SERIAL_CAN_CONVERSION_1) {
    // Test data for serial_data_to_can_frames and can_frames_to_serial_data
    std::array<uint8_t, 9> test_data = {0x06, 0x84, 0x8A, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00};
    
    // Convert to CAN frames
    auto frames = serial_data_to_can_frames(test_data);
    
    // Verify frame properties
    EXPECT_EQ(frames.size(), 1);
    EXPECT_EQ(frames[0].can_id & ~CAN_EFF_FLAG, 0x0600);
    EXPECT_EQ(frames[0].can_dlc, 8);
    EXPECT_EQ(frames[0].data[0], 0x84); // Common first byte (function code)
    EXPECT_EQ(frames[0].data[1], 0x8A); // Common 2nd byte
    EXPECT_EQ(frames[0].data[2], 0x01); // Common 3rd byte
    EXPECT_EQ(frames[0].data[3], 0x08); // Common 4th byte
    EXPECT_EQ(frames[0].data[4], 0x00); // Common 5th byte
    EXPECT_EQ(frames[0].data[5], 0x00); // Common 6th byte
    EXPECT_EQ(frames[0].data[6], 0x00); // Common 7th byte
    EXPECT_EQ(frames[0].data[7], 0x00); // Common 8th byte
    
    // Convert back to serial data
    auto recovered_data = can_frames_to_serial_data<9>(frames);
    
    // Verify recovered data matches original
    EXPECT_EQ(recovered_data.size(), test_data.size());
    for(size_t i = 0; i < test_data.size(); i++) {
        EXPECT_EQ(recovered_data[i], test_data[i]);
    }
    
    // Test with larger data requiring multiple frames
    std::array<uint8_t, 16> large_test_data = {
        0x06, 0x84, 0x8A, 0x01, 0x08, 0x12, 0x34, 0x56,
        0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0xAB, 0xCD, 0xEF
    };
    
    auto large_frames = serial_data_to_can_frames(large_test_data);
    
    // Verify multiple frames
    EXPECT_EQ(large_frames.size(), 2);
    EXPECT_EQ(large_frames[0].can_id & ~CAN_EFF_FLAG, 0x0600);
    EXPECT_EQ(large_frames[0].can_dlc, 8);
    EXPECT_EQ(large_frames[0].data[0], 0x84); // Common first byte (function code)
    EXPECT_EQ(large_frames[0].data[1], 0x8A); // Common 2nd byte
    EXPECT_EQ(large_frames[0].data[2], 0x01); // Common 3rd byte
    EXPECT_EQ(large_frames[0].data[3], 0x08); // Common 4th byte
    EXPECT_EQ(large_frames[0].data[4], 0x12); // Common 5th byte
    EXPECT_EQ(large_frames[0].data[5], 0x34); // Common 6th byte
    EXPECT_EQ(large_frames[0].data[6], 0x56); // Common 7th byte
    EXPECT_EQ(large_frames[0].data[7], 0x78); // Common 8th byte
    
    EXPECT_EQ(large_frames[1].can_id & ~CAN_EFF_FLAG, 0x0601);
    EXPECT_EQ(large_frames[1].can_dlc, 8);
    EXPECT_EQ(large_frames[1].data[0], 0x84); // Common first byte (function code)
    EXPECT_EQ(large_frames[1].data[1], 0x9A); // Common 9th byte
    EXPECT_EQ(large_frames[1].data[2], 0xBC); // Common 10th byte
    EXPECT_EQ(large_frames[1].data[3], 0xDE); // Common 11th byte
    EXPECT_EQ(large_frames[1].data[4], 0xF0); // Common 12th byte
    EXPECT_EQ(large_frames[1].data[5], 0xAB); // Common 13th byte
    EXPECT_EQ(large_frames[1].data[6], 0xCD); // Common 14th byte
    EXPECT_EQ(large_frames[1].data[7], 0xEF); // Common 15th byte
    
    // Convert back and verify
    auto recovered_large_data = can_frames_to_serial_data<16>(large_frames);
    EXPECT_EQ(recovered_large_data.size(), large_test_data.size());
    for(size_t i = 0; i < large_test_data.size(); i++) {
        EXPECT_EQ(recovered_large_data[i], large_test_data[i]);
    }
}


TEST_F(HelperTest, TEST_SERIAL_CAN_CONVERSION_2) {
    // Test data for serial_data_to_can_frames and can_frames_to_serial_data
    std::array<uint8_t, 6> test_data = {0x01, 0xF6, 0x00, 0x03, 0xE8, 0xFF};
    
    // Convert to CAN frames
    auto frames = serial_data_to_can_frames(test_data);
    
    // Verify frame properties
    EXPECT_EQ(frames.size(), 1);
    EXPECT_EQ(frames[0].can_id & ~CAN_EFF_FLAG, 0x0100);
    EXPECT_EQ(frames[0].can_dlc, 5);
    EXPECT_EQ(frames[0].data[0], 0xF6); // Common 1st byte (function code)
    EXPECT_EQ(frames[0].data[1], 0x00); // Common 2nd byte
    EXPECT_EQ(frames[0].data[2], 0x03); // Common 3rd byte
    EXPECT_EQ(frames[0].data[3], 0xE8); // Common 4th byte
    EXPECT_EQ(frames[0].data[4], 0xFF); // Common 5th byte

    // Convert back to serial data
    auto recovered_data = can_frames_to_serial_data<6>(frames);
    
    // Verify recovered data matches original
    EXPECT_EQ(recovered_data.size(), test_data.size());
    for(size_t i = 0; i < test_data.size(); i++) {
        EXPECT_EQ(recovered_data[i], test_data[i]);
    }
    
    // Test with larger data requiring multiple frames
    std::array<uint8_t, 11> large_test_data = {
        0x02, 0xF6, 0x00, 0x03, 0xE8, 0xFF, 0x00, 0x00,
        0x01, 0x02, 0x03
    };
    
    auto large_frames = serial_data_to_can_frames(large_test_data);
    
    // Verify multiple frames
    EXPECT_EQ(large_frames.size(), 2);
    EXPECT_EQ(large_frames[0].can_id & ~CAN_EFF_FLAG, 0x0200);
    EXPECT_EQ(large_frames[0].can_dlc, 8);
    EXPECT_EQ(large_frames[0].data[0], 0xF6); // Common 1st byte (function code)
    EXPECT_EQ(large_frames[0].data[1], 0x00); // Common 2nd byte
    EXPECT_EQ(large_frames[0].data[2], 0x03); // Common 3rd byte
    EXPECT_EQ(large_frames[0].data[3], 0xE8); // Common 4th byte
    EXPECT_EQ(large_frames[0].data[4], 0xFF); // Common 5th byte
    EXPECT_EQ(large_frames[0].data[5], 0x00); // Common 6th byte
    EXPECT_EQ(large_frames[0].data[6], 0x00); // Common 7th byte
    EXPECT_EQ(large_frames[0].data[7], 0x01); // Common 8th byte
    
    EXPECT_EQ(large_frames[1].can_id & ~CAN_EFF_FLAG, 0x0201);
    EXPECT_EQ(large_frames[1].can_dlc, 3);    
    EXPECT_EQ(large_frames[1].data[0], 0xF6); // Common 1st byte (function code)
    EXPECT_EQ(large_frames[1].data[1], 0x02); // Common 9th byte
    EXPECT_EQ(large_frames[1].data[2], 0x03); // Common 10th byte
    // Convert back and verify
    auto recovered_large_data = can_frames_to_serial_data<11>(large_frames);
    EXPECT_EQ(recovered_large_data.size(), large_test_data.size());
    for(size_t i = 0; i < large_test_data.size(); i++) {
        EXPECT_EQ(recovered_large_data[i], large_test_data[i]);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

} // namespace driver

} // namespace timr