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

TEST_F(HelperTest, TEST_SERIAL_CAN_CONVERSION) {
    // Test data for serial_data_to_can_frames and can_frames_to_serial_data
    std::array<uint8_t, 9> test_data = {0x06, 0x84, 0x8A, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00};
    
    // Convert to CAN frames
    auto frames = serial_data_to_can_frames(test_data);
    
    // Verify frame properties
    EXPECT_EQ(frames.size(), 1);
    EXPECT_EQ(frames[0].can_id & ~CAN_EFF_FLAG, 0x0600);
    EXPECT_EQ(frames[0].can_dlc, 8);
    EXPECT_EQ(frames[0].data[0], 0x84); // Common first byte
    
    // Convert back to serial data
    auto recovered_data = can_frames_to_serial_data(frames);
    
    // Verify recovered data matches original
    EXPECT_EQ(recovered_data.size(), test_data.size());
    for(size_t i = 0; i < test_data.size(); i++) {
        EXPECT_EQ(recovered_data[i], test_data[i]);
    }
    
    // Test with larger data requiring multiple frames
    std::array<uint8_t, 16> large_test_data = {
        0x06, 0x84, 0x8A, 0x01, 0x08, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    
    auto large_frames = serial_data_to_can_frames(large_test_data);
    
    // Verify multiple frames
    EXPECT_EQ(large_frames.size(), 2);
    EXPECT_EQ(large_frames[0].can_id & ~CAN_EFF_FLAG, 0x0600);
    EXPECT_EQ(large_frames[1].can_id & ~CAN_EFF_FLAG, 0x0601);
    
    // Convert back and verify
    auto recovered_large_data = can_frames_to_serial_data(large_frames);
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