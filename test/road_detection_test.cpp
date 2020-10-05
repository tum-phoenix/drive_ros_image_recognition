#include <gtest/gtest.h>
#include "drive_ros_image_recognition/road_detection.h"


// Declare test
TEST(RoadDetection, testCase0)
{
  EXPECT_EQ(1,1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
