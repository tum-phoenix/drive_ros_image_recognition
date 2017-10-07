#include <gtest/gtest.h>
#include "drive_ros_image_recognition/common_image_operations.h"


// Declare test
TEST(TestSuite, testCase0)
{
  EXPECT_EQ(1,1);
}

// Declare test
TEST(TestSuite, testCase1)
{
  EXPECT_EQ(1,0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
