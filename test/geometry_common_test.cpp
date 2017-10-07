#include <gtest/gtest.h>
#include "drive_ros_image_recognition/geometry_common.h"


// Declare test
TEST(GeometryCommon, moveOrtho)
{
  linestring line1, line2, line3;
  // 2 points on a vertical line (easier to test)
  point_xy p1(56.567, 37.5989);
  point_xy p2(56.567, 1985.4);
  line1.push_back(p1);
  line1.push_back(p2);

  const double dis=78.5569; // distance to move
  const double err=0.0000001; // allowed error

  // move line to the right
  drive_ros_geometry_common::moveOrthogonal(line1, line2, dis);
  EXPECT_NEAR(line1.at(0).x()+dis, line2.at(0).x(), err);
  EXPECT_NEAR(line1.at(1).x()+dis, line2.at(1).x(), err);
  EXPECT_NEAR(line1.at(0).y(), line2.at(0).y(), err);
  EXPECT_NEAR(line1.at(1).y(), line2.at(1).y(), err);

  // move line back to the left
  drive_ros_geometry_common::moveOrthogonal(line2, line3, -dis);
  EXPECT_NEAR(line2.at(0).x()-dis, line3.at(0).x(), err);
  EXPECT_NEAR(line2.at(1).x()-dis, line3.at(1).x(), err);
  EXPECT_NEAR(line2.at(0).y(), line3.at(0).y(), err);
  EXPECT_NEAR(line2.at(1).y(), line3.at(1).y(), err);

  EXPECT_NEAR(line1.at(0).x(), line3.at(0).x(), err);
  EXPECT_NEAR(line1.at(1).x(), line3.at(1).x(), err);
  EXPECT_NEAR(line1.at(0).y(), line3.at(0).y(), err);
  EXPECT_NEAR(line1.at(1).y(), line3.at(1).y(), err);

}
