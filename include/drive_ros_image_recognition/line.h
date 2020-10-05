#ifndef LINE_H
#define LINE_H

namespace drive_ros_image_recognition {

///
/// \brief The StreetLine class
/// Represents a line, defined by two points.
/// The class saves the two points in image coordinates and world coordinates.
/// The world coordinates are in meters, e.g. 0.5 is 50 cm.
/// wP1 is always closer to the vehicle than wP2.
/// You are responsible that wP1_ is the correct world points for iP1_, same for wP2_ and iP2_.
/// Important: The x and y coordinates switch when converted from image to world or back.
///
class Line {
public:

  cv::Point2f iP1_, iP2_;
  cv::Point2f wP1_, wP2_;

  Line(cv::Point2i iP1, cv::Point2i iP2, cv::Point2f wP1, cv::Point2f wP2)
  {
    if(wP1.x < wP2.x) {
      iP1_ = iP1;
      iP2_ = iP2;
      wP1_ = wP1;
      wP2_ = wP2;
    } else {
      iP1_ = iP2;
      iP2_ = iP1;
      wP1_ = wP2;
      wP2_ = wP1;
    }
  }

  // returns the angle of the lines between -PI and PI
  inline float getAngle() { return atan2(wP2_.y - wP1_.y, wP2_.x - wP1_.x); }

  float getWorldLength() {
    auto xDir = wP2_.x - wP1_.x;
    auto yDir = wP2_.y - wP1_.y;
    return sqrt(xDir * xDir + yDir * yDir);
  }
};
} // namespace drive_ros_image_recognition

#endif // LINE_H
