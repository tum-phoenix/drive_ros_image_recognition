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
    enum LineType {
        UNKNOWN = 0,
        LEFT_LINE = 1,
        MIDDLE_LINE = 2,
        RIGHT_LINE = 3,
        HORIZONTAL_LEFT_LANE = 4,
        HORIZONTAL_RIGHT_LANE = 5,
        HORIZONTAL_OUTER_LEFT = 6,
        HORIZONTAL_OUTER_RIGHT = 7,
        STOP_LINE = 8,
        START_LINE = 9,
        GUESS = 10
    };

  cv::Point iP1_, iP2_, iMid_;
  cv::Point2f wP1_, wP2_, wMid_;
  LineType lineType_;

  Line(cv::Point iP1, cv::Point iP2, cv::Point2f wP1, cv::Point2f wP2, LineType lineType)
      : lineType_(lineType)
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

    iMid_ = (iP1_ + iP2_) * .5;
    wMid_ = (wP1_ + wP2_) * .5;
  }

  // returns the angle of the lines between 0 and PI
  inline double getAngle() { return std::abs(atan2(wP1_.x - wP2_.x, wP1_.y - wP2_.y)); }

  float getLength() {
//    return getDistanceBetweenPoints(wP2_, wP1_);
    auto xDir = wP2_.x - wP1_.x;
    auto yDir = wP2_.y - wP1_.y;
    return sqrt(xDir * xDir + yDir * yDir);
  }
};
} // namespace drive_ros_image_recognition

#endif // LINE_H
