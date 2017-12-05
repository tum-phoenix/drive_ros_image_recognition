#ifndef LINE_H
#define LINE_H

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
  cv::Point iP1_, iP2_, iMid_;
  cv::Point2f wP1_, wP2_, wMid_;

  Line(cv::Point iP1, cv::Point iP2, cv::Point2f wP1, cv::Point2f wP2)
//    : iP1_(iP1)
//    , iP2_(iP2)
//    , wP1_(wP1)
//    , wP2_(wP2)
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
    auto xDir = wP2_.x - wP1_.x;
    auto yDir = wP2_.y - wP1_.y;
    return sqrt(xDir * xDir + yDir * yDir);
  }
};

inline float getDistance(const cv::Point2f a, const cv::Point2f b) {
  auto dX = a.x - b.x;
  auto dY = a.y - b.y;
  return sqrt(dX * dX + dY * dY);
}

#endif // LINE_H
