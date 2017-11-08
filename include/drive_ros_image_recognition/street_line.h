#ifndef STREET_LINE_H
#define STREET_LINE_H

///
/// \brief The StreetLine class
/// Represents a line, defined by two points.
/// The class saves the two points in image coordinates and world coordinates.
/// The world coordinates are in meters, e.g. 0.5 is 50 cm.
/// You are responsible that wP1_ is the correct world points for iP1_, same for wP2_ and iP2_.
///
class StreetLine {
public:
  cv::Point iP1_, iP2_, iMid_;
  cv::Point2f wP1_, wP2_, wMid_;

  StreetLine(cv::Point iP1, cv::Point iP2, cv::Point2f wP1, cv::Point2f wP2)
    : iP1_(iP1)
    , iP2_(iP2)
    , wP1_(wP1)
    , wP2_(wP2)
  {
    iMid_ = (iP1_ + iP2_) * .5;
    wMid_ = (wP1_ + wP2_) * .5;
  }

  // returns the angle of the lines between 0 and PI
  inline double getAngle() { return abs(atan2(iP1_.y - iP2_.y, iP1_.x - iP2_.x)); }
};

#endif // STREET_LINE_H
