// todo: move this to common package, will be used for trajectory as well
#ifndef GEOMETRY_COMMON_H
#define GEOMETRY_COMMON_H
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

namespace trans = boost::geometry::strategy::transform;
namespace bg = boost::geometry;

typedef boost::geometry::model::d2::point_xy<double> point_xy;
typedef boost::geometry::model::linestring<point_xy> linestring;
typedef boost::geometry::model::referring_segment<point_xy> segment;
typedef boost::geometry::model::segment<point_xy> segment_hc;


namespace drive_ros_geometry_common {

const trans::rotate_transformer<boost::geometry::degree, double, 2, 2> orthogonal_transformer(90.0);

template<typename Segment>
struct lengthHelper{
    lengthHelper(double length) : length_(length){}

    inline void operator()(Segment &s){
      double length = boost::geometry::distance(s.first, s.second);
      if(length > 0){
          // normalize
        trans::scale_transformer<double, 2, 2> scale(length_/length);
        boost::geometry::transform(s, s, scale);
//          trans::translate_transformer<double, 2, 2> trans((bg::get<0>(s.second)-bg::get<0>(s.first))*(length_/length),
//                                                           (bg::get<1>(s.second)-bg::get<1>(s.first))*(length_/length));
//          boost::geometry::transform(s.second, s.second, trans);
      }
    }

    double length_;
};

template<typename Segment>
struct translateHelper{
    translateHelper(double offset) : offset_(offset){}

    inline void operator()(Segment &s){
      double length = boost::geometry::distance(s.first, s.second);
      if(length > 0){
          // translate
          segment_hc trans;
          trans::scale_transformer<double, 2, 2> norm_trans(1/length);
          boost::geometry::transform(s, trans, norm_trans);
          trans::translate_transformer<double, 2, 2> translate( (bg::get<0>(trans.second) - bg::get<0>(trans.first))*offset_,
                                                               (bg::get<1>(trans.second) - bg::get<1>(trans.first))*offset_);
          boost::geometry::transform(s, s, translate);
      }
    }

    double offset_;
};

template<typename Segment>
struct orthogonalHelper{
    orthogonalHelper(double distance) : distance_(distance){}

    inline void operator()(Segment &s){
      double length = boost::geometry::distance(s.first, s.second);
      if(length > 0){
          // offset 90 degrees
          segment_hc rot;
          boost::geometry::transform(s, rot, orthogonal_transformer);

          // normalize
          trans::scale_transformer<double, 2, 2> scale(distance_/length);
          boost::geometry::transform(rot, rot, scale);

          // offset original segment
          trans::translate_transformer<double, 2, 2> translate(bg::get<0>(rot.second)-bg::get<0>(rot.first), bg::get<1>(rot.second)-bg::get<1>(rot.first));
          boost::geometry::transform(s,s,translate);
      }
    }

    double distance_;
};

template<typename Segment>
struct rotationalHelper{
    rotationalHelper(double angle) : angle_(angle){}

    inline void operator()(Segment &s){
      trans::rotate_transformer<boost::geometry::degree, double, 2, 2> rot_trans(angle_);
      boost::geometry::transform(s, s, rot_trans);
    }

    double angle_;
};


inline void moveOrthogonal(const linestring &lines_in, linestring &lines_out, double distance) {
  lines_out = lines_in;
  orthogonalHelper<segment> helper(distance);
  bg::for_each_segment(lines_out, helper);
  return ;
}

inline void moveVertical(const linestring &lines_in, linestring& lines_out, double offset) {
  lines_out = lines_in;
  translateHelper<segment> helper(offset);
  bg::for_each_segment(lines_out, helper);
  return ;
}

inline void rescaleLength (linestring &lines_in, double length) {
  lengthHelper<segment> helper(length);
  bg::for_each_segment(lines_in, helper);
  return ;
}

inline void rotate (linestring &lines_in, double angle) {
  rotationalHelper<segment> helper(angle);
  bg::for_each_segment(lines_in, helper);
  return ;
}

// in case we want to change interface -> boost esoterics are annoying
inline void simplify (const linestring &lines_in, linestring &lines_out, double distance ) {
  bg::simplify(lines_in, lines_out, distance);
  return ;
}

} // namespace drive_ros_geometry_common

#endif // GEOMETRY_COMMON_H

