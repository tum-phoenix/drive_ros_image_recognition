#ifndef EDGE_POINT_H
#define EDGE_POINT_H

#include <cmath>
#include <memory>
//#include "lms/imaging/draw_debug.h"
//#include "lms/imaging/image.h"
//#include "lms/deprecated.h"
//#include "lms/math/vertex.h"
//#include "lms/config.h"
#include "drive_ros_image_recognition/image_object.h"
//#include "lms/imaging_detection/sobel_array.h"
//#include "lms/imaging/image_factory.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace drive_ros_image_recognition{
namespace detection{
/**
 * @brief Find a Low-High edge, High-Low edge or plane, beginning from a
 * starting point and moving to a target point specified by angle and distance
 * relative to the starting point.
 *
 * The sobel angle is computed for every point along the search line to
 * find an adge.
 */

// point2f inheritance to store point as before
class EdgePoint: public ImageObject, public cv::Point2f {//: public lms::math::vertex2f,public ImageObject {
public:
    static constexpr int TYPE = 0;
    enum class EdgeType {LOW_HIGH, HIGH_LOW, PLANE};

    // completely get rid of sobel array stuff
    struct EdgePointParam{ //:public SobelArray::SobelArrayParam{
        EdgePointParam() : x(0), y(0), target(nullptr), searchLength(0),
            searchAngle(0), searchType(EdgeType::PLANE), sobelThreshold(0),
            findMax(false) {
        }

        // replace with dynamic reconfigure or nodehandle params in the future (ROS-native)
        virtual void fromConfig (const EdgePointParam *config) {//(const lms::Config *config){
            // todo: get rid of those params completely, together with sobelArray class, use smarter reconfigure instead
            //SobelArrayParam::fromConfig(config);
//            if(config->hasKey("sobelThreshold"))
//                sobelThreshold = config->get<float>("sobelThreshold",150);
//            if(config->hasKey("findMax"))
//                findMax = config->get<bool>("findMax",false);
            //TODO searchType = config->get<EdgeType>("searchType",EdgeType::PLANE);

        }

        inline EdgePointParam operator=(const EdgePointParam& a) {
                x=a.x;
                y=a.y;
                target.reset(new cv::Mat(*a.target));
                searchLength = a.searchLength;
                searchAngle = a.searchAngle;
                searchType = a.searchType;
                findMax = a.findMax;
                sobelThreshold = a.sobelThreshold;
            }

        int x;
        int y;

        // now using OpenCV images
        std::unique_ptr<cv::Mat> target;
        float searchLength;
        /**
         * @brief searchAngle in rad, don't forget that y is pointing downwards!
         */
        float searchAngle;
        EdgeType searchType;
        int sobelThreshold;
        bool findMax;
    };

    typedef EdgePointParam parameterType;

private:
    EdgePointParam m_searchParam;
    /**
     * @brief m_sobelX < 0 if the pixels on the left are darker
     */
    int m_sobelX;

    /**
     * @brief m_sobelY < 0 if the pixels on the top are darker
     */
    int m_sobelY;
    float m_sobelNormal;
    float m_sobelTangent;
    EdgeType m_type;

    /**
     * @brief setType "calculates" the type high_low/low_high edge
     * @return the found type
     */
    EdgePoint::EdgeType calculateType();

public:
    void setSearchParam(const EdgePointParam &searchParam);
    EdgePointParam &searchParam(){
        return m_searchParam;
    }

    /**
     * @brief Start searching for an edge.
     * @return true if an edge of the specified type and the minimum threshold
     * is found, otherwise false
     */
    // was receiving debug parameter before
    bool find() override;
    bool find(const EdgePointParam &searchParam); //DRAWDEBUG_PARAM);
    int sobelX();
    int sobelY();

    /**
     * @brief sobelAngle
     * @return the angle from -PI to PI
     */
    float sobelTangent();
    float sobelNormal();
    /**
     * @brief findAlongLine starts at x,y and tries to find a sobel that is greater then the threshold
     * @return true if an edge of the specified type was found
     */
    bool findAlongLine();
    /**
      TODO: refactor(not smart at all)
     * @brief findAlongLine starts at x,y and tries to find a sobel that is greater then the threshold and is a local max/min
     * @return true if an edge of the specified type was found
     */
    bool findMaxALongLine();
    /**
     * @brief type
     * @return the type of the EdgePoint
     */
    EdgeType type();
    int getType() const override;
};

} //namespace detection
} //namespace drive_ros_image_recognition

#endif // IMAGE_EDGE_POINT
