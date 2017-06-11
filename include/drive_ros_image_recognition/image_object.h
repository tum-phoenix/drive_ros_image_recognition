#ifndef IMAGE_OBJECT_H
#define IMAGE_OBJECT_H

namespace drive_ros_image_recognition {
namespace detection {
class ImageObject{
public:
    /**
     * @brief getType
     * @return the type of the ImageObject
     */
    virtual int getType() const = 0;

    /**
     * @brief Start searching for an edge.
     * @return true if an edge of the specified type and the minimum threshold
     * is found, otherwise false
     */
    virtual bool find() = 0;
};
}
}

#endif //IMAGE_OBJECT_H
