#include <drive_ros_image_recognition/edge_point.h>

namespace drive_ros_image_recognition{
namespace detection{

void EdgePoint::setSearchParam(const EdgePointParam &searchParam){
  m_searchParam = searchParam;
}

// Sets parameter and searches
bool EdgePoint::find(const EdgePointParam &searchParam){ //DRAWDEBUG_PARAM_N){
  setSearchParam(searchParam);
  return find();
}

bool EdgePoint::find(){
  // stored search paramter (=config) determines what sobel looks for (max or not)
  if(m_searchParam.findMax){
    return findMaxALongLine();
  }else{
    return findAlongLine();
  }
}

bool EdgePoint::findAlongLine(){
  // make sobel region size adjustible
  int sobel_region_size = 5;

  //pointer so it can be set in the bresenhamLine-function
  bool found= false;
  //end-points for the bresenham-function
  int xMax = m_searchParam.x+m_searchParam.searchLength*cos(m_searchParam.searchAngle);
  int yMax = m_searchParam.y+m_searchParam.searchLength*sin(m_searchParam.searchAngle);

  // hardcopy image to be able to process it
  cv::Mat img_processed = *m_searchParam.target;

  // sobel entire image (simpler to implement for now, test against convoluting just at image location)
  int ddepth = CV_16S;
  cv::GaussianBlur( img_processed, img_processed, cv::Size(sobel_region_size,sobel_region_size), 0);
  cv::Mat grad_x, grad_y;
  cv::Sobel( img_processed, grad_x, ddepth, 1, 0);
  cv::Sobel( img_processed, grad_y, ddepth, 0, 1);

  // get line iterator for Bresenham and do the exact same
  cv::LineIterator it(*m_searchParam.target,cv::Point2i(m_searchParam.x,m_searchParam.y),cv::Point2i(xMax,yMax));

  for(int i = 0; i < it.count; i++, ++it)
  {
    //check if the point is valid
    // transfer this from sobel array
//    if(m_searchParam.useBlackList){
//      if(m_searchParam.blackList.contains(it.pos().x,it.pos().y)){
//        return true;
//      }
//    }

#ifndef NDEBUG
    // DEBUG DRAW POINT, MARK WITH CIRCLE
    cv::namedWindow("Edge point debug",CV_WINDOW_NORMAL);
    cv::Mat disp = *m_searchParam.target;
    cv::circle(disp, it.pos(),1,cv::Scalar(255));
    cv::imshow("Edge point debug",disp);
#endif
    // convolute at pixel as alternative (as in existing code)
    //        // gaussian blur around the search point (fixed region size)
    //        cv::Rect region(it.pos().x-sobel_region_size, it.pos().x-it.pos().x, sobel_region_size, sobel_region_size);
    //        cv::GaussianBlur(img_processed(region), img_processed(region), cv::Size(0, 0), 5);
    //        // todo: convolute with sobel kernels for x and y at location

    m_sobelX = grad_x.at<int>(it.pos().x,it.pos().y);
    m_sobelY = grad_x.at<int>(it.pos().x,it.pos().y);

    if(pow(m_sobelX,2)+pow(m_sobelY,2) > pow(m_searchParam.sobelThreshold,2)){
      //found an edge
      //set the type
      calculateType();
      if(type() == m_searchParam.searchType){
        //found an edge you were looking for :)
        //calculate the angle

        m_sobelNormal = atan2(m_sobelY,m_sobelX);
        m_sobelTangent = m_sobelNormal;
        //TODO doesnt care about rotation-count of the searchAngle % PI
        if(-M_PI_2 <m_searchParam.searchAngle && m_searchParam.searchAngle < M_PI_2){
          m_sobelTangent-=M_PI_2;
        }else{
          m_sobelTangent+=M_PI_2;
        }
        this->x =it.pos().x;
        this->y =it.pos().y;

        //TODO check if the sobel-angle is in the threshold
        found = true;
        //stop the bresenham
        return found;
      }
    }
  }
  return found;

  //    lms::math::bresenhamLine(m_searchParam.x,m_searchParam.y,xMax,yMax,[this,&found DRAWDEBUG_CAPTURE](int _x, int _y){
  //        //check if points are inside the image
  //        if(_x < 0 || _x > m_searchParam.target->width() || _y < 0 || _y >m_searchParam.target->height())
  //            return false;
  //        //check if the point is valid
  //        if(m_searchParam.useBlackList){
  //            if(m_searchParam.blackList.contains(_x,_y)){
  //                return true;
  //            }
  //        }

  //        //draw debug point
  //        DRAWPOINT(_x,_y,0,0,255);
  //        //gauss surrounding

  //        int xMin = _x-2;
  //        int xMax = _x+2;
  //        int yMin = _y-2;
  //        int yMax = _y+2;
  //        /*
  //         * TODO that could be optimized as the next pixel will be inside the gaussed rectangle!
  //         * That's why we don't need to gauss a rectangle. Gaussing 3 to 5 pixel instead of 9 would ne enough
  //         */
  //        op::gaussBox(*m_searchParam.target,*m_searchParam.gaussBuffer,xMin,yMin,xMax,yMax);
  //        //for 5x5 sobel
  //        //sobel pxl
  //        /*
  //        std::cout<< "MATTTTTTTTTTTTTTT: "<<std::endl;
  //        for(int y = yMin;y <= yMax;y++){
  //            for(int x = xMin;x <= xMax;x++){
  //                std::cout << std::to_string(*(m_searchParam.gaussBuffer->data()+m_searchParam.gaussBuffer->width()*y + x ))<< " , ";
  //            }
  //            std::cout << std::endl;
  //        }
  //        */
  //        //m_sobelX = op::imageOperator(*m_searchParam.gaussBuffer,_x,_y,&op::KERNEL_SOBEL_5_X[0][0],5,5);
  //        //m_sobelY = op::imageOperator(*m_searchParam.gaussBuffer,_x,_y,&op::KERNEL_SOBEL_5_Y[0][0],5,5);
  //        //m_searchParam.sobelThreshold = 1000;
  //        m_sobelX = -op::sobelX(_x,_y,*m_searchParam.gaussBuffer);
  //        m_sobelY = op::sobelY(_x,_y,*m_searchParam.gaussBuffer);

  //        //check if gradient of sobel is big enough
  //        if(pow(m_sobelX,2)+pow(m_sobelY,2) > pow(m_searchParam.sobelThreshold,2)){
  //            //found an edge
  //            //set the type
  //            calculateType();
  //            if(type() == m_searchParam.searchType){
  //                //found an edge you were looking for :)
  //                //calculate the angle

  //                m_sobelNormal = atan2(m_sobelY,m_sobelX);
  //                m_sobelTangent = m_sobelNormal;
  //                //TODO doesnt care about rotation-count of the searchAngle % PI
  //                if(-M_PI_2 <m_searchParam.searchAngle && m_searchParam.searchAngle < M_PI_2){
  //                    m_sobelTangent-=M_PI_2;
  //                }else{
  //                    m_sobelTangent+=M_PI_2;
  //                }
  //                this->x =_x;
  //                this->y =_y;

  //                //TODO check if the sobel-angle is in the threshold
  //                found = true;
  //                //stop the bresenham
  //                return false;
  //            }
  //        }
  //        //continue searching points!
  //        return true;
  //    });
  //    return found;
}

EdgePoint::EdgeType EdgePoint::calculateType() {
    float x2 = cos(m_searchParam.searchAngle);
    float y2 = -sin(m_searchParam.searchAngle); //- wegen nach unten zeigender y-Achse
    float scalar = -sobelX()*x2+sobelY()*y2;

    //std::cout << "SOBEL-VAL"<<pow(pow(m_sobelX,2)+pow(m_sobelY,2),0.5)<< " x,y: " <<sobelX() << " , "<<sobelY() <<std::endl;
    //std::cout <<"scalar: "<<scalar <<std::endl;
    if(scalar > 0){
        m_type = EdgeType::LOW_HIGH;
    }else if(scalar < 0){
        m_type = EdgeType::HIGH_LOW;
    }else{
        m_type = EdgeType::PLANE;
    }
    return m_type;
}


bool EdgePoint::findMaxALongLine(){
  // sobel entire image and check the direction
  //SobelArray sa;
  cv::Mat img_processed = *m_searchParam.target;
  cv::Mat grad_x, grad_y;
  int sobel_region_size = 2;

  int ddepth = CV_16S;
  cv::GaussianBlur( img_processed, img_processed, cv::Size(sobel_region_size,sobel_region_size), 0);
  cv::Sobel( img_processed, grad_x, ddepth, 1, 0);
  cv::Sobel( img_processed, grad_y, ddepth, 0, 1);
  float maxSobel = 0;
  int maxIndex = -1;

  // add normal line iteration
  int xMax = m_searchParam.x+m_searchParam.searchLength*cos(m_searchParam.searchAngle);
  int yMax = m_searchParam.y+m_searchParam.searchLength*sin(m_searchParam.searchAngle);
  //    if(!sa.find(m_searchParam DRAWDEBUG_ARG)){
  //        //Should never happen
  //        return false;
  //    }

  // get line iterator for Bresenham ( completely replaced sobel array)
  cv::LineIterator it(*m_searchParam.target,cv::Point2i(m_searchParam.x,m_searchParam.y),cv::Point2i(xMax,yMax));

//  for(uint i = 0; i < sa.sobelVals.size();i++ ){
  for(int i = 0; i < it.count; i++, ++it)
  {
//    SobelArray::SobelVal sv = sa.sobelVals[i];
    //check if it is smaller then the threshold
    float currentSobel = pow(pow(grad_x.at<int>(it.pos().x,it.pos().y),2)+pow(grad_y.at<int>(it.pos().x,it.pos().y),2),0.5);//pow(pow(sv.sobelX,2)+pow(sv.sobelY,2),0.5);
    if(currentSobel < m_searchParam.sobelThreshold || currentSobel < maxSobel){
      continue;
    }
    //found new maxSobel, have to check the type
    m_sobelX = it.pos().x;
    m_sobelY = it.pos().y;
//    m_sobelX = sv.sobelX;
//    m_sobelY = sv.sobelY;
    if(m_searchParam.searchType !=calculateType()){
      //wrong type
      continue;
    }

    // set max position values directly after finding a new max
    this->x = it.pos().x;
    this->y = it.pos().y;
//    maxIndex = i;
  }
  // do not need this
//  if(maxIndex == -1)
//    return false;

  //set pos etc.
//  x = sa.sobelVals[maxIndex].xPos;
//  y = sa.sobelVals[maxIndex].yPos;
  return true;

}

int EdgePoint::sobelX(){
  return m_sobelX;
}

int EdgePoint::sobelY(){
  return m_sobelY;
}

float EdgePoint::sobelTangent(){
  return m_sobelTangent;
}

float EdgePoint::sobelNormal(){
  return m_sobelNormal;
}

EdgePoint::EdgeType EdgePoint::type(){
  return m_type;
}

int EdgePoint::getType() const{
  return EdgePoint::TYPE;
}

} // namespace detection
} // namespace drive_ros_image_recognition
