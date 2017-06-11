//#include "lms/imaging_detection/line_point.h"
//#include "lms/imaging_detection/edge_point.h"
//#include <lms/imaging/draw_debug.h>
#include <cmath>

namespace drive_ros_image_recognition{
namespace detection{



void LinePoint::setParam(const LinePointParam &param){
    m_LinePointParam = param;
}

bool LinePoint::find(const LinePointParam &param){
    setParam(param);
    return find(DRAWDEBUG_ARG_N);
}

bool LinePoint::find(){
    if(m_LinePointParam.findMax){
        return findMaxALongLine();
    }else{
        return findAlongLine();
    }
}

bool LinePoint::findMaxALongLine(){
    SobelArray sa;
   float maxSobel = 0;
   (void)maxSobel;//TODO
    int maxIndex = -1;
    if(!sa.find(m_LinePointParam)){
        //Should never happen
        return false;
    }
    for(uint i = 0; i < sa.sobelVals.size();i++ ){
        SobelArray::SobelVal sv = sa.sobelVals[i];
        //check if it is smaller then the threshold
        float currentSobel = pow(pow(sv.sobelX,2)+pow(sv.sobelY,2),0.5);
        if(currentSobel < m_LinePointParam.sobelThreshold){
            continue;
        }
        //TODO find min and max


        maxIndex = i;
    }
    if(maxIndex == -1)
        return false;
    //TODO set vals
    return true;
}


bool LinePoint::findAlongLine(){

    //try to find first point, if it fails return as no LinePoint can be found
    EdgePoint::EdgePointParam param = m_LinePointParam;
    //the first searchPoint is already set in the params, just need to set the EdgeType
    m_LinePointParam.searchType = EdgePoint::EdgeType::LOW_HIGH;
    //try to find the first edge
    if(!low_high.find(m_LinePointParam)){
        //no edge found :(
        return false;
    }

    //draw the found point
    DRAWCROSS(low_high.x,low_high.y,0,255,0);
    //check if the user is only looking for one edge
    if(m_LinePointParam.edge){
        //the low_high edge was found :)
        return true;
    }

    //set new values for second edge
    param.searchType = EdgePoint::EdgeType::HIGH_LOW;
    param.x = low_high.x;
    param.y = low_high.y;
    param.searchLength = m_LinePointParam.lineWidthMax;
    //TODO use the sobel angle?
    //TODO: Maybe do some error checking on the sobelAngle?
    //param.searchAngle = low_high.sobelNormal();
    /*
    std::cout << "#######################"<<std::endl;
    std::cout <<"vals: " << low_high.sobelX() << " , " <<low_high.sobelY() << " angle: " <<low_high.sobelNormal() << std::endl;
    std::cout << "gausbox:"<<low_high.x -2 << " , " << low_high.y-2<<std::endl;
    for(int y = low_high.y-2;y <= low_high.y +2;y++){
        for(int x = low_high.x -2;x <= low_high.x +2;x++){
            std::cout << std::to_string(*(param.gaussBuffer->data()+param.gaussBuffer->width()*y + x ))<< " , ";
        }
        std::cout << std::endl;
    }

    std::cout<<"LOW_HIGH: "<<low_high.sobelX()<<" , " << low_high.sobelY()<<std::endl;
    */
    //DRAWLINE(low_high.x,low_high.y,low_high.x-100*low_high.sobelX(),low_high.y-100*low_high.sobelY(),255,255,0);
    //TODO we could reduce the search-length
    //param.searchLength = ?

    //draw the tangent/normal of the sobel
    //DRAWLINE(low_high.x,low_high.y,low_high.x+100*cos(low_high.sobelNormal()),low_high.y+100*sin(low_high.sobelNormal()),255,255,0);
    //DRAWLINE(low_high.x,low_high.y,low_high.x+100*cos(low_high.sobelTangent()),low_high.y+100*sin(low_high.sobelTangent()),0,255,255);

    //TODO: Don't know why that doesn't work well! Sobel values are quite bad!
    bool found  = false;
    if(m_LinePointParam.useSobel){
        if(!high_low.find(param)){
            found = true;
        }
    }
    if(!found){
        //high_low edge wasn't found, try to find it in the old searchAngle
        param.searchAngle = m_LinePointParam.searchAngle;
        if(!high_low.find(param)){
            return false;
        }
    }
    //found both low->high and high->low edge!
    DRAWCROSS(high_low.x,high_low.y,255,255,0);

    //check the width of the linePoint is valid
    float _distance = distance();
    if(_distance < m_LinePointParam.lineWidthMin || _distance > m_LinePointParam.lineWidthMax){
        //width is not valid :(
        return false;
    }
    return true;
}

float LinePoint::distance(){
    return low_high.distance(high_low);
}

float LinePoint::getAngle(){
    return atan2(high_low.y-low_high.y,high_low.x-low_high.x);
}

float LinePoint::getSlope(){
    float dx = low_high.x-high_low.x;
    float dy = low_high.y-high_low.y;
    return dy/dx;
}

LinePoint::LinePointParam& LinePoint::param(){
    return m_LinePointParam;
}

const LinePoint::LinePointParam& LinePoint::param() const{
    return m_LinePointParam;
}
int LinePoint::getType() const{
    return LinePoint::TYPE;
}


int LinePoint::getX() const{
    if(param().edge){ //TODO check what type should be found
        return low_high.x;
    }else{
        return (low_high.x + high_low.x)/2;
    }
}
int LinePoint::getY() const{
    if(param().edge){ //TODO check what type should be found
        return low_high.y;
    }else{
        return (low_high.y + high_low.y)/2;
    }
}


} //namespace detection
} //namespace drive_ros_image_recognition
