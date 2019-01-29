/*
 * polynom.hpp
 *
 *  Created on: 25.11.2018
 *      Author: simon
 */

#ifndef DRIVE_ROS_CONFIG_MODULES_DRIVE_ROS_IMAGE_RECOGNITION_INCLUDE_DRIVE_ROS_IMAGE_RECOGNITION_POLYNOM_HPP_
#define DRIVE_ROS_CONFIG_MODULES_DRIVE_ROS_IMAGE_RECOGNITION_INCLUDE_DRIVE_ROS_IMAGE_RECOGNITION_POLYNOM_HPP_

#include <opencv2/core/types.hpp>
#include <vector>

class Polynom {
	int order;
	std::vector<float> coeffs;

public:

	Polynom()
	: order(0)
	, coeffs(1, 0.f)
	{

	}

	Polynom(std::vector<float> &coefficients)
	: order(coefficients.size() - 1)
	, coeffs(coefficients)
	{

	}

	Polynom(int polyOrder, std::vector<cv::Point2f> &points)
	: order(polyOrder)
	{
		cv::Mat srcX(points.size(), 1, CV_32FC1);
		cv::Mat srcY(points.size(), 1, CV_32FC1);
		cv::Mat dst(order+1, 1, CV_32FC1);

		// fill the src mats
		for(int i = 0; i < points.size(); i++) {
			srcX.at<float>(i, 0) = points.at(i).x;
			srcY.at<float>(i, 0) = points.at(i).y;
		}

		// TODO: if we use this, we have to cite it: https://github.com/kipr/opencv/blob/master/modules/contrib/src/polyfit.cpp

		CV_Assert((srcX.rows>0)&&(srcY.rows>0)&&(srcX.cols==1)&&(srcY.cols==1)
				&&(dst.cols==1)&&(dst.rows==(order+1))&&(order>=1));

		cv::Mat X;
		X = cv::Mat::zeros(srcX.rows, order+1,CV_32FC1);
		cv::Mat copy;
		for(int i = 0; i <= order; i++)
		{
			copy = srcX.clone();
			cv::pow(copy,i,copy);
			cv::Mat M1 = X.col(i);
			copy.col(0).copyTo(M1);
		}
		cv::Mat X_t, X_inv;
		transpose(X,X_t);
		cv::Mat temp = X_t*X;
		cv::Mat temp2;
		invert (temp,temp2);
		cv::Mat temp3 = temp2*X_t;
		cv::Mat W = temp3*srcY;
		W.copyTo(dst);

		coeffs.resize(order + 1);
		for(int i = 0; i <= order; i++) {
			coeffs.at(i) = dst.at<float>(i,0);
		}

	}

	float atX(float x) const {
		float result = 0.f;

		for(int i = 0; i <= order; i++) {
			float tmp = coeffs.at(i);
			for(int j = 0; j < i; j++) {
				tmp *= x;
			}
			result += tmp;
		}

		return result;
	}

	float getFstDeviationAtX(float x) const {
		float result = 0.f;
		for(int i = 1; i <= order; i++) {
			float tmp = coeffs.at(i) * i;
			for(int j = 1; j < i; j++) {
				tmp *= x;
			}
			result += tmp;
		}

		return result;
	}

	inline int getOrder() { return order; }
	inline std::vector<float> getCoeffs() { return coeffs; }

};


#endif /* DRIVE_ROS_CONFIG_MODULES_DRIVE_ROS_IMAGE_RECOGNITION_INCLUDE_DRIVE_ROS_IMAGE_RECOGNITION_POLYNOM_HPP_ */
