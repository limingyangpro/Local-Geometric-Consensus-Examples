/*
 * drawopencv.h
 *
 *  Created on: Apr 10, 2015
 *      Author: liming
 */

#ifndef OTHERLIB_DRAWOPENCV_H_
#define OTHERLIB_DRAWOPENCV_H_

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
using namespace std;

cv::Mat drawOpenCV(const cv::Mat* fond,
        const vector< vector<cv::Point2f> >* backgrdptarrays,
        const cv::Mat* refimg,
        const vector< vector<cv::Point2f> >* refptarrays,
        const vector< pair<int , pair<int,int > > > *corresp,
		const cv::Mat homo);



#endif /* OTHERLIB_DRAWOPENCV_H_ */
