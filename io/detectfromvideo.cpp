/*
 * detectfromvideo.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: liming
 */

#include "detectfromvideo.hpp"

void DetectFromVideo::detect()
{
//	cv::Mat_<uchar> img_gray;
//	cv::cvtColor( currentimage, img_gray, CV_BGR2GRAY );
//	detectorPtr->detect(img_gray, pointArray);
	detectorPtr->detect(currentimage, pointArray);

	//Draw detected points on current image
	currentimage.copyTo(currentimageWithDetectedPoints);
	for (unsigned int i = 0 ; i < pointArray.size() ; i++)
	{
		circle(currentimageWithDetectedPoints, pointArray[i], 5,  cv::Scalar(255, 0, 0), 1, 8, 0 );
	}

	//Invert detected points, for OGS use
	invertedPointArray = pointArray;
	for (unsigned int i = 0 ; i < pointArray.size() ; i++)
	{
		invertedPointArray[i].y = currentimage.rows - invertedPointArray[i].y;
	}
}

bool DetectFromVideo::readFrom(string filename)
{
	if (!cam.open(filename))
	{
		cout << "Can't connect to video file "<<filename<<" !"<<endl;;
		return false;
	}
	else
	{
		cam >> currentimage;
		detect();
		return true;
	}
}

bool DetectFromVideo::readFrom(int camID)
{
	if (!cam.open(camID))
	{
		cout << "Can't connect to camera # "<<camID<<" !"<<endl;;
		return false;
	}
	else
	{
		cam >> currentimage;
		detect();
		return true;
	}
}

bool DetectFromVideo::nextImage()
{
	pointArray.clear();           //Clear point array for last image;
	cam >> currentimage;
	if (currentimage.empty())
	{
		return false;
	}
	else
	{
		if (recordInputStream)
		{
			if (! vWriter.isOpened())
			{
				vWriter.open("recordedStream.mpg", CV_FOURCC('P','I','M','1'), 20.0f, currentimage.size(), true);
			}
			vWriter << currentimage;
		}
		detect();
		return true;
	}
}

const cv::Mat* DetectFromVideo::getImage() const
{
	return &currentimage;
}

const cv::Mat* DetectFromVideo::getImageWithDetectedPoints() const
{

	return &currentimageWithDetectedPoints;
}

const vector<cv::Point2f>* DetectFromVideo::getPoints() const
{
	return &pointArray;
}

const vector<cv::Point2f>* DetectFromVideo::getInvertedPoints() const
{
	return &invertedPointArray;
}

void DetectFromVideo::printInfo(ostream &outstream) const
{
	int i,num;
	num = pointArray.size();
	cout<<num<<endl;
	for (i = 0 ; i < num ; i++)
	{
		outstream<<pointArray[i].x<<" "<<pointArray[i].y<<endl;
	}
}


