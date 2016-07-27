/*
 * keypointDetector.hpp
 *
 *  Created on: Mar 1, 2015
 *      Author: liming
 */

#ifndef DETECTIONLIB_DETECTOR_HPP_
#define DETECTIONLIB_DETECTOR_HPP_

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "scaleddetector/scaleddetector.hpp"
using namespace std;

bool cleanup(vector<cv::Point2f>& inputoutputpoints, float thres);    //Clean up interest points
void revert_y(vector<cv::Point2f>& ioKPoints, int rownum);

class AbstractKPDetector
{
public:
	/**
	 * Constructor with default parameters
	 */
	AbstractKPDetector() {};
	virtual ~AbstractKPDetector() {};

	/**
	 * Main method for detecting
	 * @param iSrc [Input] Image for detecting
	 * @param oKPoints [Output] An array of detected key points
	 */
	virtual bool detect(const cv::Mat &iSrc, vector<cv::Point2f>& oKPoints) const;
};

class SimplePointDetector : public AbstractKPDetector
{
private:
	cv::SimpleBlobDetector::Params detectorparams;
	cv::Ptr< cv::SimpleBlobDetector > blobDetector;
public:
	SimplePointDetector();

	/**
	 * Parameterized constructor
	 * @param blobcolor [Input] Color of the blob to be detected (0 means to extract black blobs, 1 means to extract white blobs)
	 * @param minArea [Input] Each extracted blob's area should be larger than minArea.
	 * @param maxArea [Input] Each extracted blob's area should be smaller than maxArea.
	 */
	SimplePointDetector(int blobcolor, int minArea, int maxArea);
	~SimplePointDetector();
	bool detect(const cv::Mat &iSrc, vector<cv::Point2f>& oKPoints) const;
};

class GoodFeatureDetector : public AbstractKPDetector
{
private:
	int maxCorners;
	double qualityLevel;
	double minDistance;
	int blockSize;
public:
	GoodFeatureDetector();

	/**
	 * Parameterized constructor
	 * @param _maxCorners [Input] Maximum number of key points to extract.
	 * @param _qualityLevel [Input] See quality level of harris corner detector in openCV
	 * @param _minDist [Input] The distance between any two key points should be larger than _minDist
	 * @param _blockSize [Input] See blocksize in goodFeaturesToTrack in openCV
	 */
	GoodFeatureDetector(int _maxCorners, double _qualityLevel , double _minDist, int _blockSize);
	~GoodFeatureDetector();
	bool detect(const cv::Mat &iSrc, vector<cv::Point2f>& oKPoints) const;
};

class IntersectionDetector : public AbstractKPDetector
{
protected:
	int minDist;

protected:

	/**
	 * Detect intersections from binary image
	 * @param iBinary_image [Input] A binary image which contains network (white) and black background
	 * @param oIntersections [Output] Intersections of the input network
	 */
	bool detectFromInBinaryImage(const cv::Mat_<uchar> &iBinary_image, vector<cv::Point2f>& oIntersections) const;

public:
	IntersectionDetector();

	/**
	 * Parameterized constructor
	 * @param _minDist [Input] The distance between any two intersections should be larger than _minDist
	 */
	IntersectionDetector(int _minDist);
	virtual ~IntersectionDetector();
	virtual bool detect(const cv::Mat &iSrc, vector<cv::Point2f>& oIntersections) const;
};

class DrawingIntersectionDetector : public IntersectionDetector
{
private:
	int blurSize;
	int adaptiveSize;
public:
	DrawingIntersectionDetector();

	/**
	 * Parameterized constructor
	 * @param _minDist [Input] The distance between any two intersections should be larger than _minDist
	 * @param blurSize [Input] Kernel size for Gaussien blur (blurSize*blurSize)
	 * @param adaptiveSize [Input] Windows size for adaptive threshold (adaptiveSize*adaptiveSize)
	 */
	DrawingIntersectionDetector(int _minDist, int blurSize, int adaptiveSize);
	~DrawingIntersectionDetector();
	bool detect(const cv::Mat &iSrc, vector<cv::Point2f>& oIntersections) const;
};

#endif /* DETECTIONLIB_DETECTOR_HPP_ */
