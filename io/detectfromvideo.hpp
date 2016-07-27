/*
 * readPointsFromVideo.hpp
 *
 *  Created on: Mar 1, 2015
 *      Author: liming
 */

#ifndef DETECTIONLIB_DETECTFROMVIDEO_HPP_
#define DETECTIONLIB_DETECTFROMVIDEO_HPP_

#include "detector.hpp"

class DetectFromVideo
{
private:
	/**
	 * Point array detected from the current frame
	 * (0,0) at top left (opencv coordinate)
	 */
	vector<cv::Point2f> pointArray;

	/**
	 * Inverted point array along Y-direction
	 * (0,0) at bottom left (traditional coordinate)
	 */
	vector<cv::Point2f> invertedPointArray;

	/**
	 * Current frame
	 */
	cv::Mat currentimage;			        /**< current image under process */

	/**
	 * Current frame with detected points
	 */
	cv::Mat currentimageWithDetectedPoints;

	/**
	 * A pointer to the detector for interest point detection
	 */
	const AbstractKPDetector* detectorPtr;

	/**
	 * Input flow of video
	 */
	cv::VideoCapture cam;

	bool recordInputStream;
	/**
	 * Record the input stream into a default file
	 */
	cv::VideoWriter vWriter;

	/**
	 * Detect point from current image
	 */
	void detect();

public:
	/**
	 * Constructor
	 * @param _detectorPtr [Input] Detector for interest point detection
	 */
	DetectFromVideo(AbstractKPDetector* _detectorPtr) : detectorPtr(_detectorPtr), recordInputStream(false){};

	/**
	 * Constructor
	 * @param _detectorPtr [Input] Detector for interest point detection
	 * @param _capture     [Input] Use other existing video capture as input
	 */
	DetectFromVideo(AbstractKPDetector* _detectorPtr, cv::VideoCapture &_capture, bool record) : detectorPtr(_detectorPtr), cam(_capture), recordInputStream(record) {};
	~DetectFromVideo()
	{
		if (vWriter.isOpened())
		{
			vWriter.release();
		}
	};

	/**
	 * Attach the input video file to the instance
	 * @param filename [Input] Video file name
	 * For example: readFromCam("myInputVideo.mp4") attach the video file "myInputVideo.mp4"
	 */
	bool readFrom(string filename);

	/**
	 * Attach the input video file to the instance
	 * @param camID [Input] Camera ID
	 */
	bool readFrom(int camID);

	/**
	 * Load next frame from video file or camera
	 */
	bool nextImage();

	/**
	 * Get current frame
	 * Return: A pointer to current frame
	 */
	const cv::Mat* getImage() const;

	/**
	 * Get current frame, draw detected points in blue
	 */
	const cv::Mat* getImageWithDetectedPoints() const;

	/**
	 * Get detected points on current frame
	 * @return A pointer to detected points
	 */
	const vector<cv::Point2f>* getPoints() const;

	/**
	 * Get inverted detected points
	 * @return A pointer to inverted detected points
	 */
	const vector<cv::Point2f>* getInvertedPoints() const;

	/**
	 * Print detected points on current frame (Debug use)
	 * @param Output device
	 */
	void printInfo(ostream &outstream) const;
};


#endif /* DETECTIONLIB_DETECTFROMVIDEO_HPP_ */
