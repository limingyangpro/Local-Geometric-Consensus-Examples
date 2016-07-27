/*
 * detector.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: liming
 */


#include "detector.hpp"

bool cleanup(vector<cv::Point2f>& inputoutputpoints, float thres)    //Clean up interest points
{
	std::cout << "Before cleaning intersections -- Size = " << inputoutputpoints.size() << std::endl;

	// Distances
	float d;
	cv::Point2f tmppt;
	int counter;

	for (unsigned int i = 0; i < inputoutputpoints.size(); ++i)
	{
		// Removing points too close from one another
		tmppt = inputoutputpoints[i];
		counter = 1;
		for(unsigned int j=i+1; j < inputoutputpoints.size();)
		{
			d = cv::norm(inputoutputpoints[i]-inputoutputpoints[j]);

	        if(d < thres)
	        {
	        	tmppt += inputoutputpoints[j];
	        	counter++;
	        	inputoutputpoints.erase(inputoutputpoints.begin()+j);
	        }
	        else
	            j++;
	     }
	  }

	  std::cout << "After cleaning intersections -- Size = " << inputoutputpoints.size() << std::endl;
}

void revert_y(vector<cv::Point2f>& ioKPoints, int rownum)
{
	for (unsigned int i = 0 ; i < ioKPoints.size() ; i++)
	{
		ioKPoints[i].y = rownum - ioKPoints[i].y;
	}
}

void adjustContrast(cv::Mat_<uchar> &inputouput)
{
	cv::Mat_<float> whitefloat, adjustedfloat;
	cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 11, 11 ));

	cv::Mat_<float> grayfloat = inputouput;
	cv::morphologyEx(grayfloat, whitefloat, cv::MORPH_CLOSE, element);

	cv::divide(grayfloat, whitefloat, adjustedfloat);

	cv::normalize(adjustedfloat,adjustedfloat,0,255,cv::NORM_MINMAX);
	inputouput = adjustedfloat;
}

bool AbstractKPDetector::detect(const cv::Mat &iSrc, vector<cv::Point2f>& oKPoints) const
{
//	cout<<"Do not use abstract method"<<endl;
	return true;
};

SimplePointDetector::SimplePointDetector()
{
	detectorparams.filterByColor = true;        //Find black points
	detectorparams.blobColor = 0;

	detectorparams.minDistBetweenBlobs = 0.0;  // minimum 10 pixels between blobs

	detectorparams.filterByArea = true;         // filter  by area of blob
	detectorparams.minArea = 5.0;              // min 5 pixels squared
	detectorparams.maxArea = 200.0;             // max 200 pixels squared

	//Without specifying "minConvexity, some floued points can be missed
	detectorparams.filterByConvexity = true;         // filter  by area of blob
	detectorparams.minConvexity = 0.5;              // min 5 pixels squared

	blobDetector = cv::SimpleBlobDetector::create(detectorparams);
//	blobDetector = new cv::SimpleBlobDetector(detectorparams);
}

SimplePointDetector::SimplePointDetector(int blobcolor, int minArea, int maxArea)
{
	detectorparams.filterByColor = true;        //Find black points
	detectorparams.blobColor = blobcolor;

	detectorparams.minDistBetweenBlobs = 0.0;  // minimum 10 pixels between blobs

	detectorparams.filterByArea = true;         // filter  by area of blob
	detectorparams.minArea = minArea;              // min 5 pixels squared
	detectorparams.maxArea = maxArea;             // max 200 pixels squared

	//Without specifying "minConvexity, some floued points can be missed
	detectorparams.filterByConvexity = true;         // filter  by area of blob
	detectorparams.minConvexity = 0.5;              // min 5 pixels squared

//	blobDetector = new cv::SimpleBlobDetector(detectorparams);
	blobDetector = cv::SimpleBlobDetector::create(detectorparams);
}

SimplePointDetector::~SimplePointDetector()
{
//	delete blobDetector;
}

bool SimplePointDetector::detect(const cv::Mat &iSrc, vector<cv::Point2f>& oKPoints) const
{
	cv::Mat_<uchar> grayimg;
	cv::cvtColor( iSrc, grayimg, CV_BGR2GRAY );
	adjustContrast(grayimg);

	threshold(grayimg,grayimg,180,255,cv::THRESH_BINARY);
	cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 3, 3 ));
	cv::morphologyEx(grayimg, grayimg, cv::MORPH_CLOSE, element);

	vector<cv::KeyPoint> detectedBlobs;
	blobDetector->detect(grayimg, detectedBlobs);
	//Transfer the detected points to pointarray
	int i,num;
	num = detectedBlobs.size();
	oKPoints.resize(num);
	for(i = 0; i < num ; i++ )
	{
		oKPoints[i].x = detectedBlobs[i].pt.x;
		oKPoints[i].y = detectedBlobs[i].pt.y;
	}

	return AbstractKPDetector::detect(iSrc, oKPoints);
}

GoodFeatureDetector::GoodFeatureDetector()
: maxCorners(200),
  qualityLevel(0.13),
  minDistance(10),
  blockSize(12)
{
}

GoodFeatureDetector::GoodFeatureDetector(int _maxCorners, double _qualityLevel , double _minDist, int _blockSize)
: maxCorners(_maxCorners),
  qualityLevel(_qualityLevel),
  minDistance(_minDist),
  blockSize(_blockSize)
{

}

GoodFeatureDetector::~GoodFeatureDetector()
{

}

bool GoodFeatureDetector::detect(const cv::Mat &iSrc, vector<cv::Point2f>& oKPoints) const
{
	cv::Mat src_gray;
	cv::cvtColor( iSrc, src_gray, CV_BGR2GRAY );
	GaussianBlur( src_gray, src_gray, cv::Size(3*2+1,3*2+1), 0, 0, cv::BORDER_DEFAULT );

	// Apply corner detection
	goodFeaturesToTrack( src_gray,
					   oKPoints,
	                   maxCorners,
	                   qualityLevel,
					   minDistance,
	                   cv::Mat(),
	                   blockSize,
					   false,
	                   0.04 );


	// Draw corners detected
	cout<<"** Number of corners detected: "<<oKPoints.size()<<endl;
	return AbstractKPDetector::detect(iSrc, oKPoints);
}

IntersectionDetector::IntersectionDetector()
{
	minDist = 5;
}

IntersectionDetector::IntersectionDetector(int _minDist)
{
	minDist = _minDist;
}

IntersectionDetector::~IntersectionDetector()
{

}

bool IntersectionDetector::detect(const cv::Mat &iSrc, vector<cv::Point2f>& oIntersections) const
{

}

bool IntersectionDetector::detectFromInBinaryImage(const cv::Mat_<uchar> &iBinary_image, vector<cv::Point2f>& oIntersections) const
{
	ScaledDetector sDetect(iBinary_image, 4, 1);
	sDetect.detectIntersectionInCurrentScale();

	//sDetect.drawKeypoints();
	sDetect.findKeypointsInCurrentScale(oIntersections);
	return false;
}


DrawingIntersectionDetector::DrawingIntersectionDetector() : IntersectionDetector(), blurSize(3), adaptiveSize(15)
{

}

DrawingIntersectionDetector::DrawingIntersectionDetector(int _minDist, int _blurSize, int _adaptiveSize) : IntersectionDetector(_minDist), blurSize(_blurSize), adaptiveSize(_adaptiveSize)
{

}

DrawingIntersectionDetector::~DrawingIntersectionDetector()
{

}

int detect_peak(
		const cv::Mat_<float> &data,
//        const double*   data, /* the data */
//        int             data_count, /* row count of data */
		vector<int> &emi_peaks,
//        int*            emi_peaks, /* emission peaks will be put here */
//        int*            num_emi_peaks, /* number of emission peaks found */
//        int             max_emi_peaks, /* maximum number of emission peaks */
        vector<int>  &absop_peaks, /* absorption peaks will be put here */
//        int*            num_absop_peaks, /* number of absorption peaks found */
//        int             max_absop_peaks, /* maximum number of absorption peaks   */
        int          delta, /* delta used for distinguishing peaks */
        bool             emi_first /* should we search emission peak first of
                                     absorption peak first? */
        )
{
    int     i;
    double  mx;
    double  mn;
    int     mx_pos = 0;
    int     mn_pos = 0;
    bool     is_detecting_emi = emi_first;


    mx = data.at<float>(0);
    mn = data.at<float>(0);

    for(i = 1; i < data.rows; ++i)
    {
        if(data.at<float>(i) > mx)
        {
            mx_pos = i;
            mx = data.at<float>(i);
        }
        if(data.at<float>(i) < mn)
        {
            mn_pos = i;
            mn = data.at<float>(i);
        }

        if(is_detecting_emi &&
                data.at<float>(i) < mx - delta)
        {
        	emi_peaks.push_back(mx_pos);

            is_detecting_emi = false;

            i = mx_pos - 1;

            mn = data.at<float>(mx_pos);
            mn_pos = mx_pos;
        }
        else if((!is_detecting_emi) &&
                data.at<float>(i) > mn + delta)
        {
        	absop_peaks.push_back(mn_pos);

            is_detecting_emi = true;

            i = mn_pos - 1;

            mx = data.at<float>(mn_pos);
            mx_pos = mn_pos;
        }
    }

    return 0;
}

/**
 * Replacement for Matlab's bwareaopen()
 * Input image must be 8 bits, 1 channel, black and white (objects)
 * with values 0 and 255 respectively
 */
void removeSmallBlobs(cv::Mat& im, double size)
{
    // Only accept CV_8UC1
    if (im.channels() != 1 || im.type() != CV_8U)
        return;

    // Find all contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(im.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        // Calculate contour area
        double area = cv::contourArea(contours[i]);

        // Remove small objects by drawing the contour with black color
        if (area > 0 && area <= size)
        {
//        	cv::Rect boundRect;
//        	im( boundingRect( cv::Mat(contours[i]) ) ).setTo(0);
        	cv::drawContours(im, contours, i, CV_RGB(0,0,0), -1);
        }

    }
}
bool DrawingIntersectionDetector::detect(const cv::Mat &iSrc, vector<cv::Point2f>& oIntersections) const
{
	cv::Mat segmentationresult, src_gray;
	cv::cvtColor( iSrc, src_gray, CV_BGR2GRAY );

	cv::Mat_<float> whitefloat, adjustedfloat;
	cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 11, 11 ));

	cv::Mat_<float> grayfloat = src_gray;
	cv::morphologyEx(grayfloat, whitefloat, cv::MORPH_CLOSE, element);

	cv::divide(grayfloat, whitefloat, adjustedfloat);

	cv::normalize(adjustedfloat,adjustedfloat,0,255,cv::NORM_MINMAX);
	cv::Mat_<uchar> adjusted = adjustedfloat;
	threshold(adjusted,segmentationresult,180,255,cv::THRESH_BINARY);

//	adaptiveThreshold(src_gray, segmentationresult, 255, CV_ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 11, 15);

	segmentationresult = 255 - segmentationresult;
	removeSmallBlobs(segmentationresult, 2500);

//	cv::imshow("tempwindow", segmentationresult);
//	cv::waitKey(5);

	return detectFromInBinaryImage(segmentationresult, oIntersections);
}
