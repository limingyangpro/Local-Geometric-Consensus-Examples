/*
 * imagefix.cpp
 *
 *  Created on: Jan 23, 2015
 *      Author: liming
 */

#include <iostream>
#include <dirent.h>

#include "../src/include/matchinginterface.hpp"
#include "../io/detectfromvideo.hpp"
#include "../io/detector.hpp"

using namespace std;

/**
 * Three difference methods with their results: RDM, LGC, SURF
 */
int method;
DetectFromVideo *inputVideo;
Homography2DwithMultiModel *algoLGC;
vector< cv::Mat_<float> > homo_vector;

/**
 * Model point sets (referencepoint) and scene point set (scenepoint)
 */
vector<cv::Point2f> referencepoint, scenepoint;
vector<cv::Point2f> referenceborders, imageborders;

/**
 * Statistics on matching speed for three methods
 * sumAllFrame: sum of matching speed across all frames
 * sumSquareFrame: sum of squared matching speed across all frames
 * sumAllModel: sume of registration speed across all registered models
 */
Chronometer timer;
float sumAllFrame_time;
float sumAllModel_time;
float time_lgc, time_ordm, time_surf, time_common;
int numFrame = 0;
int numModel = 0;

bool readModelPtsFromFile(string iFile, vector<cv::Point2f> &points, vector<cv::Point2f> &border, vector<double>& offset, float scale = 1.0)
{
	string fileName = iFile;
	ifstream file(fileName.c_str());
	if (!file.is_open())         //Check if the corresponding file exists
	{
		std::cout<<"Fail to open file "<< fileName << std::endl;
		return false;
	}

	// Deal with all the coordinates

	int totalpoints;
	file >> totalpoints;

	vector< double > tmp_x(totalpoints), tmp_y(totalpoints);

	for(int i=0;i<totalpoints;i++)
	{
		file >> tmp_x[i] >> tmp_y[i];
	}
	file.close();

	//Move point sets with their mass center at "offset"
	std::transform(tmp_x.begin(), tmp_x.end(), tmp_x.begin(), std::bind2nd(std::minus<double>(), offset[0]));
	std::transform(tmp_y.begin(), tmp_y.end(), tmp_y.begin(), std::bind2nd(std::minus<double>(), offset[1]));

	//Record the result
	points.clear();
	for(int i = 0 ; i < totalpoints ; i++)
	{
		points.push_back(cv::Point2f(tmp_x[i]*scale, tmp_y[i]*scale));
	}

	//Find the border
	border.resize(4);
	//min corner
	border[0].x = (*std::min_element(tmp_x.begin(),tmp_x.end()))*scale;
	border[0].y = (*std::min_element(tmp_y.begin(),tmp_y.end()))*scale;
	//max corner
	border[2].x = (*std::max_element(tmp_x.begin(),tmp_x.end()))*scale;
	border[2].y = (*std::max_element(tmp_y.begin(),tmp_y.end()))*scale;
	//other corner
	border[1].x = border[0].x; border[1].y = border[2].y;
	border[3].x = border[2].x; border[3].y = border[0].y;

	return true;
}

/**
 * Draw reference points as well as detected point on images
 */
cv::Mat drawOpenCV(const cv::Mat &fond,
                   const vector<cv::Point2f>  &backgrdptarray,
                   const vector<cv::Point2f>  &refptarrays,
				   const vector<cv::Point2f>  &refborder,
				   const cv::Mat &homo)
{

   cv::Mat resultFrame = fond.clone();

   unsigned int listnum,i,j;

   //Invert coordinate y
   int testInversion = resultFrame.rows;

   // Show Img and detected points
	for (j = 0 ; j < backgrdptarray.size() ; j++)
	{
		cv::Point2f temp = backgrdptarray[j];
		cv::circle(resultFrame, cv::Point(temp.x, temp.y), 4, cv::Scalar(255,0,0),3);
	}

   // Drawing in red
	// Show Img and Points OpenCV
   if ( !homo.empty() )
	{
      // Can't draw dotted line ine OpenCV easily :/
      std::vector<cv::Point2f> inputsBorder = refborder;
      std::vector<cv::Point2f> outputsBorder;
      cv::perspectiveTransform(inputsBorder, outputsBorder, homo);
      for (j = 0 ; j < outputsBorder.size() ; j++)
      {
    	  outputsBorder[j].y = fond.rows - outputsBorder[j].y ;
	  }

      // Drawing border line
      // WARNING PIXELS ARE STORED in BGR order and not RGB
      cv::line(resultFrame, cv::Point(outputsBorder[0].x, outputsBorder[0].y), cv::Point(outputsBorder[1].x, outputsBorder[1].y), cv::Scalar(0,0,255),1);
      cv::line(resultFrame, cv::Point(outputsBorder[1].x, outputsBorder[1].y), cv::Point(outputsBorder[2].x, outputsBorder[2].y), cv::Scalar(0,0,255),1);
      cv::line(resultFrame, cv::Point(outputsBorder[2].x, outputsBorder[2].y), cv::Point(outputsBorder[3].x, outputsBorder[3].y), cv::Scalar(0,0,255),1);
      cv::line(resultFrame, cv::Point(outputsBorder[0].x, outputsBorder[0].y), cv::Point(outputsBorder[3].x, outputsBorder[3].y), cv::Scalar(0,0,255),1);
      cv::line(resultFrame, cv::Point(outputsBorder[0].x, outputsBorder[0].y), cv::Point(outputsBorder[2].x, outputsBorder[2].y), cv::Scalar(0,0,255),1);
      cv::line(resultFrame, cv::Point(outputsBorder[1].x, outputsBorder[1].y), cv::Point(outputsBorder[3].x, outputsBorder[3].y), cv::Scalar(0,0,255),1);


      //Draw points
      std::vector<cv::Point2f> projectedreferencepoints;
      cv::perspectiveTransform(refptarrays, projectedreferencepoints, homo);

      for (j = 0 ; j < projectedreferencepoints.size() ; j++)
      {
		  cv::Point2f temp = projectedreferencepoints.at(j);
		  cv::circle(resultFrame, cv::Point(temp.x, fond.rows - temp.y), 2, cv::Scalar(0,0,255), 2);
	  }

	}

   return resultFrame;
}

// main function
int main(int argc, char **argv)
{

	if (setenv ("DISPLAY", ":0", 0) == -1) return -1;

    int videoWidth = 640;
    int videoHeight = 480;
    string videoname = "";
    string referencefile = "";

	char oc;
	while((oc = getopt(argc, argv, "?v:r:w:h:")) != -1)
	{
		switch (oc)
		{
		case '?':
			cout << "Help of utility : "<<endl
			<< left <<setw(6)<< "-r : reference file name" <<endl
			<< left <<setw(6)<< "-v : video name" <<endl
			<< left <<setw(6)<< "-w : width of the window (not work for linux)" <<endl
			<< left <<setw(6)<< "-h : height of the window (not work for linux)" <<endl
			<< "For Example : opencvexample -v 0 -r A4Pattern.txt -w 800 -h 600"<<endl
			<< "Explain : read from camera number 0 and find point pattern described in A4Pattern.txt, the size of the window will be 800*600"<<endl
			<< endl;
			return 0;
		case 'v':
			videoname = string(optarg);
			break;
		case 'r':
			referencefile = string(optarg);
			break;
		case 'w':
			videoWidth = atoi(optarg);
			break;
		case 'h':
			videoHeight = atoi(optarg);
			break;
		default :
			cout<<"Incorrect Argument!! "<<endl;
			return 0;
		}
	}

	/**
	 * Initialize for all methods
	 */
	vector<double> offset(2);
	offset[0] = 0;
	offset[1] = 0;
	readModelPtsFromFile(referencefile, referencepoint, referenceborders, offset);
	algoLGC = new LGC();

	algoLGC->addModel(referencepoint);

	/**
	 * Initialize video sequence
	 */
	cv::VideoCapture vCapture;



	char * p ;
	int camID = strtol(videoname.c_str(), &p, 10 );

	if (*p == 0) //mloader.inputVideo is an integer
	{
		   if ( !vCapture.open(camID))
		   {
			   std::cout<<"There is no input video stream from camera "<<camID<<"!"<<std::endl;
			   return EXIT_FAILURE;
		   }
	}
	else
	{
		   if ( !vCapture.open(videoname))
		   {
			   std::cout<<"There is no input video stream from file "<<videoname<<std::endl;
			   return EXIT_FAILURE;
		   }
	}

	//Set video width and height
	cout << "Open Video stream from " << videoname << " with resolution "<<videoWidth<<"x"<<videoHeight<<endl;
	vCapture.set(CV_CAP_PROP_FRAME_WIDTH,videoWidth);
	vCapture.set(CV_CAP_PROP_FRAME_HEIGHT,videoHeight);

	cv::Mat firstFrame;
	vCapture >> firstFrame;
	/** This is for linux */
	cout<<"The actual window size is "<<firstFrame.cols<<"x"<<firstFrame.rows<<endl;
	/** End for linux */

	SimplePointDetector kpdetector;
	inputVideo = new DetectFromVideo(&kpdetector, vCapture, false);


	/**
	 * Create result window
	 */
	string windowname = "Result window";
	cv::namedWindow(windowname, 1);
	cv::Mat resultImage;

	/**
	 * Initialize statistics
	 */
	sumAllFrame_time = 0; sumAllModel_time = 0;
	vector< int > modelID_vector;   //model ID found by each method

	modelID_vector.clear();
	homo_vector.clear();

	while (inputVideo->nextImage())
	{
		/**
		 * Match according to the chosen method
		 */
		numFrame++;

		timer.tic();
		algoLGC->findModelsandHomography2Ds(*inputVideo->getInvertedPoints(), modelID_vector, homo_vector);
		time_common = timer.tac();
		sumAllFrame_time += time_common;

		/**
		 * Drawing results
		 */
		cv::Mat input;
		std::stringstream ss;
		{
			input = inputVideo->getImage()->clone();
			//Draw detected points

			if (modelID_vector.size() >= 1)
			{
				resultImage = drawOpenCV(input, *inputVideo->getPoints(), referencepoint, referenceborders, homo_vector[0]);
			}
			else
			{
				resultImage = drawOpenCV(input, *inputVideo->getPoints(), referencepoint, referenceborders, cv::Mat());
			}

			if (modelID_vector.size() == 0)
			{
				ss << "No matching";
			}
			else
			{
				ss << "Found model ";
			}
			putText(resultImage, ss.str(), cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cvScalar(0,0,0), 1, CV_AA);
		}

		cv::imshow(windowname, resultImage);
		char key = cv::waitKey(10);
		if ((int)key == 27)
		{
			cout<<"Program exit under user's command."<<endl;
			exit(0);
		}

	} ;

}



