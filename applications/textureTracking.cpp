/*
 * imagefix.cpp
 *
 *  Created on: Jan 23, 2015
 *      Author: liming
 */

#include <iostream>
#include <dirent.h>

#include "../io/drawopencv.h"
#include "../io/detectfromvideo.hpp"
#include "../src/include/matchinginterface.hpp"

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
vector< vector<cv::Point2f> > referencepoint, scenepoint;
vector<cv::Point2f> referenceborders, imageborders;

/**
 * Statistics on matching speed for three methods
 * sumAllFrame: sum of matching speed across all frames
 * sumSquareFrame: sum of squared matching speed across all frames
 * sumAllModel: sume of registration speed across all registered models
 */
Chronometer timer;
float sumAllFrame_lgc, sumAllFrame_ordm, sumAllFrame_surf, sumAllFrame_time;
float sumSquareFrame_lgc, sumSquareFrame_ordm, sumSquareFrame_surf;
float sumAllModel_lgc, sumAllModel_ordm, sumAllModel_surf, sumAllModel_time;
float time_lgc, time_ordm, time_surf, time_common;
int numFrame = 0;
int numModel = 0;

ofstream logfile;

/**
 * Registered current detected point set as one model
 */
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	vector<cv::Point2f> *tmppt;
	int num,i;
	vector<pcl::PointXY> pclptarray;

	if  ( event == cv::EVENT_LBUTTONDOWN )
	{
		numModel++;

		timer.tic();
		algoLGC->addModel(*inputVideo->getPoints());
		time_common = timer.tac();
		sumAllModel_time += time_common;
		logfile<<"Model "<<numModel<<" with Frame "<<numFrame<<" "<<time_common<<endl;
	}
}

// main function
int main(int argc, char **argv)
{

	if (setenv ("DISPLAY", ":0", 0) == -1) return -1;

    bool recordVideo = false;
    bool outputFileCreated = false;
    cv::VideoWriter outputVideo;
    cv::VideoWriter caminput;

    string videoname = "";
    string configfile = "";

	char oc;
	while((oc = getopt(argc, argv, "?m:rv:")) != -1)
	{
		switch (oc)
		{
		case 'v':
			videoname = string(optarg);
			break;
		case 'r':
			recordVideo = true;
			break;
		case '?':
			cout << "Help of utility : "<<endl
			<< left <<setw(6)<< "-v : video name" <<endl
			<< left <<setw(6)<< "-r : record input and output sequences as videos" <<endl
			<< "For Example : textureVideo -i texture.mov -r"<<endl
			<< "Explain : read texture.mov and record it"<<endl
			<< endl;
		default :
			cout<<"Incorrect Argument!! "<<endl;
			return 0;
		}
	}

	/**
	 * Initialize for all methods
	 */
    algoLGC = new LGC();


	/**
	 * Initialize video sequence
	 */
	GoodFeatureDetector kpdetector;
	inputVideo = new DetectFromVideo(&kpdetector);
	if (videoname.compare("") == 0)
	{
		inputVideo->readFrom(1);
	}
	else
	{
		inputVideo->readFrom(videoname);
	}
//	inputVideo->printInfo(cout);

	/**
	 * Initialization model and scene borders
	 */
	imageborders.clear();
	referenceborders.resize(4);

	referenceborders[0].x = 0;  referenceborders[0].y = 0;
	referenceborders[1].x = inputVideo->getImage()->cols;  referenceborders[1].y = 0;
	referenceborders[2].x = inputVideo->getImage()->cols;  referenceborders[2].y = inputVideo->getImage()->rows;
	referenceborders[3].x = 0;  referenceborders[3].y = referenceborders[3].y = inputVideo->getImage()->rows;

	/**
	 * Create result window
	 */
	string windowname = "Result window";
	cv::namedWindow(windowname, 1);
	cv::setMouseCallback(windowname, CallBackFunc, NULL);
	cv::Mat resultImage;

	/**
	 * Initialize statistics
	 */
	sumAllFrame_lgc = 0.0; sumAllFrame_ordm = 0.0; sumAllFrame_surf = 0.0; sumAllFrame_time = 0;
	sumAllModel_lgc = 0.0; sumAllModel_ordm = 0.0; sumAllModel_surf = 0.0; sumAllModel_time = 0;

	vector< int > modelID_vector;   //model ID found by each method

	modelID_vector.clear();
	homo_vector.clear();

	do
	{
		/**
		 * Match according to the chosen method
		 */
		numFrame++;

		timer.tic();
		algoLGC->findModelsandHomography2Ds(*inputVideo->getPoints(), modelID_vector, homo_vector);
		time_common = timer.tac();
		sumAllFrame_time += time_common;
		logfile<<"Frame "<<numFrame<<" "<<time_common<<endl;

		float matchingtime = timer.tac();
		cout<<matchingtime<<" ms"<<endl;

		/**
		 * Set model borders and scene point set for drawing
		 */
		scenepoint.clear();
		scenepoint.push_back(imageborders);          //border line, in blue (real one)
		scenepoint.push_back(*inputVideo->getPoints());         //scene points, in blue cross

//		cout<<inputVideo->getPoints()->size()<<endl;
//		for (int i = 0 ; i < inputVideo->getPoints()->size() ; i++)
//		{
//			cout<<(*inputVideo->getPoints())[i].x<<" "<<(*inputVideo->getPoints())[i].y<<endl;
//		}
		referencepoint.clear();
		referencepoint.push_back(referenceborders);          //border line , in red (matching result)

		/**
		 * Drawing results
		 */
		cv::Mat input;
		std::stringstream ss;
		{
			input = inputVideo->getImage()->clone();
			for (unsigned int i = 0 ; i < modelID_vector.size() ; i++)
			{
				resultImage = drawOpenCV(&input, NULL , NULL, &referencepoint, NULL, homo_vector[i]);
				input = resultImage.clone();
			}

			resultImage = drawOpenCV(&input, &scenepoint , NULL, NULL, NULL, cv::Mat::eye(3,3,CV_32F));

			if (modelID_vector.size() == 0)
			{
				ss << "No matching";
			}
			else
			{
				ss << "Found model ";
			}
			putText(resultImage, ss.str(), cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cvScalar(0,0,0), 1, CV_AA);

			ss.str("");
			ss << "Model Registration : "<<sumAllModel_time/numModel<<"ms/model";
			putText(resultImage, ss.str(), cvPoint(30,400), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0,0,0), 1, CV_AA);
			ss.str("");
			ss <<"Matching : "<<sumAllFrame_time/numFrame<<"ms/Frame";
			putText(resultImage, ss.str(), cvPoint(30,430), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0,0,0), 1, CV_AA);
		}

		cv::imshow(windowname, resultImage);
		char key = cv::waitKey(10);
		if ((int)key == 27)
		{
			cout<<"Program exit under user's command."<<endl;
			exit(0);
		}

		/**
		 * Save input sequences and output sequences as videos if "-r" is specified
		 */
	    if(recordVideo) {
	       if(!outputFileCreated) {
	          // Creating video output file
	          if(!outputVideo.open("resultVideo.mpg", CV_FOURCC('P','I','M','1'), 20.0f, resultImage.size(), true)) {
	             std::cout<<"Could not create output video file"<<endl;
	             exit(1);
	          }

	          if(!caminput.open("orignalVideo.mpg", CV_FOURCC('P','I','M','1'), 20.0f, inputVideo->getImage()->size(), true)) {
	             std::cout<<"Could not create input video file"<<endl;
	             exit(1);
	          }
	          std::cout<<"Result video name: "<<"resultVideo.mpg"<<std::endl;
	          std::cout<<"Original video name: "<<"orignalVideo.mpg"<<std::endl;
	          outputFileCreated = true;
	       }
	       outputVideo << resultImage;
	       caminput << inputVideo->getImage()->clone();
	    }
	} while (inputVideo->nextImage());

}



