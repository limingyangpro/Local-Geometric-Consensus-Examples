/*
 * modelLoader.h
 *
 *  Created on: Sep 4, 2014
 *      Author: liming
 */
#ifndef MODELLOADER_H_
#define MODELLOADER_H_

#include <osgDB/ReadFile>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <numeric>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "boost/algorithm/string/split.hpp"
#include "boost/algorithm/string/classification.hpp"

#include <iostream>
using namespace std;
using boost::property_tree::ptree;

namespace modelloader
{

// trim from start
static inline std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
}

/**
 * Split string using " " (white space) into several float numbers
 * Example:
 * Input: "1  30  2.0"
 * Output: {1.0f,30.0f,2.0f}
 */
template<typename RealType>
bool splitStringtoFloat(const string input, vector<RealType>& result, int length)
{
	using namespace boost::algorithm;
	vector< string > splitString;
	string trimedinput = input;
	trim(trimedinput);
	//String can contain whitespaces and enters
	split( splitString, trimedinput, is_any_of("\t\n "), token_compress_on );

	if (splitString.size() != length)
	{
		cout<<"Split "<<input<<" into "<<length<<" fails !"<<endl;
		return false;
	}
	else
	{
		result.clear();
		for (int i = 0 ; i < splitString.size() ; i++)
		{
			result.push_back(atof (splitString[i].c_str()));
		}
		return true;
	}
}

/**
 * Get a openCV mat with input float vector. Filling horizontally.
 *
 * Example:
 * input = 1 2 3 4 5 6 7 8 9
 * Output = 1 2 3
 *          4 5 6
 *          7 8 9
 */
bool vector2Mat(const vector<float>& input, cv::Mat_<float> &output, int rows, int cols);

class Object3D
{
public:
	osg::ref_ptr<osg::Node> model;
	osg::Matrixf osgMatrix;
	string fileName;
	float scale;               //Scale of the model
	vector<float> transformation16;      //Orientation and position of the model
	Object3D() : model(NULL), scale(1)
	{
	}

	bool load3DObject(const ptree &pt);
	void printInfo(ostream &outstream);
};

class ModelPoints
{
public:
	/**
	 * reference points
	 */
	string fileName;
	vector<cv::Point2f> points;
	vector<cv::Point2f> border;

	ModelPoints() {};
	bool readModelPtsFromFileAndNormalize(string iFile);
	bool readModelPtsFromFile(string iFile, vector<double>& offset, float scale);
	void printInfo(ostream &outstream);
};

class Model
{
public:
	ModelPoints modelpoints;
	vector<Object3D> objects;

	Model() {};
	bool loadModel(const ptree &pt);
	bool load(string filename);
	void printInfo(ostream &outstream);
};

class ModelLoader
{
public:
	cv::Mat_<float> camProjection;
	cv::Mat_<float> camDistortion;
	std::vector< Model > modelArray;
	struct InputVideo
	{
		string streamName;
		int width;
		int height;
	} inputVideo;

private:
	/**
	 * read camera parameter from calibration result of opencv
	 */
	bool readCameraParameters(string parameterfile);

	/**
	 * read camera parameter from a xml node
	 */
	bool readCameraParameters(const ptree &pt);

	/**
	 * Read video configuration from a xml node
	 */
	bool readVideoParameters(const ptree &pt);

public:
	ModelLoader();
	bool readDataFrom(string inputfile);
	void printInfo(ostream &outstream);

};


}

#endif /* MODELLOADER_H_ */
