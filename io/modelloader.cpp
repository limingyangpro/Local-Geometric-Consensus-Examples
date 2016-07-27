/*
 * modelloader.cpp
 *
 *  Created on: Sep 4, 2014
 *      Author: liming
 */

#include "modelloader.h"
#include <string>

namespace modelloader
{

bool vector2Mat(const vector<float>& input, cv::Mat_<float> &output, int rows, int cols)
{
	if (input.size() != rows*cols) return false;
	output = cv::Mat::zeros(3,3,CV_16S);
	for (int i = 0 ; i < rows ; i++)
	{
		for (int j = 0 ; j < cols ; j++)
		{
			output[i][j] = input[i*cols + j];
		}
	}
	return true;
}

bool Object3D::load3DObject(const ptree &pt)
{
	fileName = pt.get<std::string>("filename", "Nothing");
	model = osgDB::readNodeFile(fileName);
	if (!model)
	{
		cout<<"Load model "<<fileName<<" failed !"<<endl;
		return false;
	}
	scale = pt.get<float>("scale", 1);

	vector<float> floatvector;

	//Read object transformation matrix
	string maxtrixString = pt.get<string>("transformation", "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1");
	if (! splitStringtoFloat(maxtrixString, floatvector, 16)) return false;
	transformation16.swap(floatvector);

	osg::Matrixf scaleMatrix;
	scaleMatrix.makeScale(scale,scale,scale);
	osgMatrix.set(&transformation16[0]);

	//First scale the 3D object to right size, then rotate and translate it.
	osgMatrix.preMult(scaleMatrix);

	return true;
}

void Object3D::printInfo(ostream &outstream)
{
	outstream<<"3D object file: "<<fileName<<endl
			<<"info: "<<model<<endl
			<<"scale:"<<scale<<endl;
	outstream<<"transformation: ";
	for (int i = 0 ; i < transformation16.size() ; i++)
	{
		if (i%4 == 0)
		{
			outstream<<endl;
		}
		outstream<<transformation16[i]<<" ";
	}
	outstream<<endl;

	outstream<<"osg::Matrix: "<<endl;
	for (int r = 0 ; r < 4 ; r++)
	{
		for (int c = 0 ; c < 4 ; c++)
		{
			outstream<<osgMatrix(r,c)<<" ";
		}
		outstream<<endl;
	}
}


/**
 * Read the model points and normalize them, so that
 * 1. Their mass center is at (0,0,0)
 * 2. sqrt( (sum(x^2)+sum(y^2))/sum(n) ) = 1
 */
bool ModelPoints::readModelPtsFromFileAndNormalize(string iFile)
{
	fileName = iFile;
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

	//Move point sets with their mass center at (0,0)
	double sum_x = std::accumulate(tmp_x.begin(), tmp_x.end(), 0.0);
	std::transform(tmp_x.begin(), tmp_x.end(), tmp_x.begin(), std::bind2nd(std::minus<double>(), sum_x/totalpoints));
	double sum_y = std::accumulate(tmp_y.begin(), tmp_y.end(), 0.0);
	std::transform(tmp_y.begin(), tmp_y.end(), tmp_y.begin(), std::bind2nd(std::minus<double>(), sum_y/totalpoints));

	//Find scale
	double sq_sum = std::inner_product(tmp_x.begin(), tmp_x.end(), tmp_x.begin(), 0.0)
					+ std::inner_product(tmp_y.begin(), tmp_y.end(), tmp_y.begin(), 0.0);
	double scale = 1/std::sqrt(sq_sum / totalpoints);

	//Record the result
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
 * iFile : input
 * offset, scale : reference points' coordinate = (read coordinate - offset)*scale
 */

bool ModelPoints::readModelPtsFromFile(string iFile, vector<double>& offset, float scale)
{
	fileName = iFile;
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

void ModelPoints::printInfo(ostream &outstream)
{
	outstream<<"Points read from: "<<fileName<<endl
			<<"    Border points: "<<endl;
	for (int i = 0 ; i < 4 ; i++)
	{
		outstream<<border[i].x<<" "<<border[i].y<<endl;
	}

	outstream<<"Point list: (" <<points.size()<<" points)"<<endl;
	for (int i = 0 ; i < points.size() ; i++)
	{
		outstream<<points[i].x<<" "<<points[i].y<<endl;
	}
}


bool Model::loadModel(const ptree &pt)
{
	//Find reference file name in the ptree
	string referencefilename = pt.get<std::string>("referencefilename","");

	//Read the offset of reference point set and its scale
	vector<double> doublevector;

	//Read reference offset
	string offset = pt.get<string>("offset", "0 0");
	if (! splitStringtoFloat(offset, doublevector, 2)) return false;

	//Read reference scaling
	float scale = pt.get<float>("scale", 1);

	modelpoints.readModelPtsFromFile(referencefilename, doublevector, scale);

	//Find all models and load them
	const ptree &objectlist = pt.get_child("ObjectList");
    Object3D *tmpObj3D;
    for (ptree::const_iterator it = objectlist.begin(); it != objectlist.end(); ++it)
    {
    	cout<<"Read 3D object "<<it->first<<endl;
    	tmpObj3D = new Object3D();
    	if (tmpObj3D->load3DObject(it->second))
    	{
    		objects.push_back(*tmpObj3D);
    	}
    	delete tmpObj3D;
    }

    return true;
}

void Model::printInfo(ostream &outstream)
{
	modelpoints.printInfo(outstream);

	//Print each object
	for (int i = 0 ; i < objects.size() ; i++)
	{
		objects[i].printInfo(outstream);
	}
}

ModelLoader::ModelLoader()
{

}

bool ModelLoader::readCameraParameters(string parameterfile)
{
    cv::FileStorage fs;
    cv::Mat camMat, distortionMat;
    fs.open(parameterfile, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        cerr << "Failed to open " << parameterfile << endl;
        return false;
    }
    else
    {
    	cout << "Read camera parameter from " << parameterfile << endl;
    }
    fs["Camera_Matrix"] >> camMat;
    if (camMat.empty()) fs["camera_matrix"] >> camMat;               //deal with case sensitive
    camMat.convertTo(camProjection,CV_32F);                          // Read cv::Mat

    fs["Distortion_Coefficients"] >> distortionMat;
    if (distortionMat.empty()) fs["distortion_coefficients"] >> distortionMat;
    distortionMat.convertTo(camDistortion,CV_32F);                          // Read cv::Mat
    return true;
}

bool ModelLoader::readCameraParameters(const ptree &pt)
{
    vector<float> tmpresult;
	string projectionMatrix = pt.get<string>("Projection_Matrix", "1 0 0 0 1 0 0 0 1");
	if (! splitStringtoFloat(projectionMatrix, tmpresult, 9)) return false;
	if (! vector2Mat(tmpresult, camProjection, 3, 3)) return false;

	string distortionMatrix = pt.get<string>("Distortion_Matrix", "0 0 0 0 0");
	if (! vector2Mat(tmpresult, camDistortion, 3, 3)) return false;
}

bool ModelLoader::readVideoParameters(const ptree &pt)
{
	inputVideo.streamName = pt.get<string>("Stream_Name", "0");
	inputVideo.width = pt.get<int>("width", 640);
	inputVideo.height = pt.get<int>("height", 480);
}

bool ModelLoader::readDataFrom(string inputxmlfile)
{
    ptree root_pt;
    read_xml(inputxmlfile, root_pt);

    //Load models
    ptree &modellist = root_pt.get_child("ModelList");
    Model *tmpModel;
    for (ptree::const_iterator it = modellist.begin(); it != modellist.end(); ++it)
    {
    	tmpModel = new Model();
    	cout<<"Read model "<<it->first<<endl;
    	if (tmpModel->loadModel(it->second))
    	{
    		modelArray.push_back(*tmpModel);
    	}
    	delete tmpModel;
    }

    //Load camera parameter files
    ptree camera_root_pt;
    string cameraParameterFile = root_pt.get<string>("camera_parameter_file", "");
    if (cameraParameterFile.empty())
    {
    	ptree& camera_pt = root_pt.get_child("camera");
    	readCameraParameters(camera_pt);
    }
    else
    {
    	readCameraParameters(cameraParameterFile);
    }

    ptree& video_pt = root_pt.get_child("video");
    readVideoParameters(video_pt);

    return true;
}

void ModelLoader::printInfo(ostream &outstream)
{
	for (int i = 0 ; i < modelArray.size() ; i++)
	{
		modelArray[i].printInfo(outstream);
	}

	outstream<<"Camera projection matrix: "<<endl
			<<camProjection<<endl
			<<"Camera distortion matrix: "<<endl
			<<camDistortion<<endl;

	cout << "Open Video stream from " << inputVideo.streamName << " with resolution "<<inputVideo.width<<"x"<<inputVideo.height<<endl;
}

}

