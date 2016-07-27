/*
 * OSG.cpp
 *
 *  Created on: Apr 22, 2015
 *      Author: liming
 *
 *      Structure of scenes:
 *
 *      										 Camera(main)
 *      							     			  |
 *      							     			  |
 *      							   				root
 *               (userdata: estimated homography transformation and ID of the presented model)
 *      							                  |
 *      							     			  |
 *      					------------------------------------------------------
 *      					|					   	                             |
 *      					|						                             |
 *      			Camera (Pre_render)                                          |
 *      			   (CallBack1)                                			     |
 *      			        |													 |
 *      			        |													 |
 *      	   ---------------------------                                  Switch Node
 *      	   |                         |                                  (CallBack3)
 *			   |                         |                                       |
 *    Background Texture          Detected Points								 |
 *																	 --------------------------------------------------
 *																	 |					        |
 *														   Transformation Node 1      Transformation Node 2
 *														        (CallBack2)				   (CallBack2)
 *																	 |					        |
 *																	 |							|
 *																  Model 1                    Model 2
 *																     |
 *																     |
 *														---------------------------
 *														|                         |
 *														|						  |
 *										      Reference Point Node           Object Group
 *										      									  |
 *										      									  |
 *										      						  -----------------------------------------------------------
 *										      						  |							|
 *										      						  |							|
 *										      			    Object Transformation      Object Transformation
 *										      			    		  |                         |
 *										      			    		  |                         |
 *										      			    	  3D Object					3D Object
 *
 *
 *    CallBack functions:
 *    Callback1: UpdateTexture
 *               a> Read new frame from video stream, detect points
 *               b> Find which model presenting in the scene and the homography transformation, write the information in root node (userdata)
 *               c> Redraw Background texture and Detected points
 *
 *    CallBack2: UpdateTransformation
 *               Read homography from root node and update current global transformation matrix
 *    CallBack3: ModelSelector
 *               Read modelID from root node and switch on corresponding node.
 *
 *    userdata:  homoVector a vector of homography of size N, N being the number of models (sub-entry of "ModelList").
 *               If model i is non-visible -> homoVector[i].empty() == true.
 */

#include <osgDB/ReadFile>
#include <osg/Point>
#include <osg/LineWidth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/StateAttribute>
#include <osg/Texture2D>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osg/Notify>
#include <osg/NodeCallback>
#include <osg/AnimationPath>
#include <osg/MatrixTransform>
#include <osg/Camera>

#include <osg/ref_ptr>
#include "osgViewer/Viewer"
#include <osg/Notify>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>

#include "../io/detectfromvideo.hpp"
#include "../io/modelloader.h"

#include "../src/include/matchinginterface.hpp"
#include "../src/include/chronometer.h"
#include <iostream>
#include <string>

modelloader::ModelLoader mloader;
Homography2DwithMultiModel *algoMatching;

/**
 * create geometry for any list of 2D points
 * @param drawType: [Input] An integer indicating how to connect points, such as osg::PrimitiveSet::LINE_LOOP
 * @param pointVector: [Input] An vector contains 2D verteces
 * @param color: [Input] Color of these primitives
 * @return: A created OGS geometry
 */
osg::ref_ptr<osg::Geometry> createPrimitiveGeometry(const int drawType, const vector<cv::Point2f>& pointVector, const osg::Vec4& color)
{
	osg::ref_ptr<osg::Geometry> pointSet (new osg::Geometry());
	osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
	osg::ref_ptr<osg::Vec4Array> overallColor = new osg::Vec4Array;
	overallColor->push_back(color);

	for (unsigned int j = 0 ; j < pointVector.size() ; j++)
	{
		vertices->push_back (osg::Vec3 ( pointVector[j].x, pointVector[j].y, 0.0));
	}

	pointSet->setVertexArray (vertices.get());
	pointSet->addPrimitiveSet(new osg::DrawArrays(drawType,0,vertices->size()));
	pointSet->setColorArray(overallColor.get());
	pointSet->setColorBinding(osg::Geometry::BIND_OVERALL);
	pointSet->getOrCreateStateSet()->setAttribute( new osg::Point( 3.0f ), osg::StateAttribute::ON );

	return (pointSet.get());
}


/**
 * Create geodes for texture
 * @param camWidth: [Input] The width of camera image
 * @param camHeight: [Input] The height of camera image
 * @return: A texture attached node
 */
osg::ref_ptr<osg::Node> createGeodes(float camWidth, float camHeight)
{
	//Vortex of the geometry
    osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
    v->push_back( osg::Vec3( 0, 0, -1 ) );
    v->push_back( osg::Vec3( camWidth, 0, -1 ) );
    v->push_back( osg::Vec3( camWidth, camHeight, -1 ) );
    v->push_back( osg::Vec3( 0, camHeight, -1 ) );

    //Corner of the texture
    osg::ref_ptr<osg::Vec2Array> tc = new osg::Vec2Array;
    tc->push_back( osg::Vec2( 0.f, 0.f ) );
    tc->push_back( osg::Vec2( 1.f, 0.f ) );
    tc->push_back( osg::Vec2( 1.f, 1.f ) );
    tc->push_back( osg::Vec2( 0.f, 1.f ) );

    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
    c->push_back( osg::Vec4( 1.f, 1.f, 1.f, 1.f ) );

    osg::ref_ptr<osg::Group> grp = new osg::Group;
    {
        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
        geom->setVertexArray( v.get() );
        geom->setTexCoordArray( 0, tc.get() );
        geom->setColorArray( c.get() );
        geom->setColorBinding( osg::Geometry::BIND_OVERALL );

        // -X panal
        GLushort indices[] = { 0, 1, 2, 3 };
        geom->addPrimitiveSet( new osg::DrawElementsUShort(
            osg::PrimitiveSet::QUADS, 4, indices ) );

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable( geom.get() );
        grp->addChild( geode.get() );
    }

    return( grp.release() );
}

/**
 * This class represents the homography
 */
class ReferencedHomography : public osg::Referenced
{

public:
	vector< cv::Mat_<float> > homoVector;
//	vector< int > modelIDVector;

	ReferencedHomography(int modelNumber)
	{
		homoVector.resize(modelNumber);
//		modelIDVector.clear();
//		homo = cv::Mat::eye(3,3, CV_32F);
//		modelID = -1;
	}

	~ReferencedHomography()
	{
		homoVector.clear();
//		homo.release();
	}
};

/**
 * Call back of the background
 * - Update the background with video frames
 * - Detect interest points
 */
class UpdateTexture : public osg::NodeCallback
{
private:
	void updateBackgroundTexture(osg::Node* textureNode)
	{
		//Set the texture
		cvImg = video->getImageWithDetectedPoints()->clone();

		cv::flip(cvImg,cvImg,0);

		osg::ref_ptr<osg::Image> image = new osg::Image();
		image->setImage(cvImg.cols,
						  cvImg.rows,
						  3,
						  GL_RGB,
						  GL_BGR,
						  GL_UNSIGNED_BYTE,
						  cvImg.data,
						  osg::Image::NO_DELETE,1);

		osg::ref_ptr<osg::StateSet> state = textureNode->getOrCreateStateSet();
		osg::ref_ptr<osg::Texture2D> tex = (osg::Texture2D*) state->getTextureAttribute(0,osg::StateAttribute::TEXTURE);
		tex->setImage( image.get() );
	}

	void updateDetectedPoints(osg::Geode* detectedPointNode)
	{
		osg::ref_ptr<osg::Geometry> detectedPointGeometry = dynamic_cast<osg::Geometry*>(detectedPointNode->getDrawable(0));
		osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
		for (unsigned int j = 0 ; j < video->getPoints()->size() ; j++)
		{
			vertices->push_back (osg::Vec3 ( (*video->getInvertedPoints())[j].x, (*video->getInvertedPoints())[j].y, 0.0));
		}
		detectedPointGeometry->setVertexArray(vertices);
		detectedPointGeometry->setPrimitiveSet(0,new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));
	}

	void matchPointSet(osg::Node* node)
	{
		assert(node->getNumParents() == 1);  //The camera has only one parent node, that is the root
		ReferencedHomography* refHomo = dynamic_cast<ReferencedHomography*>( node->getParent(0)->getUserData() );

//		cv::Mat_<float> foundHomo;
		vector< cv::Mat_<float> > homos;
		vector< int > modelIDs;

		Chronometer timer;
		timer.tic();
		algoMatching->findModelsandHomography2Ds(*video->getInvertedPoints(), modelIDs, homos);
		cout<<"Matching used "<<timer.tac()<<" ms"<<endl;

		//Release all homography for all models, default is invisible
		for (unsigned int i = 0 ; i < refHomo->homoVector.size() ; i++)
		{
			refHomo->homoVector[i].release();
		}

		//Set visible ones
		for (unsigned int i = 0 ; i < modelIDs.size() ; i++)
		{
			refHomo->homoVector[modelIDs[i]] = homos[i].clone();
		}

		//Print log
		cout<<"Find "<<modelIDs.size()<<" models: ";
		for (unsigned int i = 0 ; i < modelIDs.size() ; i++)
		{
			cout<<"#"<<modelIDs[i]<<" ";
		}
		cout<<endl;
	}
public:
	UpdateTexture(cv::VideoCapture &capture)
	{
//		detector = new GoodFeatureDetector();
//		detector = new SimplePointDetector();
		detector = new DrawingIntersectionDetector(); //detect intersections
		video = new DetectFromVideo(detector, capture, false);
	}
   
   virtual void operator()( osg::Node* node, osg::NodeVisitor* nv )
   {
	   Chronometer timer;
	   timer.tic();
	   bool hasNextImage = video->nextImage();
	   cout<<"Image processing used "<<timer.tac()<<" ms"<<endl;
	   if (hasNextImage)
	   {
		   osg::ref_ptr<osg::Group> backgroundCameraNode = dynamic_cast<osg::Group*>(node);
		   for (unsigned int i = 0 ; i < backgroundCameraNode->getNumChildren() ; i++)
		   {
			   if (backgroundCameraNode->getChild(i)->getName() == string("Background texture"))
			   {
				   updateBackgroundTexture(backgroundCameraNode->getChild(i));
			   }

			   if (backgroundCameraNode->getChild(i)->getName() == string("Detected points"))
			   {
				   updateDetectedPoints(dynamic_cast<osg::Geode*>(backgroundCameraNode->getChild(i)));
			   }
		   }
		   matchPointSet(node);
		   traverse( node, nv );
	   }
	   else
	   {
		   exit(1);
	   }
   }

   AbstractKPDetector* detector;
   DetectFromVideo* video;
   cv::Mat cvImg;
};


/**
 * Seletec which model to show
 */
class ModelSelector : public osg::NodeCallback
{

public:
	/**
	 * Constructor:
	 */
	ModelSelector() {};

	/**
	 * set 3D transformation with homography
	 * @param homo: [Input] Estimated Homography
	 */
   virtual void operator()( osg::Node* node, osg::NodeVisitor* nv )
   {
		assert(node->getNumParents() == 1);  //The switch node has only one parent node, that is the root
	    vector< cv::Mat_<float> >& homos = dynamic_cast<ReferencedHomography*>( node->getParent(0)->getUserData() )->homoVector;

		dynamic_cast<osg::Switch*>(node)->setAllChildrenOff();

		for (unsigned int i = 0 ; i < homos.size() ; i++)
			if (!homos[i].empty())
			{
				dynamic_cast<osg::Switch*>(node)->setValue(i, true);
			}

		traverse( node, nv );
   }

};

/**
 * Apply transformation on models
 */
class UpdateTransformation : public osg::NodeCallback
{
private:
	osg::Quat getQuatFromRotation(cv::Mat_<float> &R)
	{
		cv::Vec4f quat;
		quat.val[0] = sqrt(1.0 + R[0][0] + R[1][1] + R[2][2]) / 2.0;
		double w4 = (4.0 * quat.val[0]);
		quat.val[1] = (R[2][1] - R[1][2]) / w4 ;
		quat.val[2] = (R[0][2] - R[2][0]) / w4 ;
		quat.val[3] = (R[1][0] - R[0][1]) / w4 ;
		w4 = norm(quat);

		osg::Quat result;
		result._v[0] = quat.val[0]/w4;
		result._v[1] = quat.val[1]/w4;
		result._v[2] = quat.val[2]/w4;
		result._v[3] = quat.val[3]/w4;

		return result;
	}

	/**
	 * Find 3D rotation and translation with homography
	 * @param homo: [Input] Estimated Homography
	 * @param rot: [Output] Rotation result
	 * @param trans: [Output] Translation result
	 */
	void findCameraPos(const cv::Mat_<float>& homo, cv::Mat_<float>& rot, cv::Vec3f &trans)
	{
		float coefH;

		cv::Mat_<float> invcam = cameraMatrix.inv();
		//coefCamGL is used to transform camera matrix in camera model to camera matrix in openGL model ( z direction is different )
		cv::Mat_<float> coefCamGL = (cv::Mat_<float>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, -1);
		rot = coefCamGL*invcam*homo;

		//scaling rotation matrix to right order = find the coefficient lost while calculating homography
		/**
		 * [Cam]*[Rot]' = [Homo]*coefH
		 *          |fx  0  u0|
		 * [Cam]' = | 0 fy  v0|
		 *          | 0  0   1|
		 *
		 *          |R11 R12 t1|
		 * [Rot]' = |R21 R22 t2|
		 *          |R31 R32 t3|
		 *
		 *           |H11 H12 H13|
		 * [Homo]' = |H21 H22 H23|
		 *           |H31 H32   1|
		 */
		coefH = 2/(cv::norm(rot.col(0)) + cv::norm(rot.col(1)));
		rot = rot * coefH;
		trans = rot.col(2);

		//The their rotation vector in the rotation matrix is produced as the cross product of other two vectors
		cv::Mat tmp = rot.col(2);
		rot.col(0).cross(rot.col(1)).copyTo(tmp);

		//Add here to make rotation matrix a "real" rotation matrix in zhang's method
		/**
		 * rot = UDVt
		 * rot_real = UVt
		 */
		cv::Mat_<float> D,U,Vt;
		cv::SVD::compute(rot,D,U,Vt);
		rot = U*Vt;
	}

public:
	/**
	 * Constructor:
	 * @param cameraMat: Camera matrix.
	 */
	UpdateTransformation(const cv::Mat_<float>& cameraMat, int ID) : cameraMatrix(cameraMat), modelID(ID)
{};

	/**
	 * set 3D transformation with homography
	 * @param homo: [Input] Estimated Homography
	 */
   virtual void operator()( osg::Node* node, osg::NodeVisitor* nv )
   {
		cv::Mat_<float> rot;
		cv::Vec3f trans;
		assert(node->getNumParents() == 1);  //The cuurent node (transformation node) has only one parent node, that is the switch node
		assert(node->getParent(0)->getNumParents() == 1);  //The switch node has only one parent node, that is the root

		cv::Mat_<float> &homo = dynamic_cast<ReferencedHomography*>( node->getParent(0)->getParent(0)->getUserData() )->homoVector[modelID];

		if (!homo.empty())
		{
			findCameraPos(homo,rot,trans);
			transformation.set(rot.at<float>(0,0), rot.at<float>(1,0), rot.at<float>(2,0), 0.0f,
					rot.at<float>(0,1), rot.at<float>(1,1), rot.at<float>(2,1), 0.0f,
					rot.at<float>(0,2), rot.at<float>(1,2), rot.at<float>(2,2), 0.0f,
					trans.val[0], trans.val[1], trans.val[2], 1.0f);

			osg::MatrixTransform* currentTransformation = dynamic_cast<osg::MatrixTransform*>( node );
			currentTransformation->setMatrix(transformation);
		}

		traverse( node, nv );
   }

   osg::Matrixf transformation;
   cv::Mat_<float> cameraMatrix;
   int modelID;
};

//Declare functions for creating the scene
osg::ref_ptr<osg::Node> createBackground(cv::VideoCapture &vCapture, float camWidth, float camHeight);
osg::ref_ptr<osg::Node> createScene(cv::Mat_<float> &camProjection, std::vector< modelloader::Model >& models);
osg::ref_ptr<osg::Node> createSwitchNode(std::vector< modelloader::Model >& models);
osg::ref_ptr<osg::Node> createModel(modelloader::Model& currentmodel);
osg::ref_ptr<osg::Node> createReferencePointNode(modelloader::ModelPoints& modelPoints);

osg::ref_ptr<osg::Node> createBackground(cv::VideoCapture &vCapture, float camWidth, float camHeight)
{
	/**Create the background camera */
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;

    //Set camera parameters
    camera->setRenderOrder( osg::Camera::PRE_RENDER );
    camera->setReferenceFrame( osg::Camera::ABSOLUTE_RF );

    camera->setProjectionMatrixAsOrtho(0,camWidth,0, camHeight, -1 , 1);
    camera->setViewMatrixAsLookAt(
	        osg::Vec3(0.0f, 0.0f, 0.0f),
			osg::Vec3(0, 0, -1),
			osg::Vec3(0, 1, 0)
		);
    osg::ref_ptr<UpdateTexture> backgroundCallback = new UpdateTexture(vCapture);
    camera->setUpdateCallback( backgroundCallback );

	//Create the geometry for texture 2D
    {
		osg::ref_ptr<osg::Node> textureNode = createGeodes(camWidth, camHeight);
		osg::StateSet* state = textureNode->getOrCreateStateSet();
		state->setMode( GL_LIGHTING, osg::StateAttribute::OFF |  osg::StateAttribute::PROTECTED );

		// Set the texture object but not connect with any image
		osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
		tex->setResizeNonPowerOfTwoHint(false);
		tex->setUnRefImageDataAfterApply( true );
		state->setTextureAttributeAndModes( 0, tex.get() );

		// Turn on blending
		osg::BlendFunc* bf = new osg::BlendFunc( osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA );
		state->setAttributeAndModes( bf );

		// Turn on alpha testing
		osg::AlphaFunc* af = new osg::AlphaFunc(osg::AlphaFunc::GREATER, 0.05f );
		state->setAttributeAndModes( af );

		textureNode->setDataVariance( osg::Object::DYNAMIC );
		textureNode->setName("Background texture");
		camera->addChild( textureNode.get() );
    }

    //Create geometry for detected points
    {
    	osg::ref_ptr<osg::Geode> detectedPointsNode (new osg::Geode());
    	detectedPointsNode->addDrawable (createPrimitiveGeometry(osg::PrimitiveSet::POINTS, vector<cv::Point2f>(), osg::Vec4(0.0,0.0,1.0,1.0)));
    	detectedPointsNode->setDataVariance( osg::Object::DYNAMIC );
    	detectedPointsNode->setName("Detected points");
    	camera->addChild( detectedPointsNode.get() );
    }

    return (camera);

}

osg::ref_ptr<osg::Node> createScene(cv::Mat_<float> &camProjection, std::vector< modelloader::Model >& models)
{
	//Set the transformation for all models
//	osg::ref_ptr<osg::MatrixTransform> globalTransform = new osg::MatrixTransform();
//	osg::ref_ptr<osg::NodeCallback> nc2 = new UpdateTransformation(mloader.camProjection);
//	globalTransform->setUpdateCallback(nc2);
//
//	globalTransform->addChild(createSwitchNode(models));
	return createSwitchNode(models);
}

osg::ref_ptr<osg::Node> createSwitchNode(std::vector< modelloader::Model >& models)
{
	osg::ref_ptr<osg::Switch> switchNode = new osg::Switch();
	osg::ref_ptr<osg::NodeCallback> switchCallBack = new ModelSelector();
	switchNode->setUpdateCallback(switchCallBack);

	for (unsigned int i = 0 ; i < models.size() ; i++)
	{
		//Create the transformation node for each model
		osg::ref_ptr<osg::MatrixTransform> globalTransform = new osg::MatrixTransform();
		osg::ref_ptr<osg::NodeCallback> nc2 = new UpdateTransformation(mloader.camProjection, i);
		globalTransform->setUpdateCallback(nc2);
		globalTransform->addChild(createModel(models[i]));

		//Add to switch node
		switchNode->addChild(globalTransform);
	}

	return switchNode;
}

osg::ref_ptr<osg::Node> createModel(modelloader::Model& currentmodel)
{
	//Set the node for each model
	osg::ref_ptr<osg::Group> modelNode = new osg::Group();


	modelNode->addChild(createReferencePointNode(currentmodel.modelpoints));

	//add model points to algorithm
	algoMatching->addModel(currentmodel.modelpoints.points);

	for (unsigned int i = 0 ; i < currentmodel.objects.size() ; i++)
	{
		osg::ref_ptr<osg::MatrixTransform> objectTransform = new osg::MatrixTransform();
		objectTransform->setMatrix(currentmodel.objects[i].osgMatrix);

		objectTransform->addChild(currentmodel.objects[i].model);
		modelNode->addChild(objectTransform);
	}

	return modelNode;
}

osg::ref_ptr<osg::Node> createReferencePointNode(modelloader::ModelPoints& modelPoints)
{
	osg::ref_ptr<osg::Geode> geode (new osg::Geode());

	geode->addDrawable (createPrimitiveGeometry(osg::PrimitiveSet::POINTS, modelPoints.points, osg::Vec4(1.0,0.0,0.0,1.0)));
	geode->addDrawable (createPrimitiveGeometry(osg::PrimitiveSet::LINE_LOOP, modelPoints.border, osg::Vec4(1.0,0.0,0.0,1.0)));
	return (geode.get());
}

int main( int argc, char** argv)
{
	algoMatching = new LGC();
	mloader.readDataFrom(argv[1]);
	mloader.printInfo(cout);

   cv::VideoCapture vCapture;
   char * p ;
   int camID = strtol(mloader.inputVideo.streamName.c_str(), &p, 10 );

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
	   if ( !vCapture.open(mloader.inputVideo.streamName))
	   {
		   std::cout<<"There is no input video stream from file "<<mloader.inputVideo.streamName<<std::endl;
		   return EXIT_FAILURE;
	   }
   }

   vCapture.set(CV_CAP_PROP_FRAME_WIDTH,mloader.inputVideo.width);  
   vCapture.set(CV_CAP_PROP_FRAME_HEIGHT,mloader.inputVideo.height);

   cv::Mat firstFrame;

   vCapture >> firstFrame;
   if (firstFrame.empty())
   {
	   std::cout<<"Video stream is empty ! "<<std::endl;
	   return EXIT_FAILURE;
   }

   /** This is for linux */
   cout<<"The actual window size is "<<firstFrame.cols<<"x"<<firstFrame.rows<<endl;
   /** End for linux */

   int width = firstFrame.cols;
   int height = firstFrame.rows;
   float camFocus = mloader.camProjection[0][0];
   float apsectRatio = width/(float)height;

   osg::ref_ptr<osg::Group> root = new osg::Group();
   root->setUserData(new ReferencedHomography(mloader.modelArray.size()));
   root->addChild( createBackground(vCapture, width, height));
   root->addChild( createScene(mloader.camProjection, mloader.modelArray) );

   osgViewer::Viewer viewer;
   viewer.setUpViewInWindow(100, 100, width, height);
   viewer.setSceneData( root );
   
   viewer.getCamera()->setClearMask( GL_DEPTH_BUFFER_BIT );
   /**
    * Set camera parameter according to real the camera
    */
   viewer.getCamera()->setProjectionMatrixAsPerspective( 360.0*atan(height/(2*camFocus))/M_PI, apsectRatio, 1., 100. );
   viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3(0, 0, 0), osg::Vec3(0, 0, -1), osg::Vec3(0, 1, 0));   //default camera position
   
   while (!viewer.done())
   {
      viewer.updateTraversal();
      viewer.frame();
   }
   
   return EXIT_SUCCESS;
}

