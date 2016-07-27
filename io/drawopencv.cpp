/*
 * drawopencv.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: liming
 */

#include "drawopencv.h"

/**
 * fond    		  : the image of the scene , can be from camera
 * backgrdptlists : sets of point lists detected on the background
 * refimg         : the image of reference, captured from topview
 * vplists        : sets of point lists detected from the reference
 */
cv::Mat drawOpenCV(const cv::Mat* fond,
                   const vector< vector<cv::Point2f> >* backgrdptarrays,
                   const cv::Mat* refimg,
                   const vector< vector<cv::Point2f> >* refptarrays,
                   const vector< pair<int , pair<int,int> > > *corresp,
				   const cv::Mat homo)
{

   cv::Mat resultFrame = fond->clone();

   unsigned int listnum,i,j;

   //Invert coordinate y
   int testInversion = resultFrame.rows;

   // Show Img and Points -> Backgroung OpenCV
   if ( (backgrdptarrays != NULL)&&(backgrdptarrays->size() != 0) )
	{
      // Can't draw dotted line ine OpenCV easily :/
	   //draw borders
	   if ((*backgrdptarrays)[0].size() != 0)
	   {
		  cv::Point2f p0((*backgrdptarrays)[0][0]);
		  cv::Point2f p1((*backgrdptarrays)[0][1]);
		  cv::Point2f p2 = (*backgrdptarrays)[0][2];
		  cv::Point2f p3 = (*backgrdptarrays)[0][3];

		  // Drawing border line --> WARNING PIXELS ARE STORED IN BGR AND NOT RGB
		  cv::line(resultFrame, cv::Point(p0.x, p0.y), cv::Point(p1.x, p1.y), cv::Scalar(255,0,0));
		  cv::line(resultFrame, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(255,0,0));
		  cv::line(resultFrame, cv::Point(p2.x, p2.y), cv::Point(p3.x, p3.y), cv::Scalar(255,0,0));
		  cv::line(resultFrame, cv::Point(p0.x, p0.y), cv::Point(p3.x, p3.y), cv::Scalar(255,0,0));
	   }
      //Draw points
		listnum= backgrdptarrays->size();
		for (i = 1 ; i < listnum ; i++)
		{
			for (j = 0 ; j < backgrdptarrays->at(i).size() ; j++)
			{
            cv::Point2f temp = backgrdptarrays->at(i).at(j);
            cv::circle(resultFrame, cv::Point(temp.x, temp.y), 4, cv::Scalar(255,0,0),3);
         }
		}
	}

   // Drawing in red
	// Show Img and Points OpenCV
   if ( (refptarrays != NULL)&&(refptarrays->size() != 0)&&(!homo.empty()) )
	{
      // Can't draw dotted line ine OpenCV easily :/
      std::vector<cv::Point2f> inputsBorder;
      std::vector<cv::Point2f> outputsBorder;

      inputsBorder.push_back((*refptarrays)[0][0]);
      inputsBorder.push_back((*refptarrays)[0][1]);
      inputsBorder.push_back((*refptarrays)[0][2]);
      inputsBorder.push_back((*refptarrays)[0][3]);

      cv::perspectiveTransform(inputsBorder, outputsBorder, homo);


      // Drawing border line
      // WARNING PIXELS ARE STORED in BGR order and not RGB
      cv::line(resultFrame, cv::Point(outputsBorder[0].x, outputsBorder[0].y), cv::Point(outputsBorder[1].x, outputsBorder[1].y), cv::Scalar(0,0,255),1);
      cv::line(resultFrame, cv::Point(outputsBorder[1].x, outputsBorder[1].y), cv::Point(outputsBorder[2].x, outputsBorder[2].y), cv::Scalar(0,0,255),1);
      cv::line(resultFrame, cv::Point(outputsBorder[2].x, outputsBorder[2].y), cv::Point(outputsBorder[3].x, outputsBorder[3].y), cv::Scalar(0,0,255),1);
      cv::line(resultFrame, cv::Point(outputsBorder[0].x, outputsBorder[0].y), cv::Point(outputsBorder[3].x, outputsBorder[3].y), cv::Scalar(0,0,255),1);
      cv::line(resultFrame, cv::Point(outputsBorder[0].x, outputsBorder[0].y), cv::Point(outputsBorder[2].x, outputsBorder[2].y), cv::Scalar(0,0,255),1);
      cv::line(resultFrame, cv::Point(outputsBorder[1].x, outputsBorder[1].y), cv::Point(outputsBorder[3].x, outputsBorder[3].y), cv::Scalar(0,0,255),1);


      //Draw points
		listnum= refptarrays->size();
		for (i = 1 ; i < listnum ; i++)
		{

         std::vector<cv::Point2f> outputs;
         cv::perspectiveTransform((*refptarrays)[i], outputs, homo);

			for (j = 0 ; j < outputs.size() ; j++)
			{
            cv::Point2f temp = outputs.at(j);
            cv::circle(resultFrame, cv::Point(temp.x, temp.y), 2, cv::Scalar(0,0,255), 2);
         }
		}

	}

   if (refimg != NULL)
   {
		cv::Mat tmpim;
//		cv::Mat reftmp = refimg->clone();
//		reftmp.setTo(cv::Scalar(255,255,255));
		cv::warpPerspective(*refimg, tmpim, homo, fond->size());
//		cv::bitwise_and(resultFrame, resultFrame, resultFrame,tmpim);
//		cv::imshow("tt",resultFrame);
//		cv::waitKey(-1);
		cv::addWeighted(resultFrame, 1, tmpim, 1, 0, resultFrame);
   }

   // Show correspondences in OpenCV
    if (corresp != NULL)
    {
 	   unsigned int num;
 	   num = corresp->size();

 	   //Get reference point in the correspondence list
 	   std::vector<cv::Point2f> reprojectedRefPoints(num), imgPoints(num);
 	   for (i = 0 ; i < num ; i++)
 	   {
 		   reprojectedRefPoints[i] = (*refptarrays)[1][(*corresp)[i].second.first];
 		   imgPoints[i] = (*backgrdptarrays)[1][(*corresp)[i].second.second];
 	   }

 	   //Transform reference points
 	   cv::perspectiveTransform(reprojectedRefPoints, reprojectedRefPoints, homo);

 	   cv::Scalar color;
 	   for (i = 0 ; i < num ; i++)
 		{
 		   if ((*corresp)[i].first == 1)
 		   {
 			   color = cv::Scalar(0,0,0);
 		   }
 		   else
 		   {
 			   color = cv::Scalar(120,120,120);
 		   }

 		  cv::line(resultFrame, reprojectedRefPoints[i], imgPoints[i], color, 4);
 		}
    }

   return resultFrame;
}


