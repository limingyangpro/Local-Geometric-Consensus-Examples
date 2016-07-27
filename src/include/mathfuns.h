/*
 * mathfuns.h
 *
 *  Created on: Apr 7, 2014
 *      Author: liming
 */

#ifndef MATHFUNS_H_
#define MATHFUNS_H_

#include "basictype.h"
#include <limits>        //for max float

/**
 * This class contains all needed math function
 * All functions are static
 */
class MathFuns
{
public:
	const static float small_value;
public:
	/**----------------------------Camera----------------------------------------*/
	/** Given homography, find translation and rotation (Assuming : cam is unity matrix, no +/- z conversion) */
	static void findCameraPos(const cv::Mat& homo, cv::Mat& rot, cv::Vec3f &trans);
	/** Given homography and camera matrix, find translation and rotation */
	static void findCameraPos(const cv::Mat& homo, const cv::Mat& cam, cv::Mat& rot, cv::Vec3f &trans);
	/** Given homography and camera focus, find rotation matrix R */
	static cv::Mat findRotationMatrix(cv::Mat homo, float focus);
	/** Given homography and camera focus, find translation and rotation */
	static void findCameraPos(cv::Mat homo, float focus, cv::Vec3f &rot, cv::Vec3f &trans);
	/** Given homography and camera focus, find rotation in quaternion form */
	static void toQuaternion(cv::Mat homo, float focus, cv::Vec4f &quat);
	/** Given camera focus, compare two homography, return their differences in degree */
	static float compareHomos(const cv::Mat &homo1, const cv::Mat &homo2, float focus);
	/** Given rotation matrix get its quaternion form */
	static void rot2Quaternion(const cv::Mat_<float> &rot, cv::Vec4f &quat);
	static void quaternion2Rot(const  cv::Vec4f &quat, cv::Mat_<float> &rot);
	/** Compare two rotation matrix, return their differences in degree */
	static float compareQuats(const cv::Vec4f &quat1, const cv::Vec4f &quat2);

	/** GIven Fundunmental matrix and camera matrix, find the rotation and translation of the camera */
	static void findCameraMotion(const cv::Mat& fundamental, const cv::Mat& cam, cv::Mat& rot, cv::Vec3f &trans);

	/**----------------------------Pure math-------------------------------------*/
	/**Generate Combination */
	static void generateCombination(const int n, const int m, std::vector< std::vector<int> > &Combi);    /**Generate Combination for whatever n,m is, stored in Combi */
	/**Generate Permutation */
	static void generatePermutation(const int n, const int m, vector< vector<int> > &Combi);    /**Generate Permutation for whatever n,m is, stored in Combi */
	/**Generate Combination one by one */
	template <typename Iterator>
	static inline bool next_combination(const Iterator first, Iterator k, const Iterator last);
	/** Hungarian assignment alogrithm */
	static void hungarianSearch(const cv::Mat_<float>& costmat0, list< std::pair<short,short> >* solution);
	/** Hungarian assignment alogrithm (legacy) */
	static void hungarianSearch(const cv::Mat_<float> costmat, vector<short>* solution);
	/** Nearest assignement */
	static void nearestAssign(PtArray pts1, PtArray pts2);

	/**----------------------------Geometrie 2D--------------------------------------*/
	/**Find triangle surface of p1,p2,p3*/
	template <typename Point2D>
	static float triangleSurface(const Point2D p1, const Point2D p2, const Point2D p3);
	/**Squared point distance */
	template <typename Point2D>
	static float pointDistance2(Point2D p1, Point2D p2);
	/**Calculate the angle of line p1p2 and line p1p3 */
	template <typename Point2D>
	static float lineAngle(Point2D p1, Point2D p2, Point2D p3);
	/**Same function with  p1 = (0,0) */
	template <typename Point2D>
	static float  lineAngle(Point2D p2, Point2D p3);
	/**Compare two affinities */
	static bool isAffinitySimilar(cv::Mat_<float> &src1, cv::Mat_<float> &src2, float ratio, float rotdiff, float dispdiff);
	/**Matrix condition number 2D*/
	static float conditionNumber(cv::Mat_<float> &src, int norm);

	/**
	 * ----------------------------Geometry 3D-------------------------------------
	 * */
	template <typename Point3D>
	static float pointDistanceSqr3D(const Point3D &p1, const Point3D &p2);
	template <typename Point3D>
	static float dot3D(const Point3D &p1, const Point3D &p2);
	template <typename Point3D>
	static Point3D cross3D(const Point3D &p1, const Point3D &p2);
	template <typename Point3D>
	static Point3D cm3D(const vector<Point3D> &pts);
};




#endif /* MATHFUNS_H_ */
