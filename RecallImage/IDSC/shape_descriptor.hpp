#ifndef SHAPE_DESCRIPTOR_H
#define SHAPE_DESCRIPTOR_H
#include <math.h>
#include <vector>
#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "reference_point.hpp"

#include <vector>
#include "Eigen/dense"

using namespace std;

struct ShapeDistance {
	double d;
	double theta;
	int dBin;
	int tBin;
};

class IDSC_descriptor
{
	std::vector<cv::Point> points_;
	std::list<ShapeDistance*> dists;
	bool getContourOrientation(const std::vector<cv::Point> &contour);
	double maxDist;
private:
	bool intersect(const cv::Point &p1a,const cv::Point &p1b,const cv::Point &p2a,const cv::Point &p2b);
	int moveLeft(int n, int size);
	int moveRight(int n, int size);
	void addShapeDist(double dist, double angle);
	void getContext(double *sc);
public:
	double dist(const cv::Point &pt1,const cv::Point &pt2);
	double angle(const cv::Point &p1,const cv::Point &p2);
	void getShapeContext(double *sc,const std::vector<cv::Point> &contour, int nPt, bool orientation);
	void getFullShapeContext(double *sc,const std::vector<cv::Point> &contour);
	void normalizeShapeContext(double *sc, int pt_num);
	cv::Mat graphSc(double *sc,int width,int height);

	vector<Point> simpleContour( const Mat& currentQuery, int n);
	Eigen::MatrixXf CostMat(double *sc1, double *sc2, int width);
	void getShapeContext(double *sc, const Mat& currentQuery, int pt_num);
};
#endif
