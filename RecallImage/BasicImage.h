#pragma once
#include "qstring.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace cv;

class BasicImage
{
public:
	BasicImage();
	~BasicImage(void);

	void LoadImage(QString filename);  //Load image data from file
	void EdgeDetection(int lowThreshold); // Get silhouette edge and store as edges.png
	void SLICSuperPixel(int pnum, double compact); // Get the super pixel segmentation result and store as cutted.png

private:
	void sortGroups(std::vector<std::vector<Point>> us);

	Mat OriginalImage;
	Mat GrayImage;
	Mat EdgesImage;
	Mat cutted;
	unsigned int* pbuff;

	std::vector<std::vector<Point>> kernal_groups;
};

