#pragma once

#include "opencv2/shape.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/utility.hpp"

#include "Eigen/core"

#include <vector>

using namespace std;
using namespace cv;

class Flexibility
{

public:
	Flexibility(vector<Point> C);
	~Flexibility(void);

	void DetectRadius();
	void CalcDescriptor();
private:
	int contourSize;
	vector<Point> contour;
	double r;
};

