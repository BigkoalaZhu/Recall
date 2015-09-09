#pragma once
#include "qstring.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

using namespace cv;
using namespace std;

class RecallAnalysis
{
public:
	RecallAnalysis(QString path);
	~RecallAnalysis(void);

	void LoadAndAnalysisOneImage(QString image_file);

private:
	vector<Point> simpleContour( const Mat& currentQuery, int n);

	QString Recall_path;
};

