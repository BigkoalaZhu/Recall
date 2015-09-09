#include "RecallAnalysis.h"

#include "IDSC\shape_descriptor.hpp"


RecallAnalysis::RecallAnalysis(QString path)
{
	Recall_path = path;
}


RecallAnalysis::~RecallAnalysis(void)
{

}

vector<Point> RecallAnalysis::simpleContour( const Mat& currentQuery, int n=300 )
{
	vector<vector<Point> > _contoursQuery;
	vector <Point> contoursQuery;
	findContours(currentQuery, _contoursQuery, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE);
	for (size_t border=0; border<_contoursQuery.size(); border++)
	{
		for (size_t p=0; p<_contoursQuery[border].size(); p++)
		{
			contoursQuery.push_back( _contoursQuery[border][p] );
		}
	}

	// In case actual number of points is less than n
	int dummy=0;
	for (int add=(int)contoursQuery.size()-1; add<n; add++)
	{
		contoursQuery.push_back(contoursQuery[dummy++]); //adding dummy values
	}

	vector<Point> cont;
	for (int i=0; i<n; i++)
	{
		cont.push_back(contoursQuery[i*(contoursQuery.size()-1)/n]);
	}
	return cont;
}

void RecallAnalysis::LoadAndAnalysisOneImage(QString image_file)
{
	Mat image = imread(image_file.toStdString().c_str(), IMREAD_GRAYSCALE);
	cv::threshold(image,image,5,255,THRESH_BINARY);

	vector<Point> contQuery = simpleContour(image);
}
