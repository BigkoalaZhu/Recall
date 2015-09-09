#pragma once

#include "opencv2/shape.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/utility.hpp"

#include <vector>

using namespace std;
using namespace cv;

class FourierDescriptor
{
public:
	static void calculate(vector<Point> contour, double* descriptor, int length)
	{
/*		int N = contour.size();
		Mat X(1, N, CV_32FC1);
		Mat Y(1, N, CV_32FC1);

		Point center(0,0);
		for (int i = 0; i < N; i++)
		{
			center += contour[i];
		}
		center = center/N;

		for (int i = 0; i < N; i++)
		{
			X.at<float>(0,i) = contour[i].x - center.x;
			Y.at<float>(0,i) = contour[i].y - center.y;
		}

		Mat planes[] = {Mat_<float>(X), Mat_<float>(Y)};
		Mat complexI;
		merge(planes, 2, complexI);
		dft(complexI, complexI);
		split(complexI, planes);
		magnitude(planes[0], planes[1], planes[0]);
		Mat magI = planes[0];

		for (int i = 2; i < N; i++)
		{
			descriptor[i-2] = magI.at<float>(0,i)/magI.at<float>(0,1);
		}*/

		int N = contour.size();
		Mat Z(1, N, CV_32F);

		Point center(0,0);
		int maxX = 0, maxX_idx = 0, maxY = 0;
		for (int i = 0; i < N; i++)
		{
			center += contour[i];
			if (contour[i].x > maxX)
			{
				maxX = contour[i].x;
				maxY = contour[i].y;
				maxX_idx = i;
			}
			if (contour[i].x == maxX && contour[i].y > maxY)
			{
				maxX = contour[i].x;
				maxY = contour[i].y;
				maxX_idx = i;
			}
		}
		center = center/N;
		for (int i = 0; i < maxX_idx; i++)
			contour.push_back(contour[i]);
		for (int i = 0; i < maxX_idx; i++)
			contour.erase(contour.begin());

		for (int i = 0; i < N; i++)
		{
			Z.at<float>(0,i) = sqrt(pow(contour[i].x - center.x,2) + pow(contour[i].y - center.y,2));
		}

		Mat planes[] = {Mat_<float>(Z), Mat::zeros(Z.size(), CV_32F)};
		Mat complexI;
		merge(planes, 2, complexI);
		dft(complexI, complexI);
		split(complexI, planes);
		magnitude(planes[0], planes[1], planes[0]);
		Mat magI = planes[0];

		for (int i = 1; i < length + 1; i++)
		{
			descriptor[i-1] = magI.at<float>(0,i)/magI.at<float>(0,0);
		}
	}

	static void calculate(const Mat& currentQuery, double* descriptor, int length)
	{
		int n = 300;
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

		calculate(cont, descriptor, length);
	}

	static double distance(double* descriptorA, double* descriptorB, int n)
	{
		double sum = 0;
		for (int i = 0; i < n; i++)
		{
			sum += pow(descriptorA[i] - descriptorB[i],2);
		}
		sum = sqrt(sum);
		return sum;
	}

	static double distance(vector<Point> contourA, vector<Point> contourB)
	{
		int N = 15;
		double * descriptorA = new double[N];
		double * descriptorB = new double[N];
		calculate(contourA, descriptorA, N);
		calculate(contourB, descriptorB, N);
		return distance(descriptorA, descriptorB, N);
	}
};

