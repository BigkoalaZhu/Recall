#include "Flexibility.h"


Flexibility::Flexibility(vector<Point> C)
{
	contour = C;
	contourSize = contour.size();
}


Flexibility::~Flexibility(void)
{
}

void Flexibility::DetectRadius()
{
	Eigen::MatrixXd pairs = Eigen::MatrixXd::Ones(contourSize, contourSize);
	for (int i = 0; i < contourSize-1; i++)
	{
		for (int j = i+1; j < contourSize; j++)
		{
			double dist = (contour[i] - contour[j]).dot(contour[i] - contour[j]);
			double denominator = double(j)/contourSize - double(i)/contourSize < 0.5 ? double(j)/contourSize - double(i)/contourSize : 1 - double(j)/contourSize + double(i)/contourSize;
			pairs(i,j) = sqrt(dist)/denominator;
		}
	}

	for (int i = 0; i < contourSize-1; i++)
	{
		for (int j = i+1; j < contourSize; j++)
		{

		}
	}
}

void Flexibility::CalcDescriptor()
{

}