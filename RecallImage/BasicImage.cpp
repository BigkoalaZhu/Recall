#include "BasicImage.h"
#include "SLIC\SLIC.h"

BasicImage::BasicImage(void)
{
}


BasicImage::~BasicImage(void)
{
}

void BasicImage::LoadImage(QString filename)
{
	OriginalImage = imread(filename.toLocal8Bit().constData(), CV_LOAD_IMAGE_COLOR);
	cvtColor( OriginalImage, GrayImage, CV_RGB2GRAY );
}

void BasicImage::EdgeDetection(int lowThreshold)
{
	Mat DeNoise;
	int ratio = 3;
	int kernel_size = 3;
	blur( GrayImage, DeNoise, Size(3,3) );
	Canny( DeNoise, DeNoise, lowThreshold, lowThreshold*ratio, kernel_size );

	EdgesImage = Scalar::all(0);
	GrayImage.copyTo( EdgesImage, DeNoise);
	imwrite("edges.png",EdgesImage);
}

void BasicImage::SLICSuperPixel(int pnum, double compact)
{
	int m = OriginalImage.channels();
	int c = OriginalImage.cols;
	int r = OriginalImage.rows;

	pbuff = new unsigned int[r*c];
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			pbuff[i*c+j] = (unsigned int)OriginalImage.at<Vec3b>(i,j)[2]*pow(2,16) + (unsigned int)OriginalImage.at<Vec3b>(i,j)[1]*pow(2,8) + (unsigned int)OriginalImage.at<Vec3b>(i,j)[0];
		}
	}

	SLIC segment;
	int* klabels = NULL;
	int numlabels(0);
	segment.DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels(pbuff, c, r, klabels, numlabels, pnum, compact);

	std::vector<std::vector<Point>> unsorted_kernal_groups;
	unsorted_kernal_groups.resize(numlabels);
	cutted = OriginalImage.clone();
	for (int i = 1; i < r-1; i++)
	{
		for (int j = 1; j < c-1; j++)
		{
			int n0 = klabels[i*c+j];
			int n1 = klabels[i*c+j-1];
			int n2 = klabels[i*c+j+1];
			int n3 = klabels[(i+1)*c+j];
			int n4 = klabels[(i-1)*c+j];
			int n5 = klabels[(i+1)*c+j+1];
			int n6 = klabels[(i+1)*c+j-1];
			int n7 = klabels[(i-1)*c+j+1];
			int n8 = klabels[(i-1)*c+j-1];
			if (n0 != n1 || n0 != n2 || n0 != n3 || n0 != n4 ||
				n0 != n5 || n0 != n6 || n0 != n7 || n0 != n8 )
			{
				cutted.at<Vec3b>(i,j)[0] = 0;
				cutted.at<Vec3b>(i,j)[1] = 0;
				cutted.at<Vec3b>(i,j)[2] = 188;

				int index = klabels[i*c+j];
				unsorted_kernal_groups[index].push_back(Point(i,j));
			}
		}
	}
	sortGroups(unsorted_kernal_groups);
	imwrite("cutted.png",cutted);
}

void BasicImage::sortGroups(std::vector<std::vector<Point>> us)
{
	int gsize = us.size();
	kernal_groups.resize(gsize);

	for (int i = 0; i < gsize; i++)
	{
		kernal_groups[i].push_back(us[i][0]);
		us[i].erase(us[i].begin());
		int xc = kernal_groups[i][0].x;
		int yc = kernal_groups[i][0].y;

		for (int j = 0; j < us[i].size(); j++)
		{
			int tmpx = us[i][j].x;
			int tmpy = us[i][j].y;

			if ((xc == tmpx && abs(yc-tmpy) == 1) || (yc == tmpy && abs(xc-tmpx) == 1))
			{
				kernal_groups[i].push_back(us[i][j]);
				xc = us[i][j].x;
				yc = us[i][j].y;
				us[i].erase(us[i].begin()+j);
				j = -1;
			}
		}
	}
}