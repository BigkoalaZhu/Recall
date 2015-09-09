#include "GenerateViewSpace.h"
#include "opencv2/shape.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core/utility.hpp>
#include <fstream>
#include "Eigen/Dense"
#include <fstream>

#include "qstring.h"
#include <QtCore/QTextStream>
#include <QtCore/QFile>
#include <QtCore/QIODevice>

#include "IDSC\shape_descriptor.hpp"
#include "FourierDescriptor.h" 

GenerateViewSpace::GenerateViewSpace(const Eigen::MatrixXd V, const Eigen::MatrixXi F, const Eigen::VectorXi L, int width, char * edges)
{
	Vertices = V;
	FaceIndex = F;
	Labels = L;
	Projected_Vertices = Eigen::MatrixXd::Zero(V.rows(), 2);
	Eigen::VectorXd bmax = V.colwise().maxCoeff();
	Eigen::VectorXd bmin = V.colwise().minCoeff();
	length = (bmax - bmin).norm();
	Width = width;
	loadCameras();

	edges_path = new char[1024];
	strcpy(edges_path, edges);

	//////////////////////////////Build color map
	Eigen::MatrixXd colormap = Eigen::MatrixXd::Zero(64,3);
	std::ifstream ifs("cameras/colormap.txt");
	for (int i = 0; i < 64; i++)
	{
		double tmp;
		ifs >> tmp;
		colormap(i,0) = tmp;
		ifs >> tmp;
		colormap(i,1) = tmp;
		ifs >> tmp;
		colormap(i,2) = tmp;
	}
	int numlabel = Labels.maxCoeff() + 1;
	int step = floor(64/numlabel);
	colors = new Eigen::Vector3i[numlabel];
	color_cv.resize(numlabel);
	for (int i = 0; i < numlabel; i++)
	{
		colors[i](0) = colormap(i*step,0)*255;
		colors[i](1) = colormap(i*step,1)*255;
		colors[i](2) = colormap(i*step,2)*255;
		color_cv[i] = cv::Scalar(colors[i](0), colors[i](1), colors[i](2));
	}
}

void GenerateViewSpace::loadCameras()
{
	camera_num = 200;
	camera_direction = new Eigen::Vector3d[camera_num];
	camera_up = new Eigen::Vector3d[camera_num];
	///////////////////////////////////////////////////////
	int vertex_num = 20;
	std::vector<Eigen::Vector3i> directions;
	std::ifstream ifs("cameras/edges.txt");
	for (int i = 0; i < vertex_num; i++)
	{
		int n,a,b,c;
		ifs >> n >> a >> b >> c;
		Eigen::Vector3i tmp(a-1,b-1,c-1);
		directions.push_back(tmp);
	}
	ifs.close();
	///////////////////////////////////////////////////////
	int filenums = 10;
	int index = 0;
	for (int i = 0; i < filenums; i++)
	{
		char filename[256] = "cameras/12_";
		char idx[25];
		_itoa_s(i, idx, 10);
		strcat_s(filename, sizeof(filename)/sizeof(char), idx);
		strcat_s(filename, sizeof(filename)/sizeof(char), ".obj");
		ifs.open(filename);
		std::vector<Eigen::Vector3d> vertices;
		for (int j = 0; j < vertex_num; j++)
		{
			char tmp;
			float x,y,z;
			ifs >> tmp >> x >> y >> z;
			Eigen::Vector3d pos(x,y,z);
			vertices.push_back(pos);
		}
		for (int j = 0; j < vertex_num; j++)
		{
			camera_direction[index] = vertices[j];
			Eigen::Vector3d tmp = vertices[j].cross(Eigen::Vector3d(0,1,0));
			camera_up[index] = (tmp.cross(vertices[j])).normalized();
			index ++;
		}
		ifs.close();
	}
}

void GenerateViewSpace::sweepTriangle(CvMat *depthMap, CvMat *labelMap, int label, Eigen::Vector3d *point, Eigen::Vector3i color, IplImage* I)
{
	int upMost,downMost;
	bool updated = false;
	int margin_x[2];
	float margin_z[2];
	for(int i=0;i<3;i++)
	{
		if(!updated)
		{
			updated = true;
			upMost = downMost = point[i][1];
		}
		else
		{
			upMost = (point[i][1] < upMost)? point[i][1] : upMost;
			downMost = (point[i][1] > downMost)? point[i][1] : downMost;
		}
	}
	for(int y=upMost;y<=downMost;y++)
	{
		int mIdx = 0;
		for(int i=0;i<3;i++)
		{
			Eigen::Vector3d &p1 = point[i];
			Eigen::Vector3d &p2 = point[(i+1)%3];
			float dy1 = p1[1] - y;
			float dy2 = p2[1] - y;
			if(dy1*dy2 <= 0)
			{
				margin_x[mIdx] = (dy2-dy1==0) ? p1[0] : p1[0]-dy1/(dy2-dy1)*(p2[0]-p1[0]);
				margin_z[mIdx] = (dy2-dy1==0) ? p1[2] : ceil(p1[2]-dy1/(dy2-dy1)*(p2[2]-p1[2]));
				mIdx++;
			}
		}
		if(mIdx!=2)
		{
			continue;
		}
		if(margin_x[0] == margin_x[1])
		{
			if(depthMap ->data.fl[y*depthMap->width+margin_x[0]] > margin_z[0])
			{
				depthMap->data.fl[y*depthMap->width+margin_x[0]] = margin_z[0];
				labelMap->data.fl[y*labelMap->width+margin_x[0]] = label;
				I->imageData[y*I->widthStep+3*margin_x[0]] = color[0];
				I->imageData[y*I->widthStep+3*margin_x[0]+1] = color[1];
				I->imageData[y*I->widthStep+3*margin_x[0]+2] = color[2];
			}
			continue;
		}
		int mx_s = (margin_x[0] < margin_x[1])? margin_x[0] : margin_x[1];
		int mx_e = (margin_x[0] >= margin_x[1])? margin_x[0] : margin_x[1];
		for(int x=mx_s;x<=mx_e;x++)
		{
			float z = (margin_x[1] == margin_x[0]) ? margin_z[0] : margin_z[0]+(x-margin_x[0])/(margin_x[1]-margin_x[0])*(margin_z[1]-margin_z[0]);
			if(depthMap ->data.fl[y*depthMap->width+x] > z)
			{
				depthMap ->data.fl[y*depthMap->width+x] = z;
				I->imageData[y*I->widthStep+3*x] = color[0];
				I->imageData[y*I->widthStep+3*x+1] = color[1];
				I->imageData[y*I->widthStep+3*x+2] = color[2];

				if(label != labelMap ->data.fl[y*labelMap->width+x] && labelMap ->data.fl[y*labelMap->width+x] >= 0)
				{
					layout.insert(std::pair<int,int>(label, labelMap->data.fl[y*labelMap->width+x]));
					labelMap ->data.fl[y*labelMap->width+x] = label;
				}
			}
			else if (label != labelMap ->data.fl[y*labelMap->width+x] && labelMap ->data.fl[y*labelMap->width+x] >= 0)
			{
				layout.insert(std::pair<int,int>(labelMap->data.fl[y*labelMap->width+x], label));
			}
		}
	}
}

void GenerateViewSpace::Generate(char* dir)
{
	int centerNum = 200;
	connected_edges.clear();
	std::ifstream ifs(edges_path);
	while (!ifs.eof())
	{
		char tmp;
		int x,y;
		ifs >> tmp >> x >> y;
		connected_edges.push_back(std::pair<int,int>(x,y));
		connected_edges.push_back(std::pair<int,int>(y,x));
	}
	ifs.close();
//	#pragma omp parallel for
	for (int i = 0; i < camera_num; i++)
	{
		Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(Vertices.rows(), Vertices.cols());
		Eigen::MatrixXd result = Eigen::MatrixXd::Zero(Vertices.rows(), Vertices.cols());
		for (int j = 0; j < Vertices.rows(); j++)
		{
			Eigen::Vector3d vertex = Vertices.row(j);
			
			tmp(j,0) = (vertex - camera_direction[i]*length).dot(camera_direction[i].cross(camera_up[i]));
			tmp(j,1) = (vertex - camera_direction[i]*length).dot(camera_up[i]);
			tmp(j,2) = (vertex - camera_direction[i]*length).dot(-camera_direction[i]);
		}

		Eigen::Vector3d boundaryMax, boundaryMin, boundaryL, boundaryC;
		boundaryMax = tmp.colwise().maxCoeff();
		boundaryMin = tmp.colwise().minCoeff();
		boundaryL = boundaryMax - boundaryMin;
		boundaryC = (boundaryMax + boundaryMin)/2;
		double scale = 1.05*boundaryL.maxCoeff()/Width;
		double offset_x = Width/2.0f - boundaryC[0]/scale;
		double offset_y = Width/2.0f - boundaryC[1]/scale;

		for (int j = 0; j < Vertices.rows(); j++)
		{
			result(j,0) = tmp(j,0)/scale + offset_x;
			result(j,1) = Width - tmp(j,1)/scale - offset_y;
			result(j,2) = -tmp(j,2)/scale;
		}

		Eigen::VectorXi valid = Eigen::VectorXi::Ones(FaceIndex.rows());

		CvMat *depthMap = cvCreateMat(Width,Width,CV_32FC1);
		CvMat *labelMap = cvCreateMat(Width,Width,CV_32FC1);
		IplImage* image = cvCreateImage(cvGetSize(depthMap),8,3);

		for(int j = 0; j < Width*Width; j++)
		{
			depthMap->data.fl[j] = std::numeric_limits<float>::infinity();
			labelMap->data.fl[j] = -1;
			image->imageData[3*j] = 255;
			image->imageData[3*j+1] = 255;
			image->imageData[3*j+2] = 255;
		}

		layout.clear();

		for (int j = 0; j < FaceIndex.rows(); j++)
		{
			Eigen::Vector3d points[3];
			Eigen::Vector3i c = colors[Labels(j)];
			for (int k = 0; k < 3; k++)
			{
				points[k] = result.row(FaceIndex(j,k));
			}
			sweepTriangle(depthMap, labelMap, Labels(j), points, c, image);
		}

		char filename[256];
		char idscname[256];
		char partsname[256];
		char idx[25];
		_itoa_s(i, idx, 10);
		strcpy_s(filename,dir);
		strcat_s(filename, sizeof(filename)/sizeof(char),idx);
		strcpy_s(partsname,filename);
		strcpy_s(idscname,filename);
		strcat_s(partsname,sizeof(partsname)/sizeof(char), "_p");
		strcat_s(filename, sizeof(filename)/sizeof(char),".png");
		strcat_s(idscname, sizeof(filename)/sizeof(char),".txt");

		cvFloodFill(image, CvPoint(0,0), cvScalarAll(0),cvScalarAll(0),cvScalarAll(0),NULL,8,NULL);

		cvSaveImage(filename, image);

		///////////////////////////////////////////////
		QString sFilePath = QString::fromUtf8(idscname);
		QFile file(sFilePath);  

		if (!file.open(QIODevice::WriteOnly|QIODevice::Text)) {    
			return;    
		}
		QTextStream out(&file);
		cv::Mat mat_c = cv::cvarrToMat(image);
		cvtColor(mat_c, mat_c, CV_BGR2GRAY);
		cv::threshold(mat_c,mat_c,5,255,THRESH_BINARY);
		imwrite("test.jpg",mat_c);

		int pt_num = 300;
		double *descriptor = new double[pt_num*64];
		IDSC_descriptor idsc;
		idsc.getShapeContext(descriptor, mat_c, pt_num);

		for (int i = 0; i < pt_num; i++)
		{
			for (int j = 0; j < 64; j++)
			{
				out << descriptor[i*64 + j] << " ";
			}
			out << endl;
		}
//		out.flush();    
		file.close();
		///////////////////////////////////////////////

		DrawParts(result, partsname);

		cvReleaseMat(&labelMap);
		cvReleaseMat(&depthMap);
		cvReleaseImage(&image);
	}
}

void GenerateViewSpace::DrawParts(Eigen::MatrixXd position, char * path)
{
	std::vector<Mat> parts;
	parts.resize(color_cv.size());

	QString sFilePath = QString::fromUtf8(path);
	sFilePath += "art.txt";
	QFile file(sFilePath);  

	if (!file.open(QIODevice::WriteOnly|QIODevice::Text)) {    
		return;    
	}
	QTextStream out(&file);

	int descriptor_length = 30;

	for (int i = 0; i < parts.size(); i++)
		parts[i] = Mat(Width, Width, CV_8UC1, Scalar(0));

	for (int j = 0; j < FaceIndex.rows(); j++)
	{
		Point pt[1][3];
		int arr[] = { 3 };   
		Eigen::Vector3i c = colors[Labels(j)];
		for (int k = 0; k < 3; k++)
		{
			pt[0][k] = cvPoint(position(FaceIndex(j,k),0), position(FaceIndex(j,k),1));
		}

		const Point* ppt[1] = {pt[0]};
		fillPoly(parts[Labels(j)],ppt,arr,1,Scalar(255));
	}

	for (int i = 0; i < parts.size(); i++)
	{
		QString str = QString::fromUtf8(path);
		str += QString::number(i);
		str += "_";

		out<<"Part "<< i <<endl;
		out<<"Upper";

		std::set<std::pair<int,int>>::iterator iter = layout.begin();
		for (int j = 0; j < layout.size(); j++)
		{
			if(iter->first == i)
			{
				str += "-";
				str += QString::number(iter->second);
				out << " " << iter->second;
			}
			iter++;
		}

		out << endl << "Lower";
		str += "_";
		iter = layout.begin();
		for (int j = 0; j < layout.size(); j++)
		{
			if(iter->second == i)
			{
				str += "-";
				str += QString::number(iter->first);
				out << " " << iter->first;
			}
			iter++;
		}

		out << endl << "Connect";
		for (int j = 0; j < connected_edges.size(); j++)
		{
			if (connected_edges[j].first == i)
			{
				out << " " << connected_edges[j].second;
			}
		}

		str += ".png";
//		imwrite(str.toStdString().c_str(), parts[i]);

		out << endl << "Descriptor";
		double *descriptor = new double[descriptor_length];
		FourierDescriptor::calculate(parts[i], descriptor, descriptor_length);
//		IDSC_descriptor idsc;
//		idsc.getShapeContext(descriptor, parts[i], descriptor_length);
		for (int j = 0; j < descriptor_length; j++)
			out << " " << descriptor[j];
		out << endl << endl;
	}
	parts.clear();

	out.flush();    
	file.close();
}


GenerateViewSpace::~GenerateViewSpace(void)
{
}
