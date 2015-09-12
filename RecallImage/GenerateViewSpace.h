#pragma once

#include <Eigen/Core>
#include <vector>
#include <fstream>
#include <set>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class GenerateViewSpace
{
public:
	GenerateViewSpace(const Eigen::MatrixXd V, const Eigen::MatrixXi F, const Eigen::VectorXi L, int width, char * edges); // V = vertex positions, F = Faces, L = labels, width = width of recall image, edges = connection file name
	~GenerateViewSpace(void);

	void Generate(char* dir); // Generate recall images from all views in this pose
	void DrawParts(Eigen::MatrixXd position, char * path); // Generate parts info

	void GenerateSingleRecall(char* dir, int index);

private:
	Eigen::MatrixXd Vertices;
	Eigen::MatrixXi FaceIndex;
	Eigen::MatrixXd Projected_Vertices;
	Eigen::VectorXi Labels;
	char * edges_path;
	
	double length;
	int Width;

	int camera_num;
	Eigen::Vector3d *camera_direction;
	Eigen::Vector3d *camera_up;
	Eigen::Vector3d camera_direction_tmp;
	Eigen::Vector3d camera_up_tmp;

	Eigen::Vector3i * colors;
	std::vector<cv::Scalar> color_cv;
	std::set<std::pair<int,int>> layout;
	std::vector<std::pair<int,int>> connected_edges;
	

	void loadCameras();
	void sweepTriangle(CvMat *depthMap, CvMat *labelMap, int label, Eigen::Vector3d *point, Eigen::Vector3i color, IplImage* I);
};

