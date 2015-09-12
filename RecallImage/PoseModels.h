#pragma once
#include <Eigen\Core>
#include <vector>

class PoseModels
{
public:
	PoseModels();
	PoseModels(const char* name, const char* labelfile, const char* skeleton, const char* weights, const char* edges);
	PoseModels(char* name, char* labelfile);
	~PoseModels(void);

	bool Applydeformation(Eigen::MatrixXd bc, Eigen::MatrixXd initial);
	void randomPose();
	void GenerateRecalls(const char * dir);

	void GenerateSpecificRecall(std::vector<double> angles, std::vector<Eigen::Vector3d> directions, int view_index, const char * dir);

	int groups;
	char* filename;
	Eigen::MatrixXd U,C,W,M;
	Eigen::MatrixXd V;
	Eigen::MatrixXi F,BE;
	Eigen::VectorXi G,P;
	Eigen::VectorXi F_label;
	Eigen::VectorXi b;
	Eigen::MatrixXd bc;
};

