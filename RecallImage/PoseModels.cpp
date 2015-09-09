#include "PoseModels.h"
#define IGL_OMP_MIN_VALUE 1000000

#include <igl/colon.h>
#include <igl/directed_edge_orientations.h>
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
#include <igl/PI.h>
#include <igl/partition.h>
#include <igl/mat_max.h>
#include <igl/lbs_matrix.h>
#include <igl/slice.h>
#include <igl/deform_skeleton.h>
#include <igl/dqs.h>
#include <igl/lbs_matrix.h>
#include <igl/columnize.h>
#include <igl/readDMAT.h>
#include <igl/readTGF.h>
#include <igl/readOFF.h>
#include <igl/svd3x3/arap.h>
#include <igl/svd3x3/arap_dof.h>
#include <igl/writeOFF.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>

#include "GenerateViewSpace.h"

typedef 
  std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> >
  RotationList;

igl::ARAPData arap_grouped_data;
RotationList rest_pose;

PoseModels::PoseModels()
{

}

PoseModels::PoseModels(const char* name, const char* labelfile, const char* skeleton, const char* weights, const char* edges)
{
	igl::readOFF(name,V,F);
	G.resize(V.rows());
	F_label = Eigen::VectorXi::Zero(F.rows());
	std::ifstream ifs(labelfile);
	int idx_tmp = 0;
	groups = 0;

	filename = new char[1024];
	strcpy(filename, edges);

	while(!ifs.eof())
	{
		int label;
		ifs >> label;
		if (label > groups)
			groups = label;
		G[F(idx_tmp,0)] = label + 1;
		G[F(idx_tmp,1)] = label + 1;
		G[F(idx_tmp,2)] = label + 1;
		F_label(idx_tmp) = label;
		idx_tmp++;
	}

	ifs.close();

	////////////////////////////////////////////////////////////////
	Eigen::MatrixXd tmpW;
	igl::readTGF(skeleton,C,BE);
	igl::readDMAT(weights,tmpW);
	W = tmpW.transpose();
	igl::lbs_matrix(V,W,M);
	igl::directed_edge_parents(BE,P);
	igl::directed_edge_orientations(C,BE,rest_pose);
	U = V;
	srand((unsigned)time(NULL));
}

PoseModels::PoseModels(char* name, char* labelfile)
{
	igl::readOFF(name,V,F);
	G.resize(V.rows());
	F_label = Eigen::VectorXi::Zero(F.rows());
	std::ifstream ifs(labelfile);
	int idx_tmp = 0;
	groups = 0;

	while(!ifs.eof())
	{
		int label;
		ifs >> label;
		if (label > groups)
			groups = label;
		G[F(idx_tmp,0)] = label + 1;
		G[F(idx_tmp,1)] = label + 1;
		G[F(idx_tmp,2)] = label + 1;
		F_label(idx_tmp) = label;
		idx_tmp++;
	}

	ifs.close();
	////////////////////////////////////////////////////////////////
	b.resize(groups+1);
	for (int i = 0; i < V.rows(); i++)
		b[G[i]-1] = i;
	bc = Eigen::MatrixXd::Zero(b.size(),V.cols());
	for (int i = 0; i < b.size(); i++)
	{
		bc(i, 0) = V(b[i],0);
		bc(i, 1) = V(b[i],1);
		bc(i, 2) = V(b[i],2);
	}
	arap_grouped_data.max_iter = 2;
	arap_grouped_data.G = G;
	bool suc = igl::arap_precomputation(V,F,V.cols(),b,arap_grouped_data);
	if(!suc)
		printf("Error in precomputation model");
	////////////////////////////////////////////////////////////////
	U = V;
}

void PoseModels::randomPose()
{
	using namespace Eigen;
	using namespace std;
	RotationList pose = RotationList(rest_pose.size(),Quaterniond::Identity());
	////////////////////////////////////////////
	int random_num = 3;
	int *idx = new int[3];
	for (int i = 0; i < random_num; i++)
	{
		idx[i] = rand()%rest_pose.size();
		if (idx[i] < 5)
		{
			i--;
			continue;
		}
		Vector3d direction = Vector3d::Random();
		direction.normalize();
		double angle;
		/////////////////////////////////////////////////////
		direction = Vector3d(0,1,0);
//		if (idx[i] > 9)
//			angle = igl::PI*((rand()%80)/100.0f);
//		else
			angle = -igl::PI*((rand()%50)/100.0f);
//		if(rand()%2 == 0 && idx[i] < 10)
//			angle = -angle;
		/////////////////////////////////////////////////////

		const Quaterniond bend(AngleAxisd(angle,direction));
		pose[idx[i]] = rest_pose[idx[i]]*bend*rest_pose[idx[i]].conjugate();
	}

	RotationList vQ;
	vector<Vector3d> vT;
	igl::forward_kinematics(C,BE,P,pose,vQ,vT);
	const int dim = C.cols();
	MatrixXd T(BE.rows()*(dim+1),dim);
	for(int e = 0;e<BE.rows();e++)
	{
		Affine3d a = Affine3d::Identity();
		a.translate(vT[e]);
		a.rotate(vQ[e]);
		T.block(e*(dim+1),0,dim+1,dim) =
			a.matrix().transpose().block(0,0,dim+1,dim);
	}
	igl::dqs(V,W,vQ,vT,U);
}

bool PoseModels::Applydeformation(Eigen::MatrixXd BC, Eigen::MatrixXd initial)
{
	U = initial;
	bool suc = igl::arap_solve(BC,arap_grouped_data,U);
	return suc;
}

void PoseModels::GenerateRecalls(const char * dir)
{
	GenerateViewSpace viewer(U, F, F_label, 512,filename);
	char gd[1024];
	strcpy(gd, dir);
	viewer.Generate(gd);
}

PoseModels::~PoseModels(void)
{

}
