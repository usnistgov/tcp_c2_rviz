// UrdfKinematics.h

/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I.
*/

#pragma once

#include <vector>
#include <Eigen/Dense>
#include <boost\format.hpp>
#include "urdf_model\RobotModel.h"
#include "urdf_model\eigenmath.h"

//class UrdfJointModel
//{
//	std::string name();
//	int index();
//	enum JointType
//		{
//			UNKNOWN = 0, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
//		} type;
//	Eigen::Vector3d axis();
//	Eigen::Vector3d xyzorigin();
//	Eigen::Vector3d rpyorigin();
//	double lowerlimit();
//	double upperlimit();
//	double effortlimit();
//	double velocitylimit();
//	bool bounded();
//};
class UrdfKinParameters
{
public:
	UrdfKinParameters();
	~UrdfKinParameters() {}
	tf::Pose            ComputeFk (std::vector<double>);
	MatrixEXd           ComputeEigenJacobian( std::vector<MatrixEXd> &A0);

	EJointVector        ComputeNewtonIK(tf::Pose endpose, std::vector<double> joints );
	EJointVector        ComputeGradiantIK(tf::Pose endpose, std::vector<double> joints );

	struct UrdfJoint
	{
		std::string name;
		int index;
		urdf::LinkConstSharedPtr pLink;
		urdf::JointSharedPtr pJoint;
		enum JointType
		{
			UNKNOWN = 0, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
		} type;
		Eigen::Vector3d axis;
		Eigen::Vector3d xyzorigin;
		Eigen::Vector3d rpyorigin;
		double lowerlimit;
		double upperlimit;
		double effortlimit;
		double velocitylimit;
		bool bounded;
		std::string DumpUrdfJoint();
	};

	std::vector<tf::Pose> prejointpose;
	std::vector<tf::Pose> postjointpose;
	std::vector<tf::Pose> prejointposeinv;
	std::vector<tf::Pose> postjointposeinv;

protected:
	std::vector<MatrixEXd> A0;
	MatrixEXd _Jacobian;
	MatrixEXd InvJacobian;
};
#if 0
class UrdfFanucLRMate200id
{
public:
	UrdfFanucLRMate200id();
	EigenPose           ComputeFk (EJointVector thetas);
	EJointVector        ComputeIK(EigenPose endpose, EJointVector joints );
	//void                GenerateJointInfoFromUrdf(urdf::LinkConstSharedPtr rootlink);
	void                GetAllJointInfo(urdf::LinkConstSharedPtr link);

	UrdfKinParameters   urdfkin;
	EigenPose Et0Pre, Et7;
	EigenPose  Et7_inv, Et0Pre_inv;
	MatrixEXd T7roll,T7pitch;
};
#endif
