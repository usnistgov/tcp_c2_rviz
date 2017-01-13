// UrdfKinematics.cpp

#include "UrdfKinematics.h"
#include "Globals.h"

UrdfKinParameters::UrdfKinParameters()
{
}

MatrixEXd ComputeUrdfTransform(double angle, Eigen::Vector3d axis, Eigen::Vector3d origin, Eigen::Vector3d rotation)
{
	Eigen::Matrix3d t33;
	MatrixEXd m1=Create4x4IdentityMatrix();
	MatrixEXd tmp=Create4x4IdentityMatrix();

	Eigen::Vector3d unit = axis.array().abs();
	t33 = Eigen::AngleAxisd(axis.sum() * angle, unit );
	m1.block<3,3>(0,0) =t33;
	tmp.block<3,1>(0,3) = origin;
	return  tmp * m1 ;  // i dont understand why this matrix multiply order works!
}
tf::Pose  UrdfKinParameters::ComputeFk (EJointVector thetas)
{
	std::vector<MatrixEXd> AllM;
	MatrixEXd t=Create4x4IdentityMatrix();
	A0.clear();
	EJointVector joints(thetas);
	joints(2) =  thetas[1] + thetas[2] ;

	for ( int i = 0; i< thetas.size( ); i++ )
	{
		AllM.push_back(ComputeUrdfTransform(joints(i), jointspec[i].axis, jointspec[i].xyzorigin, jointspec[i].rpyorigin));
	}

	for ( int i = 0; i< thetas.size( ); i++ )
	{
		t = t* AllM[i];
		A0.push_back(t);
	}

	EigenPose t0_6pose =  EMatrix2Pose(t);
	return t0_6pose;
}

// See http://docs.ros.org/api/moveit_core/html/robot__state_8cpp_source.html
// Has jacobian computation from quaterian

MatrixEXd UrdfKinParameters::ComputeEigenJacobian( std::vector<MatrixEXd> &A0)
{
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Jacobian;
	Jacobian = Eigen::MatrixXd::Zero(6, A0.size()); // .resize(6,6); // FIXME: hardcoded for now

	// A0i = A01*A12*A23...Ai
	Eigen::Vector3d z0, p0, pe;
	z0  << 0, 0, 1;  // should this change for urdf?
	p0  << 0, 0, 0;
	pe = A0.back().block<3,1>(0,3); 

	for(size_t i=0; i < A0.size(); i++)
	{
		if(jointspec[i].type == UrdfKinParameters::UrdfJoint::REVOLUTE)
		{
			// Ji linear velocity
			Jacobian.block<3,1>(0,i) = (A0[i].block<3,3>(0,0)*jointspec[i].axis).cross(pe-A0[i].block<3,1>(0,3));
			// Ji angular velocity
			Jacobian.block<3,1>(3,i) = (A0[i].block<3,3>(0,0)*jointspec[i].axis);	
		}
		else if(jointspec[i].type == UrdfKinParameters::UrdfJoint::PRISMATIC)
		{
			// Ji linear velocity
			Jacobian.block<3,1>(0,i) = (A0[i].block<3,3>(0,0)*jointspec[i].axis);
			// Ji angular velocity - None
			Jacobian.block<3,1>(3,i) = Eigen::Vector3d(0,0,0);	
		}		
		else
			assert(0);
	}

	return Jacobian;
}
std::vector<double> UrdfKinParameters::ComputeNewtonIK(EigenPose endpose, std::vector<double> joints )
{
	//JointVector thetas;
	std::vector<double> qk=joints;
	std::vector<double> qk1=joints;
	int nIterations=0;
	tf::Pose currentpose,errpose;
	Eigen::VectorXd err(7);
	double errthreshold=0.0001;  // 0.0001 error is  1 mm
	MatrixEXd jInv; 

	// reset logging file
	std::stringstream s;
	Globals.WriteFile(Globals.ExeDirectory+"ComputeNewtonIK.txt", std::string(""));
	do {
		std::cout << ".";
		qk=qk1;
		currentpose = ComputeFk (qk);
		for(size_t i=0; i < A0.size(); i++)
			s << "A[" << i << "]=\n" << DumpEMatrix(A0[i]).c_str();
		_Jacobian=ComputeEigenJacobian(A0);

		// pseudo inverse if non-square
		if(_Jacobian.rows() == _Jacobian.cols())
			jInv=  _Jacobian.inverse();
		else
			jInv=PseudoInvertJacobian(_Jacobian);

		err=EErrPosesAxisAngle(currentpose, endpose); 
		qk1=qk + (jInv*err);
		//joints = ConvertJoints(qk1);
		double epsilon = err.norm();
		nIterations++;

#ifdef _DEBUG
		s << "Jacobian \n" << DumpEMatrix(_Jacobian).c_str();
		s << "Inverse Jacobian \n" << DumpEMatrix(jInv).c_str();
		//s << "Jacobian Inverse \n" << DumpEMatrix(jInv).c_str();
		s << "endpose=\n" << DumpEPose(endpose).c_str() ;
		s << "currentpose=\n" << DumpEPose(currentpose).c_str() ;
		s << "errvec=" << err ;
		s << "Old joints=\n" << DumpEJoints(qk).c_str();
		s << "New joints=\n" << DumpEJoints(qk1).c_str();
		s << "epsilon=" << epsilon << " Iterations=" << nIterations << std::endl;
		s << "------------------------------------------" << std::endl;
		Globals.AppendFile(Globals.ExeDirectory + "ComputeNewtonIK.txt", s.str( ) );
		s.str("");
#endif
	}
	while(err.norm() > errthreshold && nIterations<1000); 
	if(nIterations==1000)
		throw std::exception("ComputeNewtonIK failed to converge");
	return qk1;
}
EJointVector UrdfKinParameters::ComputeGradiantIK(EigenPose endpose, EJointVector joints )
{
	double errthreshold=0.001;  // 0.0001 error is  1 mm
	//std::vector<double> thetas;
	//thetas.insert(thetas.end(), joints.begin(), joints.end());
	EJointVector qk=joints;
	EJointVector qk1=joints;
	double alpha = 0.1;
	int nIterations=0;
	Eigen::VectorXd err(6);
	EigenPose currentpose,errpose;
	MatrixEXd jT; 

	// reset logging file
	std::stringstream s;
	Globals.WriteFile(Globals.ExeDirectory+"ComputeGradiantIK.txt", std::string(""));
	do {
		std::cout << ".";
		qk=qk1;
		currentpose = ComputeFk (qk);
		_Jacobian=ComputeEigenJacobian(A0);
		jT= _Jacobian.transpose();
		err=EErrPosesAxisAngle(currentpose, endpose);
		qk1=qk + alpha * (jT*err);
		double epsilon = err.norm();
		nIterations++;

#ifdef _DEBUG
		s << "Jacobian \n" << DumpEMatrix(_Jacobian).c_str();
		s << "Jacobian Transpose \n" << DumpEMatrix(jT).c_str();
		s << "endpose=\n" << DumpEPose(endpose).c_str() ;
		s << "currentpose=\n" << DumpEPose(currentpose).c_str() ;
		//s << "err orientation=" << errrot.x << ":" << errrot.y << ":" << errrot.z << std::endl ;
		//s << "Old joints=\n" << ConvertJoints(qk);
		//s << "New joints=\n" << ConvertJoints(qk1);
		s << "epsilon=" << epsilon << " Iterations=" << nIterations << std::endl;
		s << "------------------------------------------" << std::endl;
		Globals.AppendFile(Globals.ExeDirectory + "ComputeGradiantIK.txt", s.str( ) );
		s.str("");
#endif
	}
	while(err.norm() > errthreshold && nIterations<1000);

	//if(nIterations==1000)
	//	throw std::exception("ComputeGradiantIK failed to converge");

#ifdef _DEBUG
	s << "initial joints=\n" << joints;
	s << "Final joints=\n" << qk1;
	Globals.AppendFile(Globals.ExeDirectory + "ComputeGradiantIK.txt", s.str( ) );
#endif
	return qk1;
}
// http://docs.ros.org/api/moveit_core/html/robot__state_8cpp_source.html 
void UrdfKinParameters::ComputeVariableVelocity(Eigen::VectorXd &qdot, const Eigen::VectorXd &twist) 
{
	//Get the Jacobian of the group at the current configuration
	Eigen::MatrixXd J(6, GetJointNum());
	Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
	//getJacobian(jmg, tip, reference_point, J, false);
	J=ComputeEigenJacobian(A0); // last computation of A0

	//Do the Jacobian moore-penrose pseudo-inverse
	Eigen::JacobiSVD<Eigen::MatrixXd> svdOfJ(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
	const Eigen::MatrixXd U = svdOfJ.matrixU();
	const Eigen::MatrixXd V = svdOfJ.matrixV();
	const Eigen::VectorXd S = svdOfJ.singularValues();

	Eigen::VectorXd Sinv = S;
	static const double pinvtoler = std::numeric_limits<float>::epsilon();
	double maxsv = 0.0 ;
	for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i)
		if (fabs(S(i)) > maxsv) maxsv = fabs(S(i));
	for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i)
	{
		//Those singular values smaller than a percentage of the maximum singular value are removed
		if (fabs(S(i)) > maxsv * pinvtoler)
			Sinv(i) = 1.0 / S(i);
		else Sinv(i) = 0.0;
	}
	Eigen::MatrixXd Jinv = (V * Sinv.asDiagonal() * U.transpose());

	// Compute joint velocity
	qdot = Jinv * twist;
}



void UrdfKinParameters::SaveJointInfoFromUrdf(urdf::JointSharedPtr jointlink, urdf::LinkConstSharedPtr rootlink, std::vector<UrdfJoint> &injointspec)
{
	UrdfKinParameters::UrdfJoint jointspec;
	jointspec.pJoint=jointlink;
	jointspec.name=jointlink->name;
	jointspec.index=injointspec.size();
	jointspec.type=(UrdfKinParameters::UrdfJoint::JointType) jointlink->type;
	jointspec.axis=ConvertPosition(jointlink->axis);
	jointspec.bounded=false;
	if(jointlink->limits!=NULL)
	{
		jointspec.lowerlimit=jointlink->limits->lower;
		jointspec.upperlimit=jointlink->limits->upper;
		jointspec.bounded=true;
		if(jointspec.type == UrdfKinParameters::UrdfJoint::CONTINUOUS) 
			jointspec.bounded=false;
	}
#define WORKS
#ifdef WORKS
	jointspec.xyzorigin = ConvertPosition( jointlink->parent_to_joint_origin_transform.position);

	// This seems to fail
	//Eigen::Quaterniond q = ConvertQuaterion(jointlink->parent_to_joint_origin_transform.rotation);
	//EQuatToRpy(q, jointspec.rpyorigin(0),jointspec.rpyorigin(1),jointspec.rpyorigin(2));

	jointlink->parent_to_joint_origin_transform.rotation.getRPY (jointspec.rpyorigin(0),jointspec.rpyorigin(1),jointspec.rpyorigin(2)) ;
	//std::cout << "RPY " <<   jointspec.rpyorigin(0) << ":" << jointspec.rpyorigin(1) << ":" <<jointspec.rpyorigin(2) << std::endl;
#else 
	EigenPose origin = ConvertPose(jointlink->parent_to_joint_origin_transform);
	jointspec.xyzorigin << origin(0), origin(1), origin(2);
	EQuatToRpy(Eigen::Quaterniond(origin(6),origin(3), origin(4), origin(5) ), 
		jointspec.rpyorigin(0),jointspec.rpyorigin(1),jointspec.rpyorigin(2));
#endif

	injointspec.push_back(jointspec);

}
// assume single chain - not tree of link/joint definitions
void UrdfKinParameters::GetAllJointInfo(urdf::LinkConstSharedPtr link)
{
	prejointspec.clear(); jointspec.clear(); postjointspec.clear();
	prejointpose.clear();postjointpose.clear();prejointposeinv.clear();postjointposeinv.clear();
	// Get fixed links - use recursive descent assuming chain of link/joints
	std::vector<urdf::LinkSharedPtr>::const_iterator child = link->child_links.begin( );
	std::vector<urdf::LinkSharedPtr>::const_iterator endchild = link->child_links.end( );
	for ( ; child != endchild;  )
	{
		 urdf::JointSharedPtr jchild = (*child)->parent_joint;
		// look for fixed "joints" - premultiplied to goal pose
		if(jchild==NULL)
		{
			child++;
			continue;
		}
		if(jchild->type == urdf::Joint::FIXED )
		{
			SaveJointInfoFromUrdf(jchild,  *child, prejointspec);
			child++;
			continue;
		}
		else
		{
			break;	
		}
	}
	for ( ; child != endchild;  )
	{
		 urdf::JointSharedPtr jchild = (*child)->parent_joint;
		// look for fixed "joints" - premultiplied to goal pose
		assert(jchild!=NULL);
		assert (jchild->type != urdf::Joint::UNKNOWN);
		if(jchild->type != urdf::Joint::FIXED )
		{
			SaveJointInfoFromUrdf(jchild,  *child, jointspec);
			endchild = (*child)->child_links.end( );
			child = (*child)->child_links.begin( );
			continue;
		}
		else
		{
			break;	
		}
	}
	for ( ;child != endchild; )
	{
		 urdf::JointSharedPtr jchild = (*child)->parent_joint;

		 // look for fixed "joints" - postmultiplied to goal pose
		assert(jchild!=NULL);
		assert (jchild->type != urdf::Joint::UNKNOWN);
		if(jchild->type == urdf::Joint::FIXED )
		{
			SaveJointInfoFromUrdf(jchild,  *child, postjointspec);
			child++;
		}
		else
		{
			// there may be more, but we're done
			break;	
		}
	}
	// premultiply in reverse order - so add back to front
	for(int  i=prejointspec.size()-1 ; i>=0;  i--)
	{
		prejointpose.push_back( ECreatePose(prejointspec[i].xyzorigin, 
			//CreateQuaterian(prejointspec[i].rpyorigin(0), prejointspec[i].rpyorigin(1), prejointspec[i].rpyorigin(2))));
			EQuatFromRpy(prejointspec[i].rpyorigin(0), prejointspec[i].rpyorigin(1), prejointspec[i].rpyorigin(2) )));
	}
	for(size_t i=0; i< postjointspec.size() ; i++)
	{
		postjointpose.push_back( ECreatePose(postjointspec[i].xyzorigin, 
			CreateQuaterian(postjointspec[i].rpyorigin(0), postjointspec[i].rpyorigin(1), postjointspec[i].rpyorigin(2))));
			//EQuatFromRpy(postjointspec[i].rpyorigin(0), postjointspec[i].rpyorigin(1), postjointspec[i].rpyorigin(2) )));
	}
	for(size_t i=0; i< prejointpose.size() ; i++)
		prejointposeinv.push_back(EPoseInv(prejointpose[i]));
	for(size_t i=0; i< postjointpose.size() ; i++)
		postjointposeinv.push_back(EPoseInv(postjointpose[i]));
}
void UrdfKinParameters::GetJointInfo(urdf::LinkConstSharedPtr link,std::vector<urdf::JointSharedPtr> &  joints)
{
	static char * jointtypes[] = {     "UNKNOWN", "REVOLUTE", "CONTINUOUS", "PRISMATIC", "FLOATING", "PLANAR", "FIXED"};
	int count=0;
	for ( std::vector<urdf::LinkSharedPtr>::const_iterator child = link->child_links.begin( ); child != link->child_links.end( ); child++ )
	{
		if ( *child )
		{
			std::cout << "child(" << ( count++ ) + 1 << "):  " << ( *child )->name.c_str() << std::endl;
			for ( std::vector<urdf::JointSharedPtr>::const_iterator jchild = link->child_joints.begin( ); jchild != link->child_joints.end( ); jchild++ )
			{
				std::cout << (*jchild)->name.c_str() << " Type : " << jointtypes[(*jchild)->type] << std::endl; //  " Child:" << (*jchild)->child_link_name.c_str() << " Parent:" << (*jchild)->parent_link_name.c_str() << std::endl; 
				if((*jchild)->type != urdf::Joint::UNKNOWN && (*jchild)->type != urdf::Joint::FIXED )
				{
					std::string name = (*jchild)->name;
					if( std::find_if(joints.begin(), joints.end(),
						[name] (urdf::JointSharedPtr item) { 
							return item->name == name;
					}) == joints.end()) 
						joints.push_back(*jchild);
				}
			}
			// first grandchild
			GetJointInfo(*child, joints);
		}
		else
		{
			std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
		}
	}
}

// ------------------------------------------------------------------------------
UrdfFanucLRMate200id::UrdfFanucLRMate200id()
{
	MatrixEXd t6_7;
	T7roll = CreateMatrix(M_PI, Eigen::Vector3d(1,0,0)); 
	T7pitch = CreateMatrix(-M_PI_2, Eigen::Vector3d(0,1,0)); 
	t6_7 =    T7pitch *  T7roll  ;
	Et7=EMatrix2Pose(t6_7);
	Et7_inv=EPoseInv(Et7);
	
	MatrixEXd zoffset = CreateMatrix(EigenPosition (0,0,-0.33));
	Et0Pre=EMatrix2Pose(zoffset);
	Et0Pre_inv=EPoseInv(Et0Pre);
}
EigenPose  UrdfFanucLRMate200id::ComputeFk (EJointVector thetas)
{
	EigenPose t0_6pose = urdfkin.ComputeFk (thetas);

#ifdef HANDCRAFT
	EigenPose goal = EPoseMult(Et0Pre  , t0_6pose);
	goal = EPoseMult( goal, Et7);
#else
	EigenPose goal(  t0_6pose);
	for(size_t i=0; i< urdfkin.prejointpose.size(); i++)
	{
		goal = EPoseMult(urdfkin.prejointposeinv[i]  , goal);
	}
	for(size_t i=0; i< urdfkin.postjointpose.size(); i++)
	{
		goal = EPoseMult( goal, urdfkin.postjointpose[i] );
		//std::cout << "Post" << DumpEPose(urdfkin.postjointpose[i]).c_str() ;
		//std::cout << "PostInv" << DumpEPose(urdfkin.postjointposeinv[i]).c_str() ;
		//std::cout << "Et7" << DumpEPose(Et7).c_str() ;
	}
#endif
	return goal;
}
EJointVector UrdfFanucLRMate200id::ComputeIK(EigenPose endpose, EJointVector joints )
{
	EigenPose goal(endpose);
	// Don't Handle gearing of joints - would be handled twice
	// add in z offset in urdf
	for(size_t i=0; i< urdfkin.prejointpose.size(); i++)
	{
		goal = EPoseMult(urdfkin.prejointpose[i] , goal);
	}
	// take off the end frame rotation
	for(size_t i=0; i< urdfkin.postjointpose.size(); i++)
		goal = EPoseMult( goal, urdfkin.postjointposeinv[i] );

	//return ComputeNewtonIK(goal, joints );
	return urdfkin.ComputeGradiantIK(goal, joints );
}

void UrdfFanucLRMate200id::GetAllJointInfo(urdf::LinkConstSharedPtr link)
{
	urdfkin.GetAllJointInfo(link);
}
