

#pragma once

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#include <stdarg.h>
#include <vector>
#include <boost/format.hpp>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>

#include "Conversions.h"
#ifndef BOOST_LOGGING_H
#define LOG_DEBUG std::cout
#endif
typedef sensor_msgs::JointState_<std::allocator<void> > JointState;

using namespace Conversion;

// Debug Flags for more debugging information:
// Log gomotion move joint command
// #define DebugJointCommand
// Log controller action loop for robot servo/joint moves
// #define RobotCNCJointMove
// Log controller action loop for robot servo of world cartesian move
// #define DebugWorldCommand
// Log scene information
#define LogScene

//  Debug IK Fast information
// #define IKFASTDEBUG


namespace RCS {

    inline double ToDegree(double ang) {
        return ang * 180.0 / M_PI;
    }

    inline double Nop(double d) {
        return d;
    }

    template<typename OP> //double (*F)(double ) = Nil>
    inline std::string FcnVectorDump(std::vector<double> v, OP op=Nop) {
        std::stringstream s;

        for (size_t i = 0; i < v.size(); i++) {
            s << op(v[i]) << ":";
        }
        return s.str();
    }

    template<typename T>
    inline std::string VectorDump(std::vector<T> v) {
        std::stringstream s;

        for (size_t i = 0; i < v.size(); i++) {
            s << v[i] << ":";
        }
        return s.str();
    }

    template<>
    inline std::string VectorDump(std::vector<double> v) {
        std::stringstream s;

        for (size_t i = 0; i < v.size(); i++) {
            s << boost::format("%5.2f") % v[i] << ":";
        }
        return s.str();
    }

inline void getRPY(tf::Pose pose, double &roll, double &pitch, double &yaw){
	Eigen::Affine3d epose = Conversion::Convert<tf::Pose, Eigen::Affine3d> (pose);
       tf::Matrix3x3 tfMatrix3x3;
        Eigen::Matrix3d e = epose.rotation();
        tfMatrix3x3 = Convert<Eigen::Matrix3d, tf::Matrix3x3>(e);

        tfMatrix3x3.getRPY(roll, pitch, yaw);
        }

    /*!
     * \brief DumpPose takes a urdf pose  and generates a string describing pose. 
     * Can be used as std::cout << DumpPose(pose); 

     */
    inline std::string DumpPose(tf::Pose & pose) {
        std::stringstream s;

        s << "T   = " << 1000.0 * pose.getOrigin().x() << ":" << 1000.0 * pose.getOrigin().y() << ":" << 1000.0 * pose.getOrigin().z() << std::endl;
        double roll, pitch, yaw;
        getRPY(pose, roll, pitch, yaw);
        s << "RPY = " << Rad2Deg(roll) << ":" << Rad2Deg(pitch) << ":" << Rad2Deg(yaw) << "\n";
        s << "Q   = " << pose.getRotation().x() << ":" << pose.getRotation().y() << ":" << pose.getRotation().z() << ":" << pose.getRotation().w();
        //s << Crcl::DumpRotationAsCrcl(pose)<< std::endl;
        return s.str();
    }

    inline std::string DumpEigenPose(Eigen::Affine3d pose) {
        std::stringstream s;
        s << "Translation = " << 1000.0 * pose(0, 3) << ":" << 1000.0 * pose(1, 3) << ":" << 1000.0 * pose(2, 3);
        tf::Matrix3x3 tfMatrix3x3;
        Eigen::Matrix3d e = pose.rotation();
        tfMatrix3x3 = Convert<Eigen::Matrix3d, tf::Matrix3x3>(e);

        double roll, pitch, yaw;
        tfMatrix3x3.getRPY(roll, pitch, yaw);
        s << "Rotation = " << Rad2Deg(roll) << ":" << Rad2Deg(pitch) << ":" << Rad2Deg(yaw);

        return s.str();
    }
    //inline std::string DumpPoseSimple(RCS::Pose & pose) {

    inline std::string DumpPoseSimple(tf::Pose pose) {
        std::stringstream s;

        s << boost::format("%7.2f") % (1000.0 * pose.getOrigin().x()) << ":" <<
                boost::format("%7.2f") % (1000.0 * pose.getOrigin().y()) << ":" <<
                boost::format("%7.2f") % (1000.0 * pose.getOrigin().z()) << "|";
        double roll, pitch, yaw;
        //getRPY(pose, roll, pitch, yaw);
        tf::Matrix3x3(pose.getRotation()).getRPY(roll, pitch, yaw);
        s << boost::format("%5.2f") % Rad2Deg(roll) << ":" <<
                boost::format("%5.2f") % Rad2Deg(pitch) << ":" <<
                boost::format("%5.2f") % Rad2Deg(yaw);
        return s.str();
    }

    /*!
     * \brief DumpEPosition generates a debug string for an Eigen Vector. 
     * Can be used as std::cout << DumpEPosition(v); 
     */

//    inline std::string DumpEVector(const Eigen::Vector3d & v) {
//        std::stringstream s;
//        s << boost::format("%8.5f") % v(0) << boost::format("%8.5f") % v(1) << boost::format("%8.5f") % v(2) << "\n";
//        return s.str();
//    }

    template<typename T>
    inline std::string DumpEVector(const T & v) {
        std::stringstream s;
        for (int i = 0; i < v.size(); i++)
            s << boost::format("%8.5f:") % v(i);
        return s.str();
    }
    /*!
     * \brief DumpJoints takes a list of joints and generates a string describing pose. 
     * Can be used as std::cout << DumpPose(pose); 
     */

    inline std::string DumpJoints(JointState joints) {
        std::stringstream s;
        s << VectorDump<double> (joints.position);
        return s.str();
    }

    /*!
     * \brief DumpPose takes a urdf pose  and generates a string describing pose. 
     * Can be used as std::cout << DumpPose(pose); 
     */
    inline std::ostream & operator<<(std::ostream & os, tf::Pose & pose) {
        std::stringstream s;
        s << "Translation = " << 1000.0 * pose.getOrigin().x() << ":" << 1000.0 * pose.getOrigin().y() << ":" << 1000.0 * pose.getOrigin().z() << std::endl;
        double roll, pitch, yaw;
        getRPY(pose, roll, pitch, yaw);
        s << "Rotation = " << Rad2Deg(roll) << ":" << Rad2Deg(pitch) << ":" << Rad2Deg(yaw) << std::endl;
        s << "Quaterion = " << pose.getRotation().x() << ":" << pose.getRotation().y() << ":" << pose.getRotation().z() << ":" << pose.getRotation().w();
        os << s.str();
    }

    /*!
     * \brief DumpQuaterion takes a urdf quaterion  and generates a string describing x,y,z,w coordinates. 
     * Can be used as std::cout << DumpQuaterion(urdf::rotation); 
     */
    inline std::string DumpQuaterion(std::ostream & os, const tf::Quaternion & rot) {
        std::stringstream s;
        s << "Quaterion = ";
        s << boost::format("X=%8.4f") % rot.x() << ":";
        s << boost::format("Y=%8.4f") % rot.y() << ":";
        s << boost::format("Z=%8.4f") % rot.z() << ":";
        s << boost::format("W=%8.4f") % rot.w() << ":";
        //       s << std::endl;
        return s.str();
    }
    inline std::string Dump4x4Matrix(Eigen::Matrix4d m) {
        std::stringstream s;
        for (size_t i = 0; i < 4; i++) {
            s << "|";
            for (size_t j = 0; j < 4; j++) {
                s << boost::format("%8.4f") % m(i, j) << " ";
            }
            s << "|\n";
        }
        s << "\n";
        return s.str();
    }
};



template<typename T>
inline std::string MapDump(std::map<std::string, T> m) {
    std::stringstream s;

    for (typename std::map<std::string, T>::iterator it = m.begin(); it != m.end(); it++) {
        s << (*it).first << "=" << (*it).second << std::endl;
    }
    s << std::endl;
    return s.str();
}

