

#pragma once
#include <ros/ros.h>

#include "Debug.h"
#include "RvizMarker.h"
#include "Globals.h"
#include <urdf/model.h>

struct JointStateUpdater {
    bool bRvizPubSetup;
    ros::Publisher rviz_jntcmd; /**< ros publisher information for joint_publisher */
    std::map<std::string, std::vector<double>> NamedJointMove;
    ros::NodeHandle _nh;
    // URDF Derived knowledge
    std::string robotname;
    std::vector<std::string> joint_names;
    std::vector<std::string> link_names;
    std::vector< double> jointvalues;
    std::vector< double> joint_min;
    std::vector< double> joint_max;
    std::vector< bool> joint_has_limits;
    std::vector< double> joint_effort;
    std::vector< double> joint_velmax;

    std::vector<Eigen::Vector3d> axis;
    std::vector<Eigen::Vector3d> xyzorigin;
    std::vector<Eigen::Vector3d> rpyorigin;
    size_t num_joints;
    sensor_msgs::JointState jointcmd;
    JointStateUpdater(ros::NodeHandle &nh) : _nh(nh) {
        bRvizPubSetup=false;
    }

    void Update(std::vector<std::string> names, std::vector<double> values){
        jointcmd.name=names;
        jointcmd.position = values;
        rviz_jntcmd.publish(jointcmd);
    }
        /*!
     * \brief Initialize kinematics using robot_description to fill parameters .
     * \param  nh ros node handle of node.
     */
    virtual void Init( std::string baselinkname,
            std::string tiplinkname) {
        
        if (!bRvizPubSetup) {
            rviz_jntcmd = _nh.advertise<sensor_msgs::JointState>("nist_controller/robot/joint_states", 1);
            bRvizPubSetup = true;
        }
        ROS_DEBUG_NAMED("JointStateUpdater", "init reading xml file from parameter server");
        std::string urdf_xml;
        if (!_nh.getParam("robot_description", urdf_xml)) {
            ROS_FATAL_NAMED("JointStateUpdater", "Could not load the xml from parameter server: %s", urdf_xml.c_str());

        }
        if (!ParseURDF(urdf_xml, baselinkname, tiplinkname))
            ROS_FATAL_NAMED("JointStateUpdater", "Could not parse the xml for kinematic solver", robotname.c_str());
        num_joints = joint_names.size();

    }
    std::string Name() {
        return robotname;
    }
    size_t NumJoints() {
        assert(joint_names.size() != 0);
        return num_joints;
    }

    virtual std::vector<std::string> JointNames() {
        return joint_names;
    }

    //http://docs.ros.org/diamondback/api/urdf/html/classurdf_1_1Model.html 
    bool ParseURDF(std::string xml_string,
            std::string base_frame,
            std::string tiplinkname) {
        urdf::Model robot_model;
        robot_model.initString(xml_string);

        robotname = robot_model.getName();
        
        //ROS_DEBUG_STREAM_NAMED("nc", "Reading joints and links from URDF");

        boost::shared_ptr<urdf::Link> link = boost::const_pointer_cast<urdf::Link>(robot_model.getLink(tiplinkname));
        while (link->name != base_frame) { // && joint_names.size() <= num_joints_) {
            //ROS_DEBUG_NAMED("nc", "Link %s", link->name.c_str());
            link_names.push_back(link->name);
            boost::shared_ptr<urdf::Joint> joint = link->parent_joint;
            if (joint) {
                if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
                    //ROS_DEBUG_STREAM_NAMED("nc", "Adding joint " << joint->name);

                    joint_names.push_back(joint->name);
                    axis.push_back(Convert<urdf::Vector3, Eigen::Vector3d>(joint->axis));
                    xyzorigin.push_back(Convert<urdf::Vector3, Eigen::Vector3d>(joint->parent_to_joint_origin_transform.position));
                    double roll, pitch, yaw;
                    joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
                    rpyorigin.push_back(Eigen::Vector3d(roll, pitch, yaw));

                    float lower, upper, maxvel = 0.0, maxeffort = 0.0;
                    int hasLimits;
                    if (joint->type != urdf::Joint::CONTINUOUS) {
                        maxvel = joint->limits->velocity;
                        maxeffort = joint->limits->effort;
                        if (joint->safety) {
                            lower = joint->safety->soft_lower_limit;
                            upper = joint->safety->soft_upper_limit;
                        } else {
                            lower = joint->limits->lower;
                            upper = joint->limits->upper;
                        }
                        hasLimits = 1;
                    } else {
                        lower = -M_PI;
                        upper = M_PI;
                        hasLimits = 0;
                    }
                    if (hasLimits) {
                        joint_has_limits.push_back(true);
                        joint_min.push_back(lower);
                        joint_max.push_back(upper);
                    } else {
                        joint_has_limits.push_back(false);
                        joint_min.push_back(-M_PI);
                        joint_max.push_back(M_PI);
                    }
                    joint_effort.push_back(maxeffort);
                    joint_velmax.push_back(maxvel);
                }
            } else {
                ROS_WARN_NAMED("nc", "no joint corresponding to %s", link->name.c_str());
            }
            link = link->getParent();
        }

        std::reverse(link_names.begin(), link_names.end());
        std::reverse(joint_names.begin(), joint_names.end());
        std::reverse(joint_min.begin(), joint_min.end());
        std::reverse(joint_max.begin(), joint_max.end());
        std::reverse(joint_has_limits.begin(), joint_has_limits.end());
        std::reverse(axis.begin(), axis.end());
        std::reverse(xyzorigin.begin(), xyzorigin.end());
        std::reverse(rpyorigin.begin(), rpyorigin.end());
        std::reverse(joint_effort.begin(), joint_effort.end());
        std::reverse(joint_velmax.begin(), joint_velmax.end());

        return true;
    }

    /*!
     * \brief Compute distance between seed state and solution joint state.
     * First, normalize all solution joint values to (-2pi,+2pi).
     * \param  ik_seed_state contains original joint value
     * \param  solution is candidate joint values.
     * \return distance between joint vectors
     */
    double Harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const {
        double dist_sqr = 0;
        std::vector<double> ss = ik_seed_state;
        for (size_t i = 0; i < ik_seed_state.size(); ++i) {
#if 0
            while (ss[i] > 2 * M_PI) {
                ss[i] -= 2 * M_PI;
            }
            while (ss[i] < -2 * M_PI) {
                ss[i] += 2 * M_PI;
            }
            while (solution[i] > 2 * M_PI) {
                solution[i] -= 2 * M_PI;
            }
            while (solution[i] < -2 * M_PI) {
                solution[i] += 2 * M_PI;
            }
#endif
            dist_sqr += fabs(ik_seed_state[i] - solution[i]);
        }
        return dist_sqr;
    }

     JointState MakeJointState(std::vector<uint64_t> jointnums,
            std::vector<double> jointval) {
        JointState joints;
        joints.name.resize(jointval.size());
        joints.position.resize(jointval.size(), 0.0);
        joints.velocity.resize(jointval.size(), 0.0);
        joints.effort.resize(jointval.size(), 0.0);
                // Check each joint, to see if joint is being actuated, if so, change goal position
        for (size_t i = 0; i < jointnums.size(); i++) {
            size_t n = jointnums[i]; // should already have indexes -1;
            joints.position[i] = jointval[i]; // joint numbers already adjusted from CRCL to rcs model
            joints.name[i] = joint_names[n]; // joint numbers already adjusted from CRCL to rcs model
        }
        return joints;
    }
};
