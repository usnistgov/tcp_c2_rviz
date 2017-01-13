


#include <string>
#include <vector>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <math.h>       /* isnan, sqrt */
#include <stdarg.h>
#include <stdlib.h> /*setenv()*/

#include <boost/format.hpp>
#include <boost/property_tree/ini_parser.hpp>
namespace pt = boost::property_tree;
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include "NIST/Config.h"
#include "NIST/RCSThreadTemplate.h"
#include "NIST/RCSMsgQueue.h"
#include "AsioSocketCmdLine.h"
#include "JointStateUpdater.h"

#include "Debug.h"
#include "RvizMarker.h"
#include "Globals.h"


#define ROSPACKAGENAME "cmdinterpreter"
static std::string myname("cmdinterpreter");
static RCS::CMessageQueue<std::string> incmds;

static bool SetEnvironmentFromMap(std::map<std::string, std::string> envmap) {
    std::map<std::string, std::string>::iterator it;
    for (it = envmap.begin(); it != envmap.end(); it++)
        setenv((*it).first.c_str(), (*it).second.c_str(), true);
    return true;
}

int main(int argc, char** argv) {

    try {

        boost::shared_ptr<CRvizMarker> rvizMarker;
        std::string exedirectory;
        std::string packagepath;

        // Find path of executable
        std::string path(argv[0]);
        exedirectory = path.substr(0, path.find_last_of('/') + 1);

        // Environment variable setup for Netbeans IDE use
        Nist::Config cfg;
        cfg.load(exedirectory + ROSPACKAGENAME + ".ini");
        std::map<std::string, std::string> envmap = cfg.getmap("env");
        SetEnvironmentFromMap(envmap);
        double hz = cfg.GetSymbolValue("options.hz", 50.0).toNumber<double>();
        int port = cfg.GetSymbolValue("options.port", 29000).toNumber<int>();


        // Initialize ROS
        ros::init(argc, argv, ROSPACKAGENAME);
        ros::NodeHandle nh;
        ros::Rate r(hz); // 10 times a second = 10Hz
        //  Required for multithreaded ROS communication  NOT TRUE: if not ros::spinOnce
        ros::AsyncSpinner spinner(2); // thread count = 2?
        spinner.start();

        //   RvizMarker setup
        rvizMarker = boost::shared_ptr<CRvizMarker>(new CRvizMarker(nh));
        rvizMarker->Init();

        packagepath = ros::package::getPath(ROSPACKAGENAME);

        JointStateUpdater updater(nh);
        updater.Init("world", "motoman_link_t");
        std::cout << "NC " << updater.Name().c_str() << "\n";
        std::cout << "num joints " << updater.NumJoints() << "\n";
        std::cout << "Joint names " << RCS::VectorDump<std::string>(updater.JointNames()).c_str() << "\n" << std::flush;

        socket_line_connection cmdline;
        cmdline.Init(port);
        bool bDegrees = false;
        bool bFlag = true;
        do {
            cmdline.io_service.run_one();
            if (incmds.SizeMsgQueue() > 0) {
                std::string msg = incmds.PopFrontMsgQueue();
                std::cout << "Echo=" << msg;
                msg = Trim(msg);
                std::vector<std::string> tokens = Split(msg, ' ');
                switch (tokens.size()) {
                    case 1:
                        if (strcasecmp(tokens[0].c_str(), "quit") == 0)
                            bFlag = false;
                        else if (strcasecmp(tokens[0].c_str(), "degrees") == 0)
                            bDegrees = true;
                        else if (strcasecmp(tokens[0].c_str(), "radians") == 0)
                            bDegrees = false;
                        else if (strcasecmp(tokens[0].c_str(), "degree") == 0)
                            bDegrees = true;
                        else if (strcasecmp(tokens[0].c_str(), "radian") == 0)
                            bDegrees = false;
                        else if (strcasecmp(tokens[0].c_str(), "clear") == 0) {
                            updater.Update(updater.JointNames(), std::vector<double> (updater.NumJoints(), 0.0));
                            rvizMarker->Clear();
                        } else if (strcasecmp(tokens[0].c_str(), "home") == 0) {
                            updater.Update(updater.JointNames(), std::vector<double> (updater.NumJoints(), 0.0));
                        } else if (strcasecmp(tokens[0].c_str(), "rand") == 0) {
                            std::vector<double> positions = GenRandomVector(updater.NumJoints(), -M_PI, M_PI);
                            updater.Update(updater.JointNames(), positions);
                        }
                        break;
                    case 2:
                        if (strcasecmp(tokens[0].c_str(), "mark") == 0) {
                            std::vector<std::string> dbls = Split(tokens[1], ',');
                            std::vector<double> pos = Convert<std::vector<std::string>, std::vector<double>>(dbls);
                            tf::Pose pose = Conversion::CreateRPYPose(pos);
                            rvizMarker->Send(pose);
                        } else if (strcasecmp(tokens[0].c_str(), "color") == 0) {
                            std::vector<std::string> dbls = Split(tokens[1], ',');
                            std::vector<double> colors = Convert<std::vector<std::string>, std::vector<double>>(dbls);
                            if (dbls.size() < 3)
                                break;
                            rvizMarker->SetColor(colors[0], colors[1], colors[2], 1.0);
                        } else if (strcasecmp(tokens[0].c_str(), "rgb") == 0) {
                            std::vector<std::string> dbls = Split(tokens[1], ',');
                            std::vector<double> colors = Convert<std::vector<std::string>, std::vector<double>>(dbls);
                            colors = ScaleVector<double>(colors, 1.0 / 255.0); // 
                            if (dbls.size() < 3)
                                break;
                            rvizMarker->SetColor(colors[0], colors[1], colors[2], 1.0);
                        }// Set rviz maker to bullet, arrow, 
                        else if (strcasecmp(tokens[0].c_str(), "marker") == 0) {
                            rvizMarker->SetShape(tokens[1]);
                        } else if (strcasecmp(tokens[0].c_str(), "joints") == 0 ||
                                strcasecmp(tokens[0].c_str(), "j") == 0) {
                            std::vector<std::string> dbls = Split(tokens[1], ',');
                            std::vector<double> positions = Convert<std::vector<std::string>, std::vector<double>>(dbls);
                            if (bDegrees)
                                positions = ScaleVector<double>(positions, M_PI / 180.0); // 
                            updater.Update(updater.JointNames(), positions);
                        } 
                        break;
                    case 3:
                        if (strcasecmp(tokens[0].c_str(), "move") == 0) {
                            // move 0,1  90.0,180.0
                            std::vector<std::string> vals = Split(tokens[2], ',');
                            std::vector<std::string> indexes = Split(tokens[1], ',');
                            std::vector<uint64_t> jointindexes = ConvertStringVector<uint64_t>(indexes);
                            std::vector<double> jointvals = ConvertStringVector<double>(vals);
                            if (bDegrees)
                                jointvals = ScaleVector<double>(jointvals, M_PI / 180.0); // 
                            JointState newjoints = updater.MakeJointState(jointindexes, jointvals);
                            updater.Update(newjoints.name, newjoints.position);
                        }
                        break;
                    default:
                        std::cout << "Bad command input\n";
                        break;
                }

            }
            r.sleep();
        } while (ros::ok() && bFlag);
        spinner.stop();
        std::cout << "Cntrl C pressed \n" << std::flush;

        // ^C pressed - stop all threads or will hang
        RCS::Thread::StopAll(); // includes thread for Controller, robotstatus

    } catch (std::exception e) {
        std::cout << StrFormat("%s%s", "Abnormal exception end to  cmdinterpreter", e.what());
    } catch (...) {
        std::cout << "Abnormal exception end to " << myname;
    }
    ros::shutdown();
    return 0;

}
