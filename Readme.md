
#Readme for Textbased Command and Control using Socket of a ROS RVIZ Visualization of a Motoman 
----

Michaloski, John L. (Fed)
1/26/2017 1:52:06 PM
MotomanRvizReadme.docx

This document presents a Command and Control (C&C) text based interface for communicating with Robot Operating System (ROS) package that interfaces to a ROS RVIZ visualization of a robot (currently only Motoma SI20D, Fanuc LRMate 200id, and Kuka LWR). This implementation offers robot controllers that are not ROS-based the ability to simulate robot motion that is displayed in RVIZ. It uses the Unified Robot Description Format (URDF) to provide the ROS parameter "robot_description" used by RVIZ to draw and control the robot.
The tested version information for the ROS implementation is:
 - ROS indigo 
 - OS - Ubuntu 12.04 (64-bit)
 - Boost ASIO
 - Package versions at the end of the document. Note many of the packages don't appear to be used but are included as a dependency by other packages.
The text based C&C interface appear later in the document. 
#Software Architecture
The ROS software is as minimalistic as possible. You will need ROS, RVIZ, joint_state_publisher, and robot_transform packages installed, but these are part of the main distribution or ROS so this should not be an issue.  RVIZ reads the "robot_description"  ROS parameter to determine how and where to display a robot. Using this "robot_description" ROS parameter the cmdinterpreter package understands the joint configuration of the robot. Note, only serial kinematic chains are supported (i.e., industrial robots not androids with a head and two arms.) The C&C interface maps joint numbers (0..n-1) into the corresponding joint names based on kinematic name position.  The command and control interface to the RVIZ robot is only through joints. (However, the ability to send RVIZ markers which requires poses (position and orientation) is available, but is not documented at this time.) 
The cmdinterpreter package advertises joint value updates to the /nist_controller/robot/joint_states topic which is read by the joint_state_publisher package.

<CENTER>
![Figure1](./images/image1.gif?raw=true)
</CENTER>

The cmdinterpreter package advertises updates to the /nist_controller/robot/joint_states topic which is read by the joint_state_publisher package. This communication is enabled in the launch file by the following snippet:

	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
	    <param name="/use_gui" value="true"/>
	    <rosparam param="/source_list">[nist_controller/robot/joint_states]</rosparam>
	</node>

In roslaunch, a "robot description" is loaded into the ROS system (default is  the motoman sia20d) is loaded and used by RVIZ. 

	<arg name="robotpkg" default="motoman_sia20d"/>
	<param name="robotpkg" value="$(arg robotpkg)"/>
	<param name="robot_description" textfile="src/$(arg robotpkg)_support/urdf/robot.urdf" />
The robot pkg argument can be specified on the roslaunch command line, using the "robot" ROS arg.

	roslaunch cmdinterpreter rviz.launch robot:= kuka_lwr

This informs the launch file that the "robotpkg" arg is now "kuka_lwr" and not  "motoman_sia20d". 
Note, you **must** be in the root directory of the workspace. The reason you must be in the root /workspace (and not /src) is that the ros parameter "robot_description" uses a semi-hard-coded path to the robot urdf file. This was because ROS does not allow embedded execution of arg inside of find, i.e., the launch file does not support: $(find $(arg robotpkg)/urdf/robot.urdf) . Plus, ROS does also not allow programs to access the launch arguments (or if possible, it wasn't obvious.)  
#Installation
The Motoman Command and Control Visualization assumes ROS indigo has been installed on your platform. It should work under Jade or Kinetic versions of ROS but has not been tested. There is not much source, and you will have to compile, so 
##Download & Build from Source
First, build your workspace:

	> mkdir -p tcp_c2_rviz/src 
	> cd tcp_c2_rviz/src
Then clone the motoman visualizaton code from the github repository.

	# clones motoman rvi packages into the src dirctory
	> git clone https://github.com/usnistgov/tcp_c2_rviz.git
	# commands are initiated from the home ws directory
	> cd ..
	# build where catkin is a set of CMake macros that simplify build and maintenance
	> catkin build -DCMAKE_BUILD_TYPE=Debug 
	# update path environment variable for ROS
	> source devel/setup.sh

In conjunction with the ability to use different robots, each robot has different base link and end effector link  and possibly world coordinate names.  This discrepency must be resolved. So the cmdinterpreter.ini file in the FIXME: workspace/devel/cmdinterpreter/lib/cmdinterpreter.ini file now includes sections for each robot handled with the key definition for the base and eelink names. The  ini file has:

	[motoman_sia20d]
	base= base_link
	eelink=link_t

and for the fanuc lrmate 200 id the following ini section was added:

	[fanuc_lrmate200id]
	base= base_link
	eelink=link_6

In RVIZ you will need to go into global options and change the world coordinates for the display to work properly. So far, it is not obvious how to change the value programmatically, and not manually.
#Testing
After compiling you will have 1 interface packages: cmdinterpreter and a bunch of robot URDF configurations, so far: motoman_sia20d_support, fanuc_lrmate200id_support, and kuka_lwr_support. 
 - cmdinterpreter contains the ROS interface to the joint publisher, which reads commands on a socket and publishes the joints to a ROS topic and has the launch files.
 - motoman_sia20d_support, fanuc_lrmate200id_support, and kuka_lwr_support contains the urdf/xacro and robot description 
To run the motoman RVIZ command and control interfface, YOU MUST BE IN THE ROOT WORKSPACE DIRECTORY, and then run the following roslaunch sequence:

	> cd tcp_c2_rviz 
	> sourc devel/setup.bash
	> roslaunch cmdinterpreter rviz.launch robot:= motoman_sia20d | fanuc_lrmate200id | kuka_lwr
If you omit the robot roslaunch command line argument, the default value is "motoman_sia20d". You should see the following two screens: (1) the rviz motoman visualization; and (2) the joint_state_publisher.

<CENTER>
![Figure2](./images/image2.gif?raw=true)
</CENTER>

<p align="center">
**Figure 1 Motoman RVIZ Visualization**
</p>

<CENTER>
![Figure3](./images/image3.gif?raw=true)
</CENTER>

<p align="center">
**Figure 2 joint_state_publisher ROS GUI**
</p>
The reason you must be in the root /workspace (and not /src) is that the ros parameter robot_description uses a semi-hard-coded path to the robot urdf file. This was because ROS does not allow embedded execution of arg inside of find, i.e., the launch file does not support: $(find $(arg robotpkg)/urdf/robot.urdf) .  So instead  the following roslaunch code was used:

	<arg name="robotpkg" default="motoman_sia20d"/>
	<param name="robot_description" textfile="src/$(arg robotpkg) _support /urdf/robot.urdf" />
Thus, we can parameterize the robot on the roslaunch command line with the robot arg:

	> roslaunch cmdinterpreter rviz.launch robot:= motoman_sia20d

Clearly not perfect but satisfactory. 
Now you can test the RVIZ motoman command and control. Open a separate terminal and change to the tcp_c2_rviz directory and under it the python directory in the cmdinterpreter src package. 

	cd xxxx/tcp_c2_rviz/src/cmdinterpreter/python

Now run the python application that will accepts text based command and control joint values for the motoman:

	> python socketcmdline.py 

You should see the following sequence of socket information be displayed, as the python program is connecting vial socket 31000 to the cmdinterpreter ROS package (where ">" is the command line prompt):

	Socket Created
	Socket Connected
	>

You can now run some test commands that should be displayed in the RVIZ motoman display. Below are some sample commands with comments following the "#" pound sign:

	> rand			# randonly move joints
	> home	 		# home joints, i.e., move joints to all zero positions
	> degrees           # use degrees as input values, not radians
	> j 45,45,45,45,45,45,45 # move ALL joints to all given positions
	                    # joints and j are identical commands
	> sleep 10.0        # sleep 10.0 seconds (double to specify seconds) 
	> move 0 90         # move n,m p,q : move joints n,m to  positions p,q
	> move 0 -90        # ditto

There is not much error checking. So buyer beware. There is also the ability to read and process a test file of RVIZ motoman commands.

	> file motoman.txt
	subtokens ['sleep', '5'] 
	subtokens ['degrees'] 
	subtokens ['home'] 
	subtokens ['rand'] 
	subtokens ['sleep', '5'] 
	subtokens ['move', '0', '90.0'] 
	subtokens ['sleep', '3'] 
	subtokens ['move', '0', '-90.0'] 
	subtokens ['sleep', '3'] 
	subtokens ['j', '45,45,45,45,45,45,45'] 
	subtokens ['sleep', '3'] 
	subtokens ['joints', '0,0,0,0,0,0,0'] 
	subtokens ['sleep', '3'] 
	subtokens ['move', '0,1', '90,90'] 
	subtokens ['sleep', '4.0'] 
	subtokens ['move', '0,1', '-90,-90'] 
	>
If you want to quit the Python command line interface program hit ^C and the program will exit. If you want to exit both the Python command line interface program and the RVIZ motoman visualization, enter the command  "quit".

Command and Control Language
Most of the command and control is for the ROS package intervace. Some commands instruct the Python test interpreter and the Description field will denote this use. 

<TABLE>
<TR>
<TD>Command<BR></TD>
<TD>Parameters<BR></TD>
<TD>Description<BR></TD>
</TR>
<TR>
<TD>rand<BR></TD>
<TD><BR></TD>
<TD>Moves to randomly generated joint values<BR></TD>
</TR>
<TR>
<TD>home<BR></TD>
<TD><BR></TD>
<TD>home joints, i.e., move joints to all zero positions<BR></TD>
</TR>
<TR>
<TD>degrees<BR></TD>
<TD><BR></TD>
<TD>Use degrees for joint angle values<BR></TD>
</TR>
<TR>
<TD>radians<BR></TD>
<TD><BR></TD>
<TD>Use radians for joint angle values<BR></TD>
</TR>
<TR>
<TD>joints<BR></TD>
<TD>Val0,..., valn<BR></TD>
<TD>move ALL joints to all given positions, ALL joint values must be specified, from 0 to number of robot joints.<BR></TD>
</TR>
<TR>
<TD>j<BR></TD>
<TD>Synomym for joints<BR></TD>
<TD><BR></TD>
</TR>
<TR>
<TD>move<BR></TD>
<TD>{i,..., j} vali,..., valj<BR></TD>
<TD>move joints i..j to joint values vali, …, valj <BR></TD>
</TR>
<TR>
<TD>file<BR></TD>
<TD>filename<BR></TD>
<TD>Python interpreter will read the file and then send each command to the ROS C&C cmdinterpreter.<BR></TD>
</TR>
<TR>
<TD>sleep<BR></TD>
<TD>seconds<BR></TD>
<TD>Python sleeps for seconds in double<BR></TD>
</TR>
<TR>
<TD>goto<BR></TD>
<TD>J1,Jn<BR></TD>
<TD>Move from current joint position to goal joint position. Use linear interpolation to move each joint. You must specify values for all joints.<BR></TD>
</TR>
</TABLE>



#Version Dependencies
<TABLE>
<TR>
<TD>Package<BR></TD>
<TD>Version<BR></TD>
</TR>
<TR>
<TD>actionlib<BR></TD>
<TD>1.11.6<BR></TD>
</TR>
<TR>
<TD>actionlib_msgs<BR></TD>
<TD>1.11.9<BR></TD>
</TR>
<TR>
<TD>catkin<BR></TD>
<TD>0.6.18<BR></TD>
</TR>
<TR>
<TD>class_loader<BR></TD>
<TD>0.3.4<BR></TD>
</TR>
<TR>
<TD>cpp_common<BR></TD>
<TD>0.5.7<BR></TD>
</TR>
<TR>
<TD>eigen_conversions<BR></TD>
<TD>1.11.8<BR></TD>
</TR>
<TR>
<TD>eigen_stl_containers<BR></TD>
<TD>0.1.4<BR></TD>
</TR>
<TR>
<TD>fcl<BR></TD>
<TD>0.3.3<BR></TD>
</TR>
<TR>
<TD>gencpp<BR></TD>
<TD>0.5.5<BR></TD>
</TR>
<TR>
<TD>genlisp<BR></TD>
<TD>0.4.15<BR></TD>
</TR>
<TR>
<TD>genmsg<BR></TD>
<TD>0.5.7<BR></TD>
</TR>
<TR>
<TD>genpy<BR></TD>
<TD>0.5.10<BR></TD>
</TR>
<TR>
<TD>geometric_shapes<BR></TD>
<TD>0.4.4<BR></TD>
</TR>
<TR>
<TD>geometry_msgs<BR></TD>
<TD>1.11.9<BR></TD>
</TR>
<TR>
<TD>graph_msgs<BR></TD>
<TD>0.1.0<BR></TD>
</TR>
<TR>
<TD>kdl_conversions<BR></TD>
<TD>1.11.8<BR></TD>
</TR>
<TR>
<TD>kdl_parser<BR></TD>
<TD>1.11.11<BR></TD>
</TR>
<TR>
<TD>libccd<BR></TD>
<TD>1.5.0<BR></TD>
</TR>
<TR>
<TD>message_filters<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>message_generation<BR></TD>
<TD>0.2.10<BR></TD>
</TR>
<TR>
<TD>message_runtime<BR></TD>
<TD>0.4.12<BR></TD>
</TR>
<TR>
<TD>moveit_core<BR></TD>
<TD>0.7.2<BR></TD>
</TR>
<TR>
<TD>moveit_msgs<BR></TD>
<TD>0.7.4<BR></TD>
</TR>
<TR>
<TD>object_recognition_msgs<BR></TD>
<TD>0.4.1<BR></TD>
</TR>
<TR>
<TD>octomap<BR></TD>
<TD>1.6.9<BR></TD>
</TR>
<TR>
<TD>octomap_msgs<BR></TD>
<TD>0.3.3<BR></TD>
</TR>
<TR>
<TD>orocos_kdl<BR></TD>
<TD>1.3.1<BR></TD>
</TR>
<TR>
<TD>pluginlib<BR></TD>
<TD>1.10.3<BR></TD>
</TR>
<TR>
<TD>python_orocos_kdl<BR></TD>
<TD>1.3.1<BR></TD>
</TR>
<TR>
<TD>random_numbers<BR></TD>
<TD>0.3.0<BR></TD>
</TR>
<TR>
<TD>resource_retriever<BR></TD>
<TD>1.11.6<BR></TD>
</TR>
<TR>
<TD>rosbag<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rosbag_migration_rule<BR></TD>
<TD>1.0.0<BR></TD>
</TR>
<TR>
<TD>rosbag_storage<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rosbuild<BR></TD>
<TD>1.11.13<BR></TD>
</TR>
<TR>
<TD>rosclean<BR></TD>
<TD>1.11.13<BR></TD>
</TR>
<TR>
<TD>rosconsole<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rosconsole_bridge<BR></TD>
<TD>0.4.4<BR></TD>
</TR>
<TR>
<TD>roscpp<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>roscpp_serialization<BR></TD>
<TD>0.5.7<BR></TD>
</TR>
<TR>
<TD>roscpp_traits<BR></TD>
<TD>0.5.7<BR></TD>
</TR>
<TR>
<TD>rosgraph<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rosgraph_msgs<BR></TD>
<TD>1.11.2<BR></TD>
</TR>
<TR>
<TD>roslaunch<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>roslib<BR></TD>
<TD>1.11.13<BR></TD>
</TR>
<TR>
<TD>roslz4<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rosmaster<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rosmsg<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rosnode<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rosout<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rospack<BR></TD>
<TD>2.2.7<BR></TD>
</TR>
<TR>
<TD>rosparam<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rospy<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rosservice<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rostest<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rostime<BR></TD>
<TD>0.5.7<BR></TD>
</TR>
<TR>
<TD>rostopic<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rosunit<BR></TD>
<TD>1.11.13<BR></TD>
</TR>
<TR>
<TD>roswtf<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>rviz_visual_tools<BR></TD>
<TD>2.2.1<BR></TD>
</TR>
<TR>
<TD>sensor_msgs<BR></TD>
<TD>1.11.9<BR></TD>
</TR>
<TR>
<TD>shape_msgs<BR></TD>
<TD>1.11.9<BR></TD>
</TR>
<TR>
<TD>srdfdom<BR></TD>
<TD>0.3.1<BR></TD>
</TR>
<TR>
<TD>std_msgs<BR></TD>
<TD>0.5.10<BR></TD>
</TR>
<TR>
<TD>tf<BR></TD>
<TD>1.11.8<BR></TD>
</TR>
<TR>
<TD>tf2<BR></TD>
<TD>0.5.13<BR></TD>
</TR>
<TR>
<TD>tf2_msgs<BR></TD>
<TD>0.5.13<BR></TD>
</TR>
<TR>
<TD>tf2_py<BR></TD>
<TD>0.5.13<BR></TD>
</TR>
<TR>
<TD>tf2_ros<BR></TD>
<TD>0.5.13<BR></TD>
</TR>
<TR>
<TD>tf_conversions<BR></TD>
<TD>1.11.8<BR></TD>
</TR>
<TR>
<TD>topic_tools<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
<TR>
<TD>trajectory_msgs<BR></TD>
<TD>1.11.9<BR></TD>
</TR>
<TR>
<TD>urdf<BR></TD>
<TD>1.11.11<BR></TD>
</TR>
<TR>
<TD>urdf_parser_plugin<BR></TD>
<TD>1.11.11<BR></TD>
</TR>
<TR>
<TD>urdfdom_py<BR></TD>
<TD>0.3.1<BR></TD>
</TR>
<TR>
<TD>visualization_msgs<BR></TD>
<TD>1.11.9<BR></TD>
</TR>
<TR>
<TD>xmlrpcpp<BR></TD>
<TD>1.11.20<BR></TD>
</TR>
</TABLE>


![Word2Markdown](./images/word2markdown.jpg?raw=true)  Autogenerated from Microsoft Word by [Word2Markdown](https://github.com/johnmichaloski/SoftwareGadgets/tree/master/Word2Markdown)