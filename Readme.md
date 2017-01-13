
#Readme for Textbased Command and Control using Socket of  a ROS RVIZ Visualization of a Motoman 
----

Michaloski, John L. (Fed)
11/17/2016 5:13:00 PM
MotomanRvizReadme.docx

This document presents a Robot Operating System (ROS) package for  interfacing to a  ROS RVIZ visualation of a Motoman.
This implementation provides a simulation that is displayed in RVIZ. It use the Unified Robot Description Format (URDF) to provide the ROS parameter "robot_description" used by RVIZ to draw and control the robot.
The visualization of the Motoman robot below is to sort small and medium "gears" into appropriate gear holders is shown in an animated gif below (in this case on "vessels" which hold one type of gear, while a kit may hold varios size gears – or at least in this demo.)  In the code a vision system would generate JSON representation of the parts and instances of the parts. The demo code reads the JSON file using the boost Property Tree package. 
<p align="center">

</p>
<p align="center">
**Figure 1 Animated gif of Motoman sorting gears**
</p>
The version information for the  is:
 - ROS indigo 
 - OS - Ubuntu 12.04 (64-bit)
 - Package versions in Appendix I
#Software Architecture
The ROS software is minimalistic as possible. You will need ROS, RVIZ, joint_state_publisher, and robot_transform packages installed, but these are part of the main distribution so this should not be an issue. 
The cmdinterpreter package advertises updates to the /nist_controller/robot/joint_states topic which is read by the joint_state_publisher package. This communication is enabled in the launch file by the following snippet:

	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
	    <param name="/use_gui" value="true"/>
	    <rosparam param="/source_list">[nist_controller/robot/joint_states]</rosparam>
	</node>

<CENTER>
![Figure1](./images/image1.gif?raw=true)
</CENTER>

In roslaunch, a robot description for the motoman sia20d is loaded and used by RVIZ.

	<param name="robot_description" command="$(find xacro)/xacro.py $(find motoman_sia20d_support)/urdf/ motoman_sia20d.xacro" />
	
	<node name="rviz" pkg="rviz" type="rviz">

#Installation
The Motoman Command and Control Visualization assumes ROS indigo has been installed on your platform. It should work under Jade or Kinetic versions of ROS but has not been tested. There is not much source, and you will have to compile, so 
##Download & Build from Source
First, build your workspace:

	> mkdir -p motoman_ws/src 
	> cd motoman_ws/src
Then clone the motoman visualizaton code from the github repository.

	# clones motoman rvi packages into the src dirctory
	> git clone https://github.com/usnistgov/motoman_rviz.git
	# commands are initiated from the home ws directory
	> cd ..
	# build where catkin is a set of CMake macros that simplify build and maintenance
	> catkin build -DCMAKE_BUILD_TYPE=Debug 
	# update path environment variable for ROS
	> source devel/setup.sh

#Testing
After compiling you will have 3 packages: cmdinterpreter, motoman_sia20d_support, and robotconfig. 
 - motoman_sia20d_support  contains the urdf/xacro and robot description 
 - robotconfig configures the motoman xacro and has launch file
 - cmdinterpreter contains the ROS interface to the joint publisher, which reads commands on a socket and publishes the joints to a ROS topic.
To run the motoman RVIZ command and control interfface, run he following roslaunch 

	> cd motoman_ws 
	> sourc devel/setup.bash
	> roslaunch cmdinterpreter rviz.launch
You should see the following two screens: (1) the rviz motoman visualization; and (2) the joint_state_publisher

<CENTER>
![Figure2](./images/image2.gif?raw=true)
</CENTER>

<p align="center">
**Figure 2 Motoman RVIZ Visualization**
</p>
Now you can test the RVIZ motoman command and control. Open a separate terminal and change to the motoman_ws directory and under it the python directory in the cmdinterpreter src package. 

	cd xxxx/motoman_ws/src/cmdinterpreter/python

Now run the python application that will accepts text based command and control joint values for the motoma:

	> python socketcmdline.py 

You should see the following sequence of socket information be displayed, as the python program is connecting vial socket 31000 to the cmdinterpreter ROS package:

	Socket CreatedSocket Connected
	>

You can now run some test commands that should be displayed in the RVIZ motoman display. Below are some sample commands with comments following the "#" pound sign:

	> rand			# randonly move joints> home	 		# home joints, i.e., move joints to all zero positions> degrees           # use degrees as input values, not radians> j 45,45,45,45,45,45,45 # move ALL joints to all given positions
	                    # joints and j are identical commands> sleep 10.0        # sleep 10.0 seconds (double to specify seconds)> move 0 90         # move n,m p,q : move joints n,m to  positions p,q> move 0 -90        # ditto

There is not much error checking. So buyer beware. There is also the ability to read and process a test file of RVIZ motoman commands.

	> file motoman.txtsubtokens ['sleep', '5']subtokens ['degrees']subtokens ['home']subtokens ['rand']subtokens ['sleep', '5']subtokens ['move', '0', '90.0']subtokens ['sleep', '3']subtokens ['move', '0', '-90.0']subtokens ['sleep', '3']subtokens ['j', '45,45,45,45,45,45,45']subtokens ['sleep', '3']subtokens ['joints', '0,0,0,0,0,0,0']subtokens ['sleep', '3']subtokens ['move', '0,1', '90,90']subtokens ['sleep', '4.0']subtokens ['move', '0,1', '-90,-90']>
If you want to quit the Python command line interface program hit ^C and the program will exit. If you want to exit both the Python command line interface program and the RVIZ motoman visualization, enter the command  "quit".

<CENTER>
![Figure3](./images/image3.gif?raw=true)
</CENTER>

<p align="center">
**Figure 3 joint_state_publisher ROS GUI**
</p>



![Word2Markdown](./images/word2markdown.jpg?raw=true)  Autogenerated from Microsoft Word by [Word2Markdown](https://github.com/johnmichaloski/SoftwareGadgets/tree/master/Word2Markdown)