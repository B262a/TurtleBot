# thermalfollower
This repository contains the source code for the Thermal follower program of group B262a.
This code is used in the group's P1 project as well as for the Robot Programming mini-project

Below are instructions on how to run the program:
*The computer must have ROS(at least indigo) and OpenCV installed before continuing

1. Navigate to a ROS worspace primary src folder, by example:
  $ cd ~/catkin_ws/src

2. Clone this git repository
  $ git clone https://github.com/B262a/thermalfollower.git
  
  A new directory named "thermalfollower" will be created, containing all the necessary files

3. Navigate to the root of the ROS workspace
  $ cd ~/catkin_ws

4. Build the package using catkin_make
  $ catkin_make
  
5. Run the program
  $ rosrun thermalfollower follower
