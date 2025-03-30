# ME5413_Final_Project

##remember to source:
cd ~/ME5413_Final_Project

source devel/setup.bash


### 1. Gazebo World


roslaunch me5413_world world.launch

*************************************************************************
### 2. Navigation



#teb planner:
#sudo apt-get install ros-noetic-teb-local-planner
roslaunch me5413_world navigation_teb.launch

#dwa planner:
roslaunch me5413_world navigation_dwa.launch

***********************************************************************

### 3. Set navigation goal (including reaching goal in state1 and explore in state2)
## here exploration (counting boxes) is by mannually setting navigatoin waypoints to cover the whole area

rosrun me5413_world nav_client.py


***********************************************************************
### 4. perception

###modification : added depth camera in jackal urdf file


#terminal 1:

rosrun me5413_world detector.py

rostopic pub /rviz_panel/goal_name std_msgs/String "data: 'box'" #sending msg to start counting box

#terminal 2:

rosrun me5413_world visualizer.py  #visualize the image

#terminal 3:

rosrun me5413_world roi_coordinate_calculator.py # obtain the coordinates when finding the target





