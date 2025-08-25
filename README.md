##Final_Project

##Autonomous Mobile Robotics Final Project  
> Authors: [Anurag Roy](https://github.com/anuragroy2001), [Ariel Hu](https://github.com/loggcc), [Chun Pok Lam](https://github.com/bobbylammy71446307), [Jinhan Xue](https://github.com/tchb0910), [Praveen Krishnapur](https://github.com/Praveen8978), and [Yurong Liu](https://github.com/liuyurong129)

![Ubuntu 20.04](https://img.shields.io/badge/OS-Ubuntu_20.04-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)
![ROS Noetic](https://img.shields.io/badge/Tools-ROS_Noetic-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)
![C++](https://img.shields.io/badge/Code-C++-informational?style=flat&logo=c%2B%2B&logoColor=white&color=2bbc8a)
![Python](https://img.shields.io/badge/Code-Python-informational?style=flat&logo=Python&logoColor=white&color=2bbc8a)

![cover_image](src/me5413_world/media/gz_world.png)

## Click below for video
[![Watch the video](media/thumbnail.png)](https://www.youtube.com/watch?v=4XAWQVB_mV8)

## Dependencies

* System Requirements:
  * Ubuntu 20.04 (18.04 not yet tested)
  * ROS Noetic (Melodic not yet tested)
  * C++11 and above
  * CMake: 3.0.2 and above
* Standard ROS packages:
  * `roscpp`, `rospy`, `rviz`, `std_msgs`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `pluginlib`, `map_server`, `gazebo_ros`, `jsk_rviz_plugins`
  * `jackal_gazebo`, `jackal_navigation`, `velodyne_simulator`, `teleop_twist_keyboard`
* External Models:
  * [Gazebo official models](https://github.com/osrf/gazebo_models)

## Project Structure

### SLAM
- **Cartographer2D**
- **Cartographer3D**
- **Fast-LIO2**

### Planning and Control (PnC)
- **A\***
- **RRT**
- **DWA**
- **TEB**

### Perception
- **EasyOCR**

### Decision and Exploration
- **Finite State Machine (FSM)**

## Installation

This repo is a ROS workspace with three packages:

- `interactive_tools`
- `jackal_description`
- `me5413_world`

```bash
# Clone your own fork of this repo
cd ~
git clone https://github.com/<YOUR_GITHUB_USERNAME>/ME5413_Final_Project.git
cd ME5413_Final_Project

# Install all dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 1. Build SLAM packages

#### Cartographer

Install Abseil:

```bash
~/ME5413_Final_Project/src/third_party/cartographer/cartographer/scripts/install_abseil.sh
```

Build Cartographer:

```bash
cd ~/ME5413_Final_Project
catkin build cartographer*
```

#### Fast-LIO2

Build Fast-LIO:

```bash
cd ~/ME5413_Final_Project
catkin build fast_lio
```

### 2. Build Extra Packages

```bash
catkin build
```

### 3. Prepare Perception (optional)

```bash
# Install Conda first, then:
conda create -n me5413 python=3.8
conda activate me5413

# Install dependencies
conda install pytorch==2.1.1 torchvision==0.16.1 pytorch-cuda=12.1 -c pytorch -c nvidia
conda install -c conda-forge opencv rosdep rospkg easyocr decorator pexpect numpy defusedxml ipdb

# Set PYTHONPATH if needed
export PYTHONPATH=$PYTHONPATH:/usr/lib/python3.8/dist-packages
```

### 4. Source Workspace

```bash
source devel/setup.bash
```

### 5. Prepare Gazebo Models

```bash
# Official models
cd ~
mkdir -p .gazebo/models
git clone https://github.com/osrf/gazebo_models.git
cp -r ~/gazebo_models/* ~/.gazebo/models

# Custom models
cp -r ~/ME5413_Final_Project/src/me5413_world/models/* ~/.gazebo/models
```

## Usage

### 0. Gazebo World

```bash
roslaunch me5413_world world.launch
```

### 1. Manual Control

```bash
roslaunch me5413_world manual.launch
```

---

### 2. Mapping

Choose one of the following launch files depending on the method:

#### **Cartographer 2D**:

```bash
roslaunch me5413_world mapping_carto_2d.launch
```

#### **Cartographer 3D**:

```bash
roslaunch me5413_world mapping_carto_3d.launch
```

- To save the map:

```bash
roscd me5413_world/maps/
rosrun map_server map_saver -f my_map map:=/map
```

#### **Fast-LIO2**:

```bash
roslaunch me5413_world mapping_fast_lio.launch
```

- Point Cloud to Map

```bash
source ~/ME5413_Final_Project/devel/setup.bash
roslaunch pcd2pgm pcd2pgm.launch
rosrun map_server map_saver
```

---

### 3. Navigation

```bash
roslaunch me5413_world navigation_teb.launch
```
If there is an issue with the rviz Control panel for spwanning objects, remove the third part folder, clear build and devel, and catkin_make again. Fix still pending
---

### 4. FSM

```bash
rosrun me5413_world fsm.py
```

---

## Contribution

We welcome contributions via pull requests.  
Follow these style guides:

- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#main)
- [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)

## License

This project is released under the [MIT License](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project/blob/main/LICENSE)
