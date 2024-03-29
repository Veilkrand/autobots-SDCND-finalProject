This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

| Team Member | Email |
|:---------------------:|:---------------------:|
| Alberto Naranjo	| alberto.galet@gmail.com |
| Abhishek Singhal | singhal.abhi@gmail.com |
| Jose Centeno | centeno.jose.manuel@gmail.com |
| Muhieddine EL Kaissi | emukaissi@hotmail.com |
| Sachin Malusare | sachinmidesign@gmail.com |

### Overview

The goal of this project was to integrate the various systems and modules of a self driving car so as to successfully pilot a car in a simulator and then a real car (CARLA). The car should be able to plot its own course, and identify and respond to traffic lights.

![](imgs/SystemIntegration.png)

#### 1. Waypoint updater
The waypoint updater subscribes to the topics of base waypoints, current_pose, current velocity, traffic_waypoint and computes the final waypoints based on the current status of each of the information elements (topics) described above. 

* Update local variables based on messages received on topics
* Create a final_waypoints message from the base waypoints; These waypoints encode target position and velocity set by the waypoint_loader
* Based on /traffic_waypoint status. Update viewpoints to stop smoothly before the Stop lines of each waypoint
* Publish output

#### 2. DBW Node
The dbw_node is responsible for converting twist cmd to steer brake and throttle commands to be sent to simulator bridge or Carla. 

* Initialize PID controllers to achieve target throttle, brake and steering angles
* Update variables based on messages received on topics
* Calculate error between current and target velocities and orientations
* Map the output to throttle brake and steering values
* If /vehicle/dbw_enabled is true: run a PID iteration to compute new throttle, brake and steering values
* Publish output

#### 3. Traffic Light Detector Node
This node is responsible for receiving images from the camera and detecting and classifying images for traffic light status. 

##### Generate Training Data
* Subscribe to topics and update local variables
* Using received image message and the /vehicle/traffic_lights message values, calculate the pixel co-ordinates of traffic light bounding box
* Record bounding box annotation and image to permanent storage for training the classifier
* Publish traffic light ground truth state based on /vehicle/traffic lights

##### Prediction
* Load frozen model
* Subscribe to topics and update local variables
* Convert image msg to numpy array usable by model
* Predict using loaded model
* Publish traffic light state based on predicted output


### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
