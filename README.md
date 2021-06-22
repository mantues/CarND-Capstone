# System Integration Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
## Capstone Project
### Overview
This is the final project for the Udacity Self-Driving Car Engineer Nanodegree.  In this project, created several ROS nodes to implement core functionality of an autonomous vehicle.  For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/b9040951-b43f-4dd3-8b16-76e7b52f4d9d/modules/85ece059-1351-4599-bb2c-0095d6534c8c/lessons/01cf7801-7665-4dc5-a800-2a9cca06b38b/concepts/addf15cf-d10e-4414-98dd-870d7368a542).


## Setup
### Native Installation
* If you prefer not to use the Workspace, follow the steps below to get set up:
  * Ubuntu 14.04 with ROS Indigo
  * Ubuntu 16.04 with ROS Kinetic
  * You are welcome to use your own Ubuntu installation or virtual machine (unsupported), or you can use the VM provided in Your Virtual Machine in the "Introduction to ROS" lesson. The provided VM has ROS and Dataspeed DBW installed already.
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/install/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage using docker 

##### 0. Enter an interactive shell session on an already-running container
```bash
 docker exec -it <container_name> /bin/bash
```

##### 1. Clone the project repository
```bash
(official github)
git clone https://github.com/udacity/CarND-Capstone.git
or
unzip project.zip
```

##### 2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```

##### 3. Make and run styx
```bash
cd ros
source devel/setup.sh
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

##### 4. Run the simulator

## Project Overview
### ROS Architecture
The ROS Architecture consists of different nodes (written in Python or C++) that communicate with each other via ROS messages.

### Node Design
Those are the Waypoint Updater(waypoint_updater.py), the Traffic Light Detection & Classification (tl_detector.py) and the Drive-By-Wire node (dbw_node.py). 
#### Waypoint Updater(waypoint_updater.py)
 * Subscribe Topics
   * /current_pose
   * /base_waypoints
   * /traffic_waypoint
 * Publish Topics
   * final_waypoints

**Decision making**
The waypoint updater node publish final_waypoints whitch determines waypoints the car should follow. It subscribes car's state (/current_pose and /base_waypoints) and /traffic_waypoint.

#### Traffic Light Detection & Classification (tl_detector.py)
 * Subscribe Topics
   * /current_pose
   * /base_waypoints
   * /vehicle/traffic_lights
   * /image_color (optional)
 * Publish Topics
   * /traffic_waypoint
**Decision making**
It subscribes car's state (/current_pose and /base_waypoints) and traffic light's state(/vehicle/traffic_lights) and camera image(/image_color). This node shows the waypoint(/traffic_waypoint) which the car should stop.
In this case, I tried to use tensorflow to recognize and classify the signal, but it did not work.
So I converted the camera image(/image_color) to HSV and calculated the components to detect the red color and determine the color of the signal. I have confirmed that it works well on the simulator, but I think it is difficult to implement it in real space because it is easily affected by signboards and sunlight.

#### Drive-By-Wire (DBW) Node(dbw_node.py)
 * Subscribe Topics
   * /vehicle/dbw_enabled
   * /twist_cmd
   * /current_velocity
 * Publish Topics
   * /vehicle/steering_cmd
   * /vehicle/throttle_cmd
   * /vehicle/brake_cmd
  
**Decision making**
In case dbw_enabled is set to true, Drive-By-Wire (DBW) Node works. It subscribes /twist_cmd include throttle, brake and steering values with the help of a PID-controller and Lowpass filter. The dbw node directly publishes /vehicle commands for the car/simulator.


