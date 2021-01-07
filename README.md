# gym_hectorquad

## Main Installation

### Prerequisites - 
 - Ubuntu 16.04
 - ROS Kinetic Installation
 - Python3 venv / conda env enabled

### Prepare Catkin Workspace - 
 ```
    mkdir -p quad_ws/src
    pip install catkin_pkg rospkg empy pyyaml # Do not install em. Install empy only.
    cd quad_ws
    catkin build --cmake-args -DPYTHON_VERSION=3.6 
```

Finally add `source {path_to_quad_ws}/devel/setup.bash` to your `.bashrc`. Reference - [ROS Create Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)


### Install Packages - 
```
cd quad_ws/src
git clone -b melodic-devel https://github.com/ros/geometry
git clone -b melodic-devel https://github.com/ros/geometry2
git clone -b melodic https://github.com/ros-perception/vision_opencv.git
git clone -b kinetic-devel https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git
git clone -b kinetic-devel https://github.com/ros-simulation/gazebo_ros_pkgs.git


git clone https://github.com/kkelchte/hector_quadrotor.git # We use Hector Quad from the DoShiCo paper instead of the original to get Bebop 2 Drone
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git 


rosdep install --from-paths ~/quad_ws  --ignore-src --rosdistro=kinetic # Pulls Dependencies
sudo apt install ros-kinetic-octomap-ros ros-kinetic-octomap-msgs ros-kinetic-moveit-msgs
cd ..
catkin build --cmake-args -DPYTHON_VERSION=3.6 
```


You might run into C++ errors while building. This is mostly probably because certain package require C++11 support. Fix this by adding `add_compile_options(-std=c++11)
` to the `CMakeLists.txt` of those package/sub-packages. It would usually be these packages - 

```
hector_quadrotor_interface 
hector_quadrotor_pose_estimation 
hector_quadrotor_controllers 
hector_quadrotor_teleop 
hector_quadrotor_actions
```

## Testing
```
roslaunch gym_hectorquad drive.launch

```


## Bebop_Autonomy / ParrotSDK (Optional)

Only needed to interface with Parrot Sphinx simulator or the actual drone.
We need the ROS Packages `bebop_autonomy` which in turn requires `parrot_arsdk`.

Ensure you have a working ROS installation / Catkin workspace setup. Then - 
```
cd ~/quad_ws
git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy
git clone https://github.com/AutonomyLab/parrot_arsdk.git src/parrot_arsdk
catkin_make
```

You will likely run into problems, especially if using Python 3.

### parrot_arsdk
A potential problem is that it is not able to detect the native architecture. To check this run `parrot_arsdk/script/get_arch.py`. It should correctly display `x64` (for 64 bit). 

If it doesn't then edit the script to fix this, by editing the following line -
```
arch = subprocess.check_output(["gcc", "-dumpmachine"]).strip().split("-")[0]
```
to this - 
```
arch = subprocess.check_output(["gcc", "-dumpmachine"]).decode("utf-8").strip().split("-")[0]
```
finally `catkin clean` and then `catkin build` again.

### bebop_autonomy
You may run into an problems when the subpackage `bebop_driver` is being build with errors like 
```
error: ‘array’ is not a member of ‘std’ geometry_msgs
```
This is because `bebop_autonomy` is originally a package for `ros-indigo` and thus the its cmake file needs to be updated to use `C++11`. 

To do this edit `bebop_autonomy/bebop_driver/CMakeLists.txt` and the following line `set(CMAKE_CXX_STANDARD 11)` near the top.

Finally clean and build again.

### teleop_tools
This package is needed for using a joystick. Edit the script `joy_teleop.py` and replace the function call to `iteritems()` with `items()` to make it compatible for python3.

### bebop_tools
Subpackage of `bebop_autonomy`. Needed for using a joystick.

Edit `bebop_autonomy/bebop_tools/launch/joy_teleop.launch` and set the default value of the arg `joy_config` to `xbox360`. 

Note - Joystick Controls given at `bebop_autonomy/bebop_tools/config/xbox360.yaml`