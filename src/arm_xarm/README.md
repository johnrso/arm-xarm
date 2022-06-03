# ARM for Real-world XARM

## Install

For the camera:
```bash
sudo apt-get install ros-kinetic-realsense2-camera ros-kinetic-rgbd-launch
```

For the robot:
```bash
sudo apt-get install ros-kinetc-ros-control ros-kinetic-ros-controllers ros-kinetic-moveit ros-kinetic-moveit-visual-tools ros-kinetic-jsk-visualization
```

You can now follow the instruction here: https://github.com/xArm-Developer/xarm_ros#4-getting-started-with-xarm_ros


## Camera Calibration

Connect the realsense along with the arm. Ensure you are connected to the robot network.

To launch the calibration tool, run the following: 

```commandline
roslaunch arm_xarm setup.launch
```

This opens up RViz, where you calibrate the extrinsic parameters (transformation between the camera and robot base)
by aligning the point cloud of the robot to the URDF; [example video here](https://drive.google.com/uc?export=view&id=1n1re5P7M6l-biPBxcnP4qtoVkakAAqkA).

Once the alignment looks good, you get the current transformation via running:

```commandline
rosrun tf tf_echo link_base camera_link
```


## VR

```commandline
roscd arm_xarm
roslaunch arm_xarm setup.launch
```


## Issues

### Error when launching robot node

The first stop should always be to make sure that the robot firmware and xarm_ros repo is up-to-date!

### Installing scikit-robot

When running `pip install scikit-robot[all]` you may encounter an issue with `python-fcl`.
If so you will need to install fcl (from source, because we are using Ubuntu < 17).

```commandline
sudo apt-get install liboctomap-dev 
git clone https://github.com/flexible-collision-library/fcl.git && cd fcl
mkdir build && cd build
cmake ..
make
```

You may encounter errors when installing fcl. I resolved these by swapping to branch `0.7.0`.

This needs multiple dependecies that can cause issues.


### Running vr script

#### Import Error: cannot import name 'gcd' from 'fractions'

For this I added `networkx>=2.5` to requirements.txt for scikit-robot and reinstalled.

### Other

#### AttributeError: 'module' object has no attribute 'Interpreter

If you get something like this when running catkin build

```commandline
pip uninstall em
pip install empy
```



# Alternative Instructions

Still in progress

```
mkdir -p ~/ros_arm_xarm/src
cd ~/ros_arm_xarm/src

git clone https://github.com/stepjam/arm_xarm.git

# Gets dependecies 
ln -s ros_arm_xarm/src/arm_xarm/rosinstall ros_arm_xarm/src/.rosinstall
wstool up

rosdep update
rosdep install --from-path ros_arm_xarm/src/arm_xarm/

```

I hacked 'from functools import reduce' into /opt/ros/kinetic/lib/python2.7/dist-packages/message_filters/__init__.py


catkin config --merge-devel -DPYTHON_EXECUTABLE=/home/stepjam/miniconda/bin/python  -DPYTHON_INCLUDE_DIR=/home/stepjam/miniconda/include/python3.9 -DPYTHON_LIBRARY=/home/stepjam/miniconda/lib/libpython3.9.so  --cmake-args -DCMAKE_BUILD_TYPE=Release -DOCTOMAP_OMP=1