# RoboND Where Am I?

![](.readme/whee.gif)

To run, execute:

```bash
roslaunch where_am_i world.launch
roslaunch where_am_i amcl.launch
```

You can also use keyboard teleop to control the bot manually:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
``` 

Building requires ROS Kinetic; you can try executing `./run_nvidia.sh` to drop into an X11 aware
Docker container with NVIDIA GPU support.

Here's a video of the bot when executing a plan. The overlaid white map
is the local map, and the white arrows show the AMCL particles. As soon as the
bot rotates, localization improves drastically:

![](.readme/automatic.webp)

Here's a video of the bot being controlled via teleop through the legs of the
table (shown in the first GIF of this README):

![](.readme/manual.webp)

One of the biggest issues in control that's still unsolved is the disagreement
of the local planner with the global planner. In the following video,
the local costmap was never populated, resulting in the bot heading straight
from the wall. Eventually, only removing both the `devel` and `build` directories, 
building from scratch and restarting the Docker container helped me there -
but still, the bot goes straight until the local cost map shows a clear obstacle. 

![](.readme/derp.webp)

Note that in the current setup, the applied torque will result in the
bot doing a wheelie when accelerating, as well as a stoppie when braking.
This results in the LiDAR temporarily scanning the ceiling or the floor.
The cost maps are set up to ignore short-term interferences of this kind
and will recover from this behavior immediately, so it's not much of
an issue regarding the project.

## Building with CLion IDE

**Note:** This does not _really_ work, as CLion will be unable to find generated headers. It's still a bit
          better than doing everything the hard way.

The full requirements for setting up CLion are given in the [sunsided/robond-ros-docker](https://github.com/sunsided/robond-ros-docker)
repository. In short, run SSHD in Docker, configure a Remote Host build to connect to it, then configure
the your build settings for ROS. For this repo and the included Dockerfile, this configuration will do:

**CMake options:**

```
-DCATKIN_DEVEL_PREFIX:PATH=/workspace/devel -DCMAKE_PREFIX_PATH=/workspace/devel;/opt/ros/kinetic;/opt/ros/kinetic/share
```

**Environment:**

```
ROS_ROOT=/opt/ros/kinetic/share/ros;ROS_PACKAGE_PATH=/workspace/src:/opt/ros/kinetic/share;ROS_MASTER_URI=http://localhost:11311;ROS_PYTHON_VERSION=2;ROS_VERSION=1;ROSLISP_PACKAGE_DIRECTORIES=/workspace/devel/share/common-lisp;ROS_DISTRO=kinetic;ROS_ETC_DIR=/opt/ros/kinetic/etc/ros;PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages;PKG_CONFIG_PATH=/workspace/devel/lib/pkgconfig:/opt/ros/kinetic/lib/pkgconfig:/opt/ros/kinetic/lib/x86_64-linux-gnu/pkgconfig;LD_LIBRARY_PATH=/workspace/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH;PATH=/opt/ros/kinetic/bin:$PATH
```
