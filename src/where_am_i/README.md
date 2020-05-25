# Where Am I? - world package

To run, execute

```bash
roslaunch where_am_i world.launch
```

This will spawn the Gazebo world including a white ball to chase,
as well as the differential drive robot equipped with Camera and
an unused LiDAR. RViz will be started alongside Gazebo.

See [`urdf/my_robot.xacro`](urdf/my_robot.xacro) for the robot definition.
