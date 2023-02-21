# Normal costmap

A ROS package plugin to compute normals from a PointCloud2 and build a costmap2d Layer if the angle of the normals is abose a threshold.

## Compile the package

As usual in ROS, you have to compile the package and add it to the path.
Simply do the following:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Add the plugin in a navigation stack
In the parameters file of the desired costmap (local or global) add the following line in the plugins section:
```yaml
  - {name: normal_layer,                type: "normal_layer_namespace::NormalLayer"}
```
In the package, a .yaml file is already created to set the parameters of the plugin.

## Launch a demo
A launch file has been created to do a demo (with a rosbag for example).
Simply, run the following command:
```bash
roslaunch normal_vect normal_simulation.launch
```
This will create different nodes:
- Static transform publisher from map to base_link
- Static transfrom publisher from base_link to odom
- Rviz to visualize Husky "Paquerette" robot and messages from its sensors
- Move base mapless demo launchfile, in which the plugin had been added previously

In parallel, you can play a rosbag (don't forget too add the parameter ```--clock```), and then visualize the result in Rviz.
