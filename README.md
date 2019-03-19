# object_identification

***object_identification*** is a ROS package for identifing rectangle(red), cylinder(green), sphere(blue) and others(gray) from 3D point cloud.

![Identification Result](images/result.gif)

## Requirements
***object_identification*** requires the followings:
- ROS Kinetic
- PCL

## Example

Publish a point cloud using ex. [iai_kinect2](https://github.com/code-iai/iai_kinect2).   
Subscribed topic name is "/kinect2/hd/points".  
And launch following command.  
```bash
roslaunch object_identification object_identification.launch
```

You can see the result with rviz window.
