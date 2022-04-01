# ros2_sick
From package '[ros2_sick](https://github.com/iscumd/isc_sick)'
# File
`/src/ros2_sick.cpp`

## Summary 
 This is a Ros2 Driver for the Sick LMS-111 2D LiDAR. With the current iteration it supports IPv4 at a transmission rate of 10/100MBit
Currently only supports POSIX operating systems.

## Topics

### Publishes
- `/scan`: laserscans from the LiDAR
- `/points`: pointcloud representation of the laserscan returned from the LiDAR

## Params
- `host`: IP of the LiDAR
- `frame_id`: Frame the LiDAR publishes to.
- `port`: Port the LiDAR is listening on.
- `tf_correction`: Corrects scan angle for the tf frame

## Potential Improvements
Lots and Lots of refactoring, allowing serial communication with the LiDAR 

# Launch
## File 
 ./launch/ros2_sick.launch.py 
 
  
Launches the node with parameters set by a config yaml.
 

