# ros2_sick
From package '[ros2_sick](https://github.com/iscumd/isc_sick)'
# File
`./src/ros2_sick.cpp`

## Summary 
 This is a Ros2 Driver for the Sick LMS-111 2D LiDAR.

Currently only supports POSIX operating systems.

## Topics

### Publishes
- `/sick/scan`: laserscans from the LiDAR

## Params
- `host`: IP of the LiDAR
- `frame_id`: Frame the LiDAR publishes to.
- `port`: Port the LiDAR is listening on.
- `tf_correction`: Corrects scan angle for the tf frame(?)

## Potential Improvements
Lots and Lots of refactoring 

# Launch 
 `./launch/ros2_sick.launch.py` 
  
Just launches the node without args.
 

