# MultiLidarCameraSync
An attempt to synchronize multi-Lidar and camera system. No gps, no CANbus, no IMU.
Right now it can only do offline data processing. 
The output is used to do 3D detection with lidar and camera.
Odometry is needed for the pointcloud distortion correction. There are lots of options from Optical flow, LOAM, visual odometry etc. For me, I used LOAM because my data is recorded in city, with lots of geometry features.

Finished: A naive version of synchronization with ApproximateTime Policy in messagefilter.  

To Do: Camera-centered multi-lidar synchronization
