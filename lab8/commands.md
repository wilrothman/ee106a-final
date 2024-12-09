COMMAND STREAM:

source ~ee106a/sawyer_setup.bash

roslaunch realsense2_camera rs_camera.launch mode:=Manual color_width:=424 \
color_height:=240 depth_width:=424 depth_height:=240 align_depth:=true \
depth_fps:=6 color_fps:=6

rviz (to check the camera is up and publishing)

rosrun tf static_transform_publisher 0 0 0 0 0 0 base camera_link 100

source devel/setup.bash

rosrun perception pole_detector.py
rosrun perception light_detector.py






INFO:

base frame of sawyer: "base"
what is "camera_link"? the RealSense does not have a reference frame, so by running 'rosrun tf static_transform...' we create a frame that will correspond to the camera which is called camera_link and is transformed by the detailed transformation from the base frame of the sawyer
what is the pylance file? visual stidio python files; not to worry about





