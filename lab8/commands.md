## Setup steps

**For ALL Windows**
- `source ~ee106a/sawyer_setup.bash`

**Window 1**
- `roslaunch realsense2_camera rs_camera.launch mode:=Manual color_width:=424 color_height:=240 depth_width:=424 depth_height:=240 align_depth:=true depth_fps:=6 color_fps:=6`

**Window 2**
- `rosrun tf static_transform_publisher 0 0 0 0 0 0 base camera_link 100`

**For Window 3,4**
- `cd final/lab8/`
- `catkin_make`
- `source devel/setup.bash`

**Window 3**
- `rosrun perception pole_detector.py`

**Window 4**
- `rosrun perception light_detector.py`





