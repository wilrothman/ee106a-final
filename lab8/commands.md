## Setup steps

**For all Windows**
- `Start from the folder 'final' to run all commands`

**Window 1**
- `source ~ee106a/sawyer_setup.bash`
- `roslaunch realsense2_camera rs_camera.launch mode:=Manual color_width:=424 color_height:=240 depth_width:=424 depth_height:=240 align_depth:=true depth_fps:=6 color_fps:=6`

**Window 2**
- `source ~ee106a/sawyer_setup.bash`
- `rosrun tf static_transform_publisher 0 0 0 0 0 0 base camera_link 100`

**Window 3**
- `source ~ee106a/sawyer_setup.bash`
- `cd lab8/`
- `catkin_make`
- `source devel/setup.bash`
- `rosrun perception pole_detector.py`

**Window 4**
- `source ~ee106a/sawyer_setup.bash`
- `cd lab8/`
- `catkin_make`
- `source devel/setup.bash`
- `rosrun perception light_detector.py`





