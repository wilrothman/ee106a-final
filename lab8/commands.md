roslaunch realsense2_camera rs_camera.launch mode:=Manual color_width:=424 \
color_height:=240 depth_width:=424 depth_height:=240 align_depth:=true \
depth_fps:=6 color_fps:=6

rviz

(from source ~ee106a/sawyer_setup.bash)
roslaunch intera_examples sawyer_tuck.launch

