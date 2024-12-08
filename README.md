ee106a-aiz API code: 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

## Setup steps
**Window 1**
- `cd ros_workspaces/final`
- `source ~ee106a/sawyer_setup.bash`
- `rosrun intera_interface enable_robot.py -e`
- `rosrun intera_interface joint_trajectory_action_server.py`

**Window 2**
- `source ~ee106a/sawyer_setup.bash`
- `roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true`

**Window 3**
- `source ~ee106a/sawyer_setup.bash`

## Common commands
Once you have set up everything, here is how to run the following commands.

**Regular tuck**
- `roslaunch intera_examples sawyer_tuck.launch`

**Custom tuck**
- `roslaunch move_arm custom_saqwyer_tuck.launch`

**Lab 5 MoveIt example (for testing movement)**
- `catkin_make`
- `source devel/setup.bash`
- `rosrun move_arm ik_example.py`

**ik_final**
- `catkin_make`
- `source devel/setup.bash`
- `rosrun move_arm ik_final.py`


## Specific Commands

**Get gripper position and orientation**
- `rosrun tf tf_echo base right_gripper_tip`

??? Doc
https://docs.google.com/document/d/1FzzBy37gV783baF9xRSmaKsRRo2vlI9W1f_YD7CHH_s/edit?tab=t.0
