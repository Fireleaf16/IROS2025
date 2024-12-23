# IROS2025
Version1


## Features

This is for IROS2025. This is for automatical grabbing purposes within distances. It can be mainly divided into three parts of control: `arm_control`,`torso_control` and `base_control`

- The `arm_control` inherits the previous control method but but now add additional control strategy. Through keyboard, the user can now adjust pick up mode (RRT-based-autograb/teleop) and adjust pick and place target. When the autograb mode is selected and the target locates within the robot end-effector workspace, it will first deploy improved RRT planning to get the path and then activate the path through rapid IK solver. To be noticed, the arm_control will only be activated once the robot is in optimal grabbing locations, meaning that it will face towards the target, be close enough and at proper height.

- The `torso_control` is directly achieved through the torso command line. When autograb mode is selected, and when the target locates out of the workspace in z direction. The robot will automatically calculate the difference bewteen the height between the target and the original arm_home_place. And send the result to the torso command line which decides whether the torso will goes up or down that amout of result.

- The `base_control` implements the PID control for rotation and transition. To be more specific, when the target is out of the workspace in XY direction or existing a large gap between the current orientaion of the robot, it will trigger the robot base control. The robot will firstly adjust its orientation towards the target using PID control and then move close enough to the target waiting to pick up. It should be mentioned that the order in this case really matters as the XY-speed command given to the robot is in robot-frame, not in world/odom frame.


## Usage
 
To activate the Tiago robot, first git the repository(which should be done in the lab computer). Connect to the robot first and then open another terminal with the following command：

```
Master
IP
cd Tiago_dual_robot
source devel/setup.bash
```

To activate the program, use the following code:

`roslaunch vive_teleop vive_teleop_iros.launch sim=false record=false rviz=true`


## UI

We only use `ui_baseline.py` for the auto grabbing mode.

- `ui_baseline.py` adds obstacle warnings, obstacle frames and robot's end-effector and elbow current location with respect to the base frame (torso_lift_link). 

- For usage, in you workspace `python3 ui_baseline.py/ui_ego.py`. Remember to connect to the Unity first.

## Scripts overview

This section brefly illustrates the function of each `.py` in folder `scripts`, for more detail information please read the comments within each file.

- `data_recorder_continue.py` record controller data, robot frame data, robot to target data and all the triggered warnings.

- `ee_publisher.py` original file that only publish the warning whether robot end-effector with respect to the robot base frame is too large.

- `ee_publisher_fall.py` new version of tracking all the updated warnings including `arm_above_desk` which will freeze the torso from going down (up and down if task is shelf). `arm_move_collsion` which will freeze the base if arm is too close to the obstacle. `arm_next_target_collision` which will freeze the arm moving if it detects the user next move will hit the obstacle. And all the boundary warnings.

- `vive_pose_mapping.py` the old version that maps all the user movement to the robot end effector.

- `vive_pose_mapping_continue.py` the second version that maps all the user movement to the robot end effector and if the arm reaches the boundary, the arm will stop moving.

- `vive_pose_mapping_fall.py` the latest version that maps all the user movement to the robot end effector and if the arm reaches the boundary, it will stop moving across the boundary but can rotate and sliding along the boundary.

- `vive_teleop_continue.py` the base, torso, head control for the `combine` method.

- `vive_teleop_continue_notouchpad.py` the base, torso, head control for the `egocentric` method.

- `vive_teleop_v2.py` the base, torso, head control for the `base` method.
