# IROS2025
Version1


## Features

This is for IROS2025. This is for automatical grabbing purposes.

- The `baseline` inherits the previous control method but with obstacle avoidance and with updated fuzzy control for base control. The robot end-effectors will strictly follow the user's arm movement unless it detects that the user's movement will lead to collision with existing obstacles. In that case, it will erase the direction that may cause collision but carry on other directions.(For example, if the robot end-effectors is above the task, and user is trying to grab a bottle on the table. The end-effector will stop move downward(in z direction) but will allow user to move in planar(XY directions) 

- The `egocentric` is the novel control method also called `notouchpad` method which presented in the paper. It abandons the tradition key-press control. Instead, it allows the user to control the entire robot only use arm movement and head rotation. To be more specific, for the arm control, there exists a workspace for the arm which is calculated through maneuverability. In which, it guarantees that no collisions with other part and no singularity. When the robot end-effectors are within the workspace, it strictly follows the user's arm movement. However, when the robot end-effectors hit the boundary, depending on which boundary it hits, the robot will adjust base or torso. Similarly, the head can also control the base rotation and torso up/down movement Safety 

- The `Combine` allows the users to access the both above control method, which means the robot end-effectors will remain in the workspace and will activate base and torso control when hit the boundary while the user can also control the base and torso through key-pressing.


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

To activate the `egocentric`, use the following code:

`roslaunch vive_teleop vive_teleop_continue_notouchpad.launch sim=false record=false rviz=true`

To activate the `combine`, use the followwing code:

`roslaunch vive_teleop vive_teleop_continue_fall.launch sim=false record=false rviz=true`

## UI

There exists two forms of UI. The `ui_baseline.py` is for the `baseline` method and `ui_ego` is for the `egocentric` and `combine` method.

- `ui_baseline.py` adds obstacle warnings, obstacle frames and robot's end-effector and elbow current location with respect to the base frame (torso_lift_link). 

- `ui_ego.py` besides everything above, it adds the boundary for the workspace and warnings for arm reaches the boundaries.

- For usage, in you workspace `python3 ui_baseline.py/ui_ego.py`. Remember to connect to the Unity first.

## Scripts overview

This section brefly illustrates the function of each `.py` in folder `scripts`, for more detail information please read the comments within each file.

- `rand.py` gives a random series contains 1-6, which decides the user's task order.

- `data_recorder_continue.py` record controller data, robot frame data, robot to target data and all the triggered warnings.

- `ee_publisher.py` original file that only publish the warning whether robot end-effector with respect to the robot base frame is too large.

- `ee_publisher_fall.py` new version of tracking all the updated warnings including `arm_above_desk` which will freeze the torso from going down (up and down if task is shelf). `arm_move_collsion` which will freeze the base if arm is too close to the obstacle. `arm_next_target_collision` which will freeze the arm moving if it detects the user next move will hit the obstacle. And all the boundary warnings.

- `vive_pose_mapping.py` the old version that maps all the user movement to the robot end effector.

- `vive_pose_mapping_continue.py` the second version that maps all the user movement to the robot end effector and if the arm reaches the boundary, the arm will stop moving.

- `vive_pose_mapping_fall.py` the latest version that maps all the user movement to the robot end effector and if the arm reaches the boundary, it will stop moving across the boundary but can rotate and sliding along the boundary.

- `vive_teleop_continue.py` the base, torso, head control for the `combine` method.

- `vive_teleop_continue_notouchpad.py` the base, torso, head control for the `egocentric` method.

- `vive_teleop_v2.py` the base, torso, head control for the `base` method.
