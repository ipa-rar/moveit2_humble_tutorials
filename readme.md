# MoveIt2 foxy ur5e_workcell demo

- To launch the robot in rviz
```
 ros2 launch  ur5e_workcell_support urdf_launch.py
```
- To launch the workcell nodes
````
ros2 launch ur5e_workcell_support ur5e_workcell_launch.py
````
To send service request to the vision_node to calculate the TF of camera
````
ros2 run ur5e_workcell_core scannplan_node --ros-args -p base_frame:=table
````

## Moveit experiments

```
ros2 launch ur5e_workcell_moveit_config workcell_planning_execution_launch.py
ros2 launch ur5e_workcell_support ur5e_workcell_launch.py
```