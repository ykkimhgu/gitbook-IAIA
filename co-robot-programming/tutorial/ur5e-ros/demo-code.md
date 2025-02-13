# Example Code



### Ex 1: Simple Movement

```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2
roslaunch ur5e_rg2_moveit_config move_group.launch
rosrun ur_python ex1_move.py
```



### Ex 2: Gripper Operation

```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2
roslaunch ur5e_rg2_moveit_config move_group.launch
rosrun ur_python ex2_grip.py
```



### Ex 3: Pick & Place

```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2
roslaunch ur5e_rg2_moveit_config move_group.launch
rosrun ur_python ex3_pick_and_place.py
```



### Ex 4: Pet Feeder Robot

```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2
roslaunch ur5e_rg2_moveit_config move_group.launch
rosrun ur_python camera.py
rosrun ur_python image_display.py
rosrun ur_python pet_classifier.py    # conda environment
rosrun ur_python ex4_pet_feeder.py
```
