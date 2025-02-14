# Example Codes



```bash
catkin_ws/src
└── ur_python
    ├── CMakeLists.txt
    ├── package.xml
    └── msg
        ├── grip_command.msg
        ├── grip_state.msg
        ├── robot_state.msg
        └── pet_info.msg
    └── src
        ├── move_group_python_interface.py
        ├── ex1_move.py
        ├── ex2_grip.py
        ├── ex3_pick_and_place.py
        ├── pet_feeder.py
        ├── camera.py
        ├── image_display.py
        └── pet_classifier.py
├── ur_gazebo
├── ur5e_rg2_moveit_config
└── rg2_description
```



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
rosrun ur_python pet_classifier.py    # 딥러닝 사용가능한 terminal 환경으로
rosrun ur_python ex4_pet_feeder.py
```
