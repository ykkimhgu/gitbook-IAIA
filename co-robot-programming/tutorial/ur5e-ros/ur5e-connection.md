# UR5e Connection

### Required Packages

아래의 패키지 내역이 `catkin_ws/src` 내에 포함되어 있는지 확인

* `rg2_description`
* `universal_robot`
* `Universal_Robots_ROS_Driver`
* `ur_python`
* `ur5e_rg2_moveit_config`

패키지가 없으면, [ROS Packages for Robots](../../installation/ros/ros-packages-for-robots.md) 자료내용 참고



### 1. UR5e Hardware Connection

#### 1) Control box 전원 켜기

* 초록색 전원버튼을 누르면, 약 3분 동안 부팅과정을 거친다.
* control box의 전원이 완전히 켜지면, 다음의 사진과 같다.

![](https://user-images.githubusercontent.com/91526930/234139952-68a2a54d-3a10-4dde-b968-36ebcb89adb1.png)


#### 2) 로봇 프로그램 불러오기

* 미리 저장되어 있는 로봇 프로그램을 불러온다.
* 로봇 프로그램은 로봇의 동작을 정의하는 것을 말하며, 티칭 펜던트(조작하는 태블릿)에서 프로그램을 작성할 수 있다.
* 화면 상단에 \[열기]를 클릭한다.

![](https://user-images.githubusercontent.com/91526930/234139914-7596143b-bc0c-4dbe-ad50-744752f62f3c.png)

* 프로그램 선택
  * `ros_rg2.urp`: RG2 gripper 연결한 경우
  * `ros_vg.urp`: Vaccume 연결한 경우

![](https://user-images.githubusercontent.com/91526930/234140049-8c69bc7f-fc68-4255-802e-0981ada74a3a.png)


#### 3) 로봇 전원 On

* 화면의 좌측 하단에 \[전원꺼짐] 빨간 버튼을 누르면, 로봇의 상태를 보여준다.
* 화면 중간에, \[켜짐] 초록 버튼을 누르면, 로봇에 전원을 인가한다.
* \[시작] 초록 버튼을 눌러, 로봇을 활성화한다.

![](https://user-images.githubusercontent.com/91526930/234140283-fdfdcb14-15f2-44e7-9812-b3434a63759c.png) 

![](https://user-images.githubusercontent.com/91526930/234140311-425d50be-bbef-44c7-9e18-c4fa7a1d82f3.png) 

![](https://user-images.githubusercontent.com/91526930/234140351-9ba12fbc-da0b-46c1-85d0-9b40b49614cd.png)



#### 4) 로봇 프로그램

* 로봇의 동작 시퀀스를 프로그램으로 구성함
* `192.168.0.10` IP 주소를 갖는 외부의 PC가 로봇팔에 명령을 전달할 수 있는 상태가 됨
* 스레드(Thread)는 외부 PC의 명령과는 별개로 백그라운드에서 동작하는 것을 의미함
* 아래의 예시는, 디지털 출력핀\[1]이 True/False 여부에 따라 그리퍼/Vacuum의 동작을 정의한 것이다. ros를 통해 rg2-gripper/Vacuum을 직접 제어할 수 없는 상황이기에, 하드웨어적으로 디지털 출력핀의 설정을 통해 동작을 정의하였다.

![](https://user-images.githubusercontent.com/91526930/234140416-2553909d-5412-4f10-abd1-eb46c30aa5c0.png)



#### 5) PC 연결

* 랜선으로 PC와 연결한 후, 유선 네트워크를 다음과 같이 설정한다.

![](https://user-images.githubusercontent.com/91526930/234139649-6139dcf5-b84a-41f2-9ace-b71725601155.png)



### 2. Software Connection

#### Terminal

```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2
```



### 3. 로봇 프로그램 실행 (Teaching Pendant)

- 우측 하단의 재생버튼을 눌러 로봇 프로그램을 실행
- 실행버튼을 누르면, `192.168.0.10` 과 연결을 시도함
- 연결이 성공하면,  `192.168.0.10`의 명령에 따라 로봇팔이 움직이도록 대기하는 상태가 됨

![](https://user-images.githubusercontent.com/91526930/234138529-75eb185e-f308-400f-aebb-d2f79e8b3ffb.png)



### 4. 시뮬레이션 실행 (Moveit)

#### Terminal

```bash
roslaunch ur5e_rg2_moveit_config move_group.launch
```



### 5. Check Initialization in demo file

```python
ur5e = MoveGroupPythonInterface(real="real")
```



### 6. 데모 프로그램 실행

#### Terminal

```bash
rosrun ur_python ex1_move.py
```
