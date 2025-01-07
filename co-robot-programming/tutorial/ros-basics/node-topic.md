# Node, Topic

### 노드와 토픽이란 무엇인가?

#### **노드(Node)**

* ROS에서 실행 가능한 프로그램 단위로, 특정 작업(예: 센서 데이터 수집, 모터 제어 등)을 수행.
* 각 노드는 독립적으로 실행되며, 다른 노드와 데이터를 주고받기 위해 통신합니다.

#### **토픽(Topic)**

* 노드 간 데이터를 송수신하는 **비동기 통신 채널**.
* 퍼블리셔(Publisher)와 서브스크라이버(Subscriber)를 통해 데이터가 교환됩니다.
* 예: 카메라 노드가 `/camera/image_raw` 토픽을 통해 이미지 데이터를 퍼블리싱하고, 이를 처리하는 노드가 해당 데이터를 서브스크라이브.

### 실습: TurtleSim

#### 1. ROS 노드 실행

  1\)  **ROS 마스터 실행**: 모든 ROS 통신의 중심이 되는 `roscore` 실행.

    ```bash
    roscore
    ```
  2\)  **Turtlesim 노드 실행**:

    ```bash
    rosrun turtlesim turtlesim_node
    ```

    * GUI 창이 열리며 거북이가 나타납니다.
3)  **Turtlesim 제어 노드 실행**:

    ```bash
    rosrun turtlesim turtle_teleop_key
    ```

    * 키보드를 사용하여 거북이를 이동시킬 수 있습니다.

#### 2. 토픽 확인 및 데이터 송수신

1)  **현재 실행 중인 토픽 확인**:

    ```bash
    rostopic list
    ```

    * 실행 중인 모든 토픽 목록이 출력됩니다.
2)  **특정 토픽의 데이터 확인**:

    ```bash
    rostopic echo /turtle1/cmd_vel
    ```

    * `/turtle1/cmd_vel` 토픽에서 퍼블리싱되는 데이터를 실시간으로 확인.
3)  **토픽에 데이터 퍼블리싱**:

    ```bash
    rostopic pub /turtle1/cmd_vel geometry_msgs/Twist '{linear: {x: 2.0}, angular: {z: 1.0}}'
    ```

    * 거북이가 지정된 속도로 움직입니다.

#### 3. rqt\_graph를 통한 노드와 토픽 간 관계 확인

1)  **rqt\_graph 실행**:

    ```bash
    rqt_graph
    ```

    * 노드와 토픽 간의 관계를 시각적으로 확인할 수 있는 그래프가 표시됩니다.
2)  **노드와 토픽 구분**

    ![](https://github.com/user-attachments/assets/b01573f8-88f8-439f-bf5f-70fa3f84275e)

    * 노드
      * /teleop\_turtle : turtle1/cmd\_vel 정보를 publish
      * /turtlesim: turtle1/cmd\_vel 정보를 subscribe
    * 토픽
      * turtle1/cmd\_vel 정보

### 실습: Talker & Listener

#### 1. 노드와 토픽 실행

1)  launch 파일 실행

    ```bash
    roslaunch rospy_tutorials talker_listener.launch
    ```
2)  실행 결과

    * talker가 생성한 내용을 listener가 메시지로 받아서 화면에 출력한다.

    ![](https://user-images.githubusercontent.com/91526930/234394784-a24bfbb2-8f10-443e-b23d-f5dafda2532e.png)
3)  노드와 토픽 관찰하기

    ![](https://user-images.githubusercontent.com/91526930/234394161-ca099b10-639c-466d-9162-7fe709a4a39a.png)

#### 2. 관련 코드 확인하기

1)  Tutorial 폴더 접근

    ```bash
    roscd rospy_tutorials
    code .
    ```
2)  폴더 구성 확인

    * `.launch`와 `.py` 로 구성되어 있음.

    ![](https://user-images.githubusercontent.com/91526930/234396103-730b952f-d540-4871-b962-3101a73b3778.png)
3)  `talker_listener.launch`

    * 두 개의 node를 실행하는 것으로 구성됨.
    * 각 node를 초기화하는 구문은 각각의 python 파일에 작성되어 있음.

    ![](https://user-images.githubusercontent.com/91526930/234396233-154876be-05dc-4bba-b92e-f6e1e1acc233.png)
4)  `listener.py`

    ![](https://user-images.githubusercontent.com/91526930/234396748-210f85b3-f6da-42a1-8e1e-434460f27045.png)

    * 'listener'라는 노드를 `init_node`를 통해 초기화한다.
    * 'chatter'라는 String 형태의 msg 정보를 subscribing하고, callback함수를 실행한다.
    * callback 함수는 data를 받아서, terminal 창에 출력한다.
5)  `talker.py`

    ![](https://user-images.githubusercontent.com/91526930/234398302-2ef57b3a-b3d7-4d62-966b-13475a1e5971.png)

    * 'chatter'라는 String 형태의 msg 정보를 publishing하는 변수 pub을 선언한다.
    * 'talker'라는 노드를 초기화한다.
    * hello\_str에는 String 형태의 정보를 생산 및 할당한다.
    * publish 함수를 통해 hello\_str 변수를 publishing 한다.

### 요약

노드와 토픽은 ROS 시스템의 핵심 통신 메커니즘입니다. 이 실습을 통해 노드 실행, 토픽 데이터 송수신, 그리고 노드 간 관계를 시각적으로 확인하는 과정을 익힐 수 있습니다. 이를 기반으로 더 복잡한 시스템을 설계할 수 있습니다.
