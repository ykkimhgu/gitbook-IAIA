# 이미지 정보 생성/불러오기

* ROS는 카메라로부터 영상을 추출하여 이미지 처리가 가능함.
* 내장 카메라, USB 카메라 연결하여 영상이미지를 추출하는 실습 진행
* 이미지에 대한 메시지 송수신 관계를 파악.
* launch 파일 구성 및 파라미터 입력.
* USB-Camera 패키지 설치

### Program Structure

```bash
catkin_ws/src
└── my_package
    ├── CMakeLists.txt
    ├── msg
    │   └── Person.msg
    ├── package.xml
    └── src
        ├── custom_publisher.cpp
        ├── custom_publisher.py
        ├── custom_subscriber.cpp
        ├── custom_subscriber.py
        ├── camera.py
        └── image_display.py
```

### 실습: 이미지 정보 생성/출력

#### 1. `tutorial/src` 폴더 내부에 `camera.py` 파일 생성

```python
#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image               # sensor_msg 패키지로부터 Image type을 import함
from cv_bridge import CvBridge, CvBridgeError   # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능

class CameraNode:

    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)  # 노드 이름 "camera_node"로 초기화
        self.bridge = CvBridge()                # cv_bridge 객체 생성

        # "camera/image_raw"라는 토픽으로 메시지를 publish할 publisher 객체 생성
        self.image_pub = rospy.Publisher("camera/image_raw",Image,queue_size=1)    
        
        self.cap = cv2.VideoCapture(0)          # 카메라 연결을 위한 VideoCapture 객체 생성

    def run(self):
        rate = rospy.Rate(30)                           # 루프 실행 주기 : 30hz
        while not rospy.is_shutdown():                  # ROS가 종료되지 않은 동안
            ret, frame = self.cap.read()                # 카메라로부터 이미지를 읽음
            if ret:                                     # 이미지가 정상적으로 읽혀진 경우
                try:
                    # 읽어들인 이미지를 ROS Image 메시지로 변환하여 토픽으로 publish
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                
                except CvBridgeError as e:
                    print(e)                            # CvBridge 변환 예외 처리
            rate.sleep()                                # 지정된 루프 실행 주기에 따라 대기

if __name__ == '__main__':
    try:
        camera = CameraNode()       # CameraNode 객체 생성
        camera.run()                # run 메서드 실행
    except rospy.ROSInterruptException:
        pass
```

#### 2. `my_package/src` 폴더 내부에 `image_display.py` 파일 생성

```python
#!/usr/bin/env python3
#-*- coding:utf-8 -*- 

import rospy
from sensor_msgs.msg import Image   # sensor_msgs 패키지로부터 Image 메시지 타입을 import
from cv_bridge import CvBridge      # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능
import cv2                          # OpenCV 라이브러리

class DisplayNode:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.callback)  # camera/image_raw 토픽에서 Image 메시지 수신

    def callback(self,data):
        try:
            # 수신된 Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Camera", cv_image)  # 변환된 이미지를 "Camera"라는 이름의 윈도우에 표시
        cv2.waitKey(1)                  # 1ms 동안 키보드 입력 대기

    def run(self):
        rospy.init_node('display_node', anonymous=True) # 노드 초기화 및 이름 설정
        rospy.spin()                                    # 노드가 종료될 때까지 계속 실행

if __name__ == '__main__':
    try:
        display = DisplayNode()     # DisplayNode 클래스의 인스턴스 생성
        display.run()               # 노드 실행
    except rospy.ROSInterruptException:
        pass
```

#### 3. `my_package/package.xml` 내용 추가

```xml
<build_depend>sensor_msgs</build_depend>
<build_depend>cv_bridge</build_depend>
<build_export_depend>sensor_msgs</build_export_depend>
<build_export_depend>cv_bridge</build_export_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>cv_bridge</exec_depend>
```

#### 4. `my_package/CMakeList.txt` 수정

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs	# 추가
  cv_bridge		# 추가
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs	# 추가
)

catkin_package(
 CATKIN_DEPENDS  message_runtime sensor_msgs cv_bridge # 추가
)
```

#### 5. 패키지 빌드

```bash
[위치] ~/catkin_ws
catkin_make
```

#### 6. 파이썬 스크립트에 대해 실행권한 허용.

```bash
chmod +x ~/catkin_make/src/my_package/src/camera.py
chmod +x ~/catkin_make/src/my_package/src/image_display.py
```

#### 7. 코드 실행

```bash
roscore									# terminal 1
rosrun tutorial camera.py			# terminal 2
rosrun tutorial image_display.py		# terminal 3
rqt_graph								# terminal 4
```

![](https://user-images.githubusercontent.com/91526930/235357562-126a214d-7139-4701-8e90-07a87a32ca53.png)

### 실습: Launch파일로 parameter 입력받도록 코드 수정하기

#### 1. `my_package/launch` 폴더 내부에 `image_display.launch` 파일 생성하기

* `<arg ~ />`: 파라미터 변수 선언
* `<param ~ />`: 파라미터 변수의 값을 할당하기

```python
<launch>
  <!-- Argument for the camera number, default to 0 -->
  <arg name="camera_number" default="0"/>

  <node name="camera"           pkg="my_package" type="camera.py" output="screen">
    <param name="camera_number" value="$(arg camera_number)" />
  </node>
  <node name="image_display"    pkg="my_package" type="image_display.py" output="screen"/>
  
</launch>
```

#### 2. `camera.py` 수정하기

* `rospy.get_param()`을 통해 parameter인 `camera_number`를 가져오기

```python
#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image               # sensor_msg 패키지로부터 Image type을 import함
from cv_bridge import CvBridge, CvBridgeError   # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능

class CameraNode:

    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)  # 노드 이름 "camera_node"로 초기화
        self.bridge = CvBridge()                # cv_bridge 객체 생성

        # Get camera number from ROS parameter, default to 0
        camera_number = rospy.get_param('~camera_number', 0)
        rospy.loginfo(f"Camera number received: {camera_number}")

        # "camera/image_raw"라는 토픽으로 메시지를 publish할 publisher 객체 생성
        self.image_pub = rospy.Publisher("camera/image_raw",Image,queue_size=1)    
        
        self.cap = cv2.VideoCapture(camera_number)          # 카메라 연결을 위한 VideoCapture 객체 생성

    def run(self):
        rate = rospy.Rate(30)                           # 루프 실행 주기 : 30hz
        while not rospy.is_shutdown():                  # ROS가 종료되지 않은 동안
            ret, frame = self.cap.read()                # 카메라로부터 이미지를 읽음
            if ret:                                     # 이미지가 정상적으로 읽혀진 경우
                try:
                    # 읽어들인 이미지를 ROS Image 메시지로 변환하여 토픽으로 publish
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                
                except CvBridgeError as e:
                    print(e)                            # CvBridge 변환 예외 처리
            rate.sleep()                                # 지정된 루프 실행 주기에 따라 대기

if __name__ == '__main__':
    try:
        camera = CameraNode()       # CameraNode 객체 생성
        camera.run()                # run 메서드 실행
    except rospy.ROSInterruptException:
        pass
```

#### 3. launch 파일 실행

```bash
roslaunch tutorial image_display.launch camera_number:=0
```

### 실습: USB Camera 연결

* reference: [ROS wiki - usb\_cam](http://wiki.ros.org/usb_cam)

#### 1. 카메라 포트 확인

*   내장 카메라 포트 확인

    ```bash
    ls -ltr /dev/video*		# 연결된 카메라의 포트 확인
    ```
*   추가 연결 카메라의 포트 확인

    ```bash
    sudo apt-get install v4l-utils -y   # v4l2 비디오 출력장치 제어 드라이버 설치하기
    v4l2-ctl --list-devices       # 카메라 포트 확인
    ```

#### 2. 카메라 포트 반영: `camera.py`

```python
self.cap = cv2.VideoCapture(1)          # usb camera에 해당하는 포트 반영(e.g. 1)
```

####

### 실습: 머신비전용 카메라: Pointgrey Camera

* 모델명: PointGrey - Black fly 3
* Specification
  * 1288 x 964 해상도
  * 30fps
  * USB 연결
  * [Github - pointgrey\_camera\_driver](https://github.com/ros-drivers/pointgrey_camera_driver)
  * 카메라 렌즈 장착 필요

#### 1. Driver 설치

```bash
sudo apt-get install ros-noetic-pointgrey-camera-driver
sudo apt-get update
```

#### 2. 카메라 인식

```bash
roscore                            # Terminal 1
rosrun pointgrey_camera_driver list_cameras    # Terminal 2
```

![](https://user-images.githubusercontent.com/91526930/235361466-d02e984e-1d5d-402c-8a77-4bc341d13c3e.png)

**인식이 잘 안될 경우, USB 다시 연결 또는 재부팅 시도**

#### 3. 작동 확인

```bash
roslaunch pointgrey_camera_driver camera.launch camera:=/15384429
rqt_graph
```

![](https://user-images.githubusercontent.com/91526930/235361525-e59f1935-3f0c-4622-bd1c-3efb861abb4d.png)

*   현재, 여러 노드와 토픽이 생성된 것을 확인할 수 있음. 특히,`/camera/image_raw`토픽은 우리가 알고 있는 msg\_type이므로, subscribing 가능

    ```bash
    rosrun tutorial image_display.py
    ```

    ![](https://user-images.githubusercontent.com/91526930/235361805-789a28b5-9876-4041-9d36-c523611271b3.png)
* 필요하다면, `image_mono` 와 `image_color` 토픽을 subscribing 할 수 있음.
