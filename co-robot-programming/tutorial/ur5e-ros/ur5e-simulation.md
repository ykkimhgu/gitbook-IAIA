# UR5e Simulation

UR5e 로봇에 대한 시뮬레이션 프로그램을 실행하는 실습을 진행합니다.

- **실습구성** 
  - Practice Movement Command
  - Gripper Operation
  - Basic Mission: Pick and Place
  - Application Mission: Pet Feeder Robot with Camera



* **주의사항**
  * 시뮬레이션 모델에서는 end-effector(rg2-gripper, vaccum gripper)가 반영되어 있지 않습니다.
  * 따라서, 실제 로봇 모델의 end-effector의 좌표와 다릅니다.



### Program Structure

```bash
catkin_ws/src
└── ur_python
    ├── CMakeLists.txt
    ├── package.xml
    └── msg
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



### Check Initialization in example files

<u>예제 파일에 시뮬레이션 모드로 설정되어 있는지 확인하세요.</u>

```python
ur5e = MoveGroupPythonInterface(real="sim")
```





### 1. Practice Movement Command



#### 1.1. Terminal 명령

```bash
roslaunch ur_gazebo ur5e_bringup.launch
roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true
rosrun ur_python ex1_move.py
```

- **`ur_gazebo`**: gazebo 시뮬레이션 환경에서 UR robot을 불러올 수 있는 패키지
- **`ur5e_moveit_config`**: UR5e 모델에 대한 주요 정보(컨트롤러, 경로 산출 함수, 파라미터 등)가 내장된 패키지
  - **`moveit`**이라는 도구를 통해 **planning**과  **execution** 함수가 실행됨 (로봇팔의 경로를 계산하고 움직이게 함.)



#### 1.2. 소스코드: `ex1_move.py`

```python
#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import rospy
import numpy as np
from move_group_python_interface import MoveGroupPythonInterface
from math import tau

DEG2RAD = tau / 360.0

def main():
  try:

      ur5e = MoveGroupPythonInterface(real="sim")

      print("============ Moving initial pose using a joint state goal ...")
      # ur5e.move_to_standby()
      init_pose_joints = [tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0]          # tau = 2 * pi
      ur5e.go_to_joint_abs(init_pose_joints)

      input("============ Press `Enter` to execute a movement using a joint state goal(relative) ...")
      joint_rel = [1/8 * tau, 0, 0, 0, 0, 0]          # tau = 2 * pi
      ur5e.go_to_joint_rel(joint_rel)

      input("============ Press `Enter` to execute a movement using a joint state goal(relative) ...")
      joint_rel = [-1/8 * tau, 0, 0, 0, 0, 0]          # tau = 2 * pi
      ur5e.go_to_joint_rel(joint_rel)

      input("============ Press `Enter` to execute a movement using a relative pose ...")
      target_pose_rel_xyz = [0.0, 0.0, 0.0]
      target_pose_rel_rpy = [tau/8, 0, 0]
      ur5e.go_to_pose_rel(target_pose_rel_xyz, target_pose_rel_rpy)

      input("============ Press `Enter` to execute a movement using a relative pose ...")
      target_pose_rel_xyz = [0.0, 0.0, 0.0]
      target_pose_rel_rpy = [-tau/8, 0, 0]
      ur5e.go_to_pose_rel(target_pose_rel_xyz, target_pose_rel_rpy)

      input("============ Press `Enter` to execute a movement using a absolute pose ...")
      target_pose_abs_xyz = [-0.13, 0.49, 0.47]
      target_pose_abs_rpy = [-2.3939, -0.00, -0.00]
      ur5e.go_to_pose_abs(target_pose_abs_xyz, target_pose_abs_rpy)


      print("============ complete!")

  except rospy.ROSInterruptException:
      return
  except KeyboardInterrupt:
      print("Shut down by Key Interrupt")
      return

if __name__ == "__main__":
  main()
```



#### 1.3. 소스코드 분석하기

**MoveGroupPythonInterface()**: ur5e 로봇팔에 움직임 명령을 주는 인터페이스 역할을 하는 클래스로서, `move_group_python_interface.py` 파일에 정의됨.

<u>아래는 로봇팔을 움직이게 하는 명령어(함수) 목록이며, 각각 원리가 다릅니다. 그 원리를 파악해보세요.</u>

- go_to_joint_abs()
- go_to_joint_rel()
- go_to_pose_abs()
- go_to_pose_rel()



<u>위 함수목록 내부에 공통적으로 내장된 구문 및 메세지 정보를 파악하고, 왜 필요한 것인지 분석하세요.</u>





### 2. Gripper Operation

<u>그리퍼 동작은 로봇과 연결된 후, 진행 가능합니다.</u>

#### 2.1. Terminal 명령

```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2
roslaunch ur5e_rg2_moveit_config move_group.launch
rosrun ur_python ex2_grip.py
```



#### 2.2. 소스코드: `ex2_grip.py`

```python
#! /usr/bin/env python3
#-*- coding:utf-8 -*- 

import rospy
from move_group_python_interface import MoveGroupPythonInterface
from math import tau

def main():
    try:

        ur5e = MoveGroupPythonInterface()

        input("============ Press `Enter` to execute a movement using a joint state goal ...")
        target_joints = [tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0]          # tau = 2 * pi
        ur5e.go_to_joint_abs(target_joints)

        input("============ Press `Enter` to grip on ...")
        ur5e.grip_on()

        input("============ Press `Enter` to grip off ...")
        ur5e.grip_off()

        print("============ Python tutorial demo complete!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        print("Shut down by Key Interrupt")
        return

if __name__ == '__main__':
    main()
```



#### 2.3. 소스코드 분석하기

<u>grip_on()과 grip_off() 함수를 분석해보세요</u>





### 3. Basic Mission: Pick & Place

#### 3.1. Terminal 명령

```bash
roslaunch ur_gazebo ur5e_bringup.launch
roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true
rosrun ur_python ex3_pick_and_place.py
```



#### 3.2. 소스코드: `ex3_pick_and_place.py`

```python
#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import sys
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs

import tf
import numpy as np
from math import pi

from move_group_python_interface import MoveGroupPythonInterface

DEG2RAD = pi/180
RAD2DEG = 180/pi

def main():
    try:
        print("grip")
        ur5e = MoveGroupPythonInterface(real="sim")
        
        ur5e.move_to_standby()
        # ur5e.grip_off()
        
        # go to A top
        pos_A = np.array([30.16, -95.54, 94.27, -88.67, -89.83, -57.71])*DEG2RAD
        ur5e.go_to_joint_abs(pos_A)

        # go to A bottom
        rel_xyz = [0.0, 0.0, -0.213]
        rel_rpy = [0.0, 0.0, 0.0]
        ur5e.go_to_pose_rel(rel_xyz, rel_rpy)

        # grip
        # ur5e.grip_on()

        # go to A top
        rel_xyz = [0.0, 0.0, 0.213]
        rel_rpy = [0.0, 0.0, 0.0]
        ur5e.go_to_pose_rel(rel_xyz, rel_rpy)


        # go to B top
        pos_B = np.array([112.29, -91.15, 92.10, -90.90, -89.82, 19.23])*DEG2RAD 
        ur5e.go_to_joint_abs(pos_B)

        # go to B bottom
        rel_xyz = [0.0, 0.0, -0.178]
        rel_rpy = [0.0, 0.0, 0.0]
        ur5e.go_to_pose_rel(rel_xyz, rel_rpy)

        # grip
        # ur5e.grip_off()

        # go to B top
        rel_xyz = [0.0, 0.0, 0.178]
        rel_rpy = [0.0, 0.0, 0.0]
        ur5e.go_to_pose_rel(rel_xyz, rel_rpy)

        ur5e.move_to_standby()

        print("complete")
    
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        print("Shut down by Key Interrupt")
        return


if __name__ == '__main__':
    main()  
```



#### 3.3. 소스코드 분석하기

<u>기본적인 미션인 pick & place의 의미를 생각하며, 위 코드의 구성 및 배치가 어떻게 되어있는지 분석하세요.</u>





### 4. Application Mission: Pet Feeder Robot with Camera

#### 4.1. Terminal 명령

```bash
roslaunch ur_gazebo ur5e_bringup.launch
roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true
rosrun ur_python camera.py
rosrun ur_python image_display.py
rosrun ur_python pet_classifier.py  # 딥러닝 사용가능한 terminal 환경으로
rosrun ur_python pet_feeder.py
```



#### 4.2. 소스코드: `camera.py`

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
      
      # self.cap = cv2.VideoCapture(camera_number)          # 카메라 연결을 위한 VideoCapture 객체 생성
      self.cap = cv2.VideoCapture(0)

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



#### 4.3. 소스코드: `image_display.py`

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



#### 4.4. 소스코드: `pet_classifier.py`

```python
#!/usr/bin/env python3
#-*- coding:utf-8 -*- 

import rospy
from sensor_msgs.msg import Image   # sensor_msgs 패키지로부터 Image 메시지 타입을 import
from cv_bridge import CvBridge, CvBridgeError      # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능
import cv2                          # OpenCV 라이브러리
import numpy as np

import torch
from torchvision import models
from torchvision.models import AlexNet_Weights
from PIL import Image as PILImage
from torchvision import transforms
import torch.nn.functional as F

import json
import urllib

from ur_python.msg import robot_state, pet_info

transforms = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),           
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
        )])



# ImageNet 클래스 레이블을 로드하는 함수
def load_imagenet_labels():
url = "https://raw.githubusercontent.com/anishathalye/imagenet-simple-labels/master/imagenet-simple-labels.json"
filename = "imagenet_classes.json"
urllib.request.urlretrieve(url, filename)

with open(filename, 'r') as f:
    return json.load(f)

class PetClassifierNode():
def __init__(self, labels):
    rospy.init_node('pet_classifier', anonymous=True) # 노드 초기화 및 이름 설정
    self.bridge = CvBridge()

    
    self.bAction = False
    # Subscriber
    self.sub_image = rospy.Subscriber("camera/image_raw", Image, self.action)  # camera/image_raw 토픽에서 Image 메시지 수신
    self.sub_robot = rospy.Subscriber("robot_state", robot_state, self.chk_robot)  # camera/image_raw 토픽에서 Image 메시지 수신
    
    # Publisher
    self.pub_pet_info = rospy.Publisher('pet_classifier/pet_info', pet_info, queue_size=10)

    # Model init
    self.model = models.alexnet(weights=AlexNet_Weights.DEFAULT)
    self.model.eval()
    self.labels = labels

    # Message Control Variables
    self.bSend = False
    self.bAction = False
    self.pet_info = pet_info()
    self.thresh_cnt = 30
    self.chk_cnt = 0    
    self.pet_id_prev = 0
    self.pet_id_curr = 0
    

def chk_robot(self,robot_state):
    if robot_state.move == 1:
        self.bAction = False
    else:
        self.bAction = True

def action(self,data):
    if self.bAction:
        try:
            # 수신된 Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") 
            
            # OpenCV 이미지를 PIL 이미지로 변환
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            
            # 이미지 전처리
            input_tensor = transforms(pil_image)
            input_batch = input_tensor.unsqueeze(0)  # 배치 차원 추가

            # model 예측
            with torch.no_grad():
                output = self.model(input_batch)
                percentages = F.softmax(output, dim=1)[0] * 100
                _, indices = torch.sort(output, descending=True)
                
                label = self.labels[indices[0][0]]
                

                if label == "tabby cat":
                    self.pet_id_curr = 1
                elif label == "Golden Retriever":
                    self.pet_id_curr = 2
                else:
                    self.pet_id_curr = 0

                if (self.pet_id_curr == self.pet_id_prev) and self.pet_id_curr != 0:
                    self.chk_cnt += 1
                else:
                    self.chk_cnt = 0

                if self.chk_cnt >= self.thresh_cnt:
                    self.chk_cnt = 0
                    self.bSend = True

                if self.bSend:
                    self.pet_info.name = label
                    self.pub_pet_info.publish(self.pet_info)
                    self.bSend = False

                self.pet_id_prev = self.pet_id_curr

                rospy.loginfo("%s, %d, %d",label, self.pet_id_curr, self.chk_cnt)


            # cv2.imshow("Camera", cv_image)  # 변환된 이미지를 "Camera"라는 이름의 윈도우에 표시
            # cv2.waitKey(1)                  # 1ms 동안 키보드 입력 대기

        except Exception as e:
            print(e)

def run(self):
    rospy.spin()                                    # 노드가 종료될 때까지 계속 실행


if __name__ == '__main__':
try:
    # ImageNet 레이블 로드
    imagenet_labels = load_imagenet_labels()
    pet_classifier = PetClassifierNode(imagenet_labels)       # CameraNode 객체 생성
    pet_classifier.run()                # run 메서드 실행
except rospy.ROSInterruptException:
    pass
```



#### 4.5. 소스코드: `pet_feeder.py`

```python
#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import sys
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs

import tf
import numpy as np

from math import pi

from move_group_python_interface import MoveGroupPythonInterface
from ur_python.msg import pet_info

DEG2RAD = pi/180
RAD2DEG = 180/pi


feed_info = {   "pt_A_top"   :   {  "tabby cat"         : np.array([30.16, -95.54, 94.27, -88.67, -89.83, -57.71])*DEG2RAD   ,
                                "Golden Retriever"  : np.array([30.16, -95.54, 94.27, -88.67, -89.83, -57.71])*DEG2RAD   },
            "pt_A_bottom":   {  "tabby cat"         : np.array([30.13, -87.14, 129.82, -122.63, -89.81, -57.64])*DEG2RAD  ,
                                "Golden Retriever"  : np.array([30.13, -87.14, 129.82, -122.63, -89.81, -57.64])*DEG2RAD  },
            "pt_B_top"   :   {  "tabby cat"         : np.array([112.29, -91.15, 92.10, -90.90, -89.82, 19.23])*DEG2RAD    ,
                                "Golden Retriever"  : np.array([112.29, -91.15, 92.10, -90.90, -89.82, 19.23])*DEG2RAD    },
            "pt_B_bottom":   {  "tabby cat"         : np.array([112.26, -85.02, 112.84, -117.77, -89.80, 19.29])*DEG2RAD   ,
                                "Golden Retriever"  : np.array([112.26, -85.02, 112.84, -117.77, -89.80, 19.29])*DEG2RAD   },
            
            "quantity"  :   {   "tabby cat"         : 2,
                                "Golden Retriever"  : 4},

            "up_A"       :   {  "rel_xyz"           : [0.0, 0.0, 0.213]     ,
                                "rel_rpy"           : [0.0, 0.0, 0.0]      },
            "down_A"     :   {  "rel_xyz"           : [0.0, 0.0, -0.213]    ,
                                "rel_rpy"           : [0.0, 0.0, 0.0]      },
            "up_B"       :   {  "rel_xyz"           : [0.0, 0.0, 0.178]     ,
                                "rel_rpy"           : [0.0, 0.0, 0.0]      },
            "down_B"     :   {  "rel_xyz"           : [0.0, 0.0, -0.178]    ,
                                "rel_rpy"           : [0.0, 0.0, 0.0]       }
        }


class PetFeederNode():
def __init__(self):
    # rospy.init_node('pet_feeder', anonymous=True) # 노드 초기화 및 이름 설정

    # Subscriber
    self.sub_pet_class = rospy.Subscriber("pet_classifier/pet_info", pet_info, self.feed)  # camera/image_raw 토픽에서 Image 메시지 수신

    # Robot initialization
    self.robot = MoveGroupPythonInterface(real="sim")
    self.robot.move_to_standby()
    # self.robot.grip_off()

def feed(self, pet_info):
    
    for i in range(feed_info["quantity"][pet_info.name]):
        
        # point A
        # go_to_pose_abs(self, absolute_xyz, absolute_rpy)
        self.robot.go_to_joint_abs(feed_info["pt_A_top"][pet_info.name])
        self.robot.go_to_pose_rel(feed_info["down_A"]['rel_xyz'], feed_info["down_A"]['rel_rpy'])
        # self.robot.grip_on()
        self.robot.go_to_pose_rel(feed_info["up_A"]['rel_xyz'], feed_info["up_A"]['rel_rpy'])

        # point B
        self.robot.go_to_joint_abs(feed_info["pt_B_top"][pet_info.name])
        self.robot.go_to_pose_rel(feed_info["down_B"]['rel_xyz'], feed_info["down_B"]['rel_rpy'])
        # self.robot.grip_off()
        self.robot.go_to_pose_rel(feed_info["up_B"]['rel_xyz'], feed_info["up_B"]['rel_rpy'])

    self.robot.move_to_standby()
        
def run(self):
    rospy.spin()                                    # 노드가 종료될 때까지 계속 실행

if __name__ == '__main__':
try:
    pet_feeder = PetFeederNode()
    pet_feeder.run()                # run 메서드 실행
except rospy.ROSInterruptException:
    pass

```



#### 4.6. 소스코드 분석하기

<u>`rqt_graph`를 통해 노드와 토픽간 주고 받는 정보의 흐름을 파악해보세요.</u>

- 카메라 센서로부터 취득된 정보는 무엇인지
- 이미지 정보가 어디로 전달되는지
- 이미지 정보로부터 어떤 정보를 생성하는지
- 생성된 정보가 로봇팔에 어떻게 전달되는지

