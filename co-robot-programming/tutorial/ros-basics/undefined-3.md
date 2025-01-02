# 이미지 딥러닝 모델 연동 예제

* ROS와 pytorch를 연동하여 프로그램을 실행시키는 예제
* 예제명: Image Classifier

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
        ├── image_display.py
        └── image_classifier.py
```

### 딥러닝 프레임워크 구축

#### 1. Anaconda 설치

#### 2. 딥러닝 프래임워크 설치

#### 3. bashrc 세팅

### 실습: 이미지 분류하기

#### 1. `my_package/src` 폴더 내부에 `image_classifier.py` 파일 생성

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

class ImgClassifierNode():
    def __init__(self, labels):
        rospy.init_node('image_classifier', anonymous=True) # 노드 초기화 및 이름 설정
        self.bridge = CvBridge()

        # Subscriber
        self.sub_image = rospy.Subscriber("camera/image_raw", Image, self.classify)  # camera/image_raw 토픽에서 Image 메시지 수신
        
        # Model init
        self.model = models.alexnet(weights=AlexNet_Weights.DEFAULT)
        self.model.eval()
        self.labels = labels

    def classify(self,data):
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
                percent = percentages[0].item()
                
                print(f"{label}, {percent:.2f}")
                
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
        image_classifier = ImgClassifierNode(imagenet_labels)       # CameraNode 객체 생성
        image_classifier.run()                # run 메서드 실행
    except rospy.ROSInterruptException:
        pass
```

####

#### 2. 빌드

catkin\_make 빌드 시, ros 환경대로 설정이 된 상황에서 빌드가 되어야 합니다.

```bash
[경로] ~/catkin_ws/
act_ros				# ros환경 활성화 (bashrc 사용자 정의 함수)
catkin_make
```

#### 3. 초기화

```bash
source ~/.bashrc 	# 또는 act_ros
```

#### 4. 실행권한 부여

```bash
chmod +x ~/catkin_ws/src/my_package/src/image_classifier.py
```

#### 5. 프로그램 실행

```
# terminal 1
act_ros		# 환경세팅
roscore
# terminal 2
# 환경세팅 (선택1) act_ros / (선택2) py38
rosrun my_package camera.py

# terminal 3
# 환경세팅 (terminal 2의 환경과 동일하게)
rosrun my_package image_display.py

# terminal 4
py38	# 환경세팅
rosrun my_package image_classifier.py
```

​
