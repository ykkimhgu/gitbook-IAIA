# 빌드 시스템

## 1. 빌드 시스템 개념

### **ROS에서** `catkin_make`**의 역할**

* `catkin_make`는 ROS 워크스페이스의 빌드 시스템으로, 노드와 메시지 파일을 포함한 프로젝트 구성 요소를 컴파일하고 실행 가능한 상태로 준비합니다.

### **CMakeLists.txt와 package.xml의 기능과 상호작용**

1. **CMakeLists.txt**:
   * 빌드 시스템에 필요한 설정을 정의합니다.
   * 소스 파일, 메시지 파일, 실행 파일 등을 포함시킵니다.
2. **package.xml**:
   * 패키지의 의존성을 관리합니다.
   * 메시지 파일이나 라이브러리에 대한 의존성을 명시합니다.

### **Python과 C++ 노드에서 빌드 설정의 차이**

* **Python**:
  * 인터프리터 언어이므로 빌드 과정이 필요하지 않습니다.
  * 실행 권한(`chmod +x`)과 `package.xml`에서의 의존성 설정만 중요합니다.
* **C++**:
  * 컴파일된 실행 파일이 필요하므로 `CMakeLists.txt`에서 빌드 설정이 필요합니다.
  * Msg 파일을 사용하는 경우, 자동 생성된 헤더 파일을 참조합니다.

## 2. 전체 빌드 흐름

### Python 기반 노드의 빌드 흐름

1. `package.xml`에 의존성을 추가합니다.
2. 실행 권한을 부여한 후 노드를 실행합니다.

### C++ 기반 노드의 빌드 흐름

1. `CMakeLists.txt`에 소스 파일 및 메시지 설정 추가.
2. `package.xml`에 의존성 추가.
3. `catkin_make` 실행.
4. 생성된 헤더 파일을 활용하여 노드 작성 및 실행.

### Msg 파일 추가 시의 빌드 흐름

1. **Msg 파일 정의**: 프로젝트에 필요한 메시지 파일 작성.
2. **CMakeLists.txt와 package.xml 수정**: Msg 파일을 빌드 과정에 포함.
3. `catkin_make` **실행**: Msg 파일을 기반으로 자동 생성된 헤더 파일 준비.
4. **퍼블리셔 및 서브스크라이버에서 사용**: 헤더 파일 참조.

## 3. 실습: 빌드 과정 통합

1. `Person.msg` **수정**:
   *   `msg/Person.msg` 파일에 새로운 필드를 추가합니다:

       ```rosmsg
       string name
       int32 age
       float32 height
       bool is_student
       ```
2. **CMakeLists.txt와 package.xml 업데이트**:
   * `CMakeLists.txt`에 Msg 파일 설정이 포함되어 있는지 확인합니다.
   * `package.xml`에 `message_generation`과 `message_runtime` 의존성이 포함되어 있는지 확인합니다.
3. **Python 퍼블리셔 및 서브스크라이버 수정**:
   *   퍼블리셔:

       ```python
       msg.is_student = True
       ```
   *   서브스크라이버:

       ```python
       rospy.loginfo(f"Received: Name={data.name}, Age={data.age}, Height={data.height}, Student Status: {data.is_student}")
       ```
4. **C++ 퍼블리셔 및 서브스크라이버 수정**:
   *   퍼블리셔:

       ```cpp
       msg.is_student = true;
       ROS_INFO("Publishing: Name=%s, Age=%d, Height=%.2f, Student Status: %s", msg.name.c_str(), msg.age, msg.height, msg.is_student ? "True" : "False");
       ```
   *   서브스크라이버:

       ```cpp
       ROS_INFO("Received: Name=%s, Age=%d, Height=%.2f, Student Status: %s", msg->name.c_str(), msg->age, msg->height, msg->is_student ? "True" : "False");
       ```
5.  **워크스페이스 빌드 및 실행**:

    ```bash
    cd ~/catkin_ws
    catkin_make

    rosrun my_package custom_publisher
    rosrun my_package custom_subscriber
    ```

## 요약

ROS 빌드 시스템은 Python과 C++ 기반 노드를 포함한 모든 구성 요소를 통합합니다. 이 실습을 통해 빌드 프로세스를 체계적으로 이해하고, 노드 간 통합 작업을 효과적으로 수행할 수 있습니다.
