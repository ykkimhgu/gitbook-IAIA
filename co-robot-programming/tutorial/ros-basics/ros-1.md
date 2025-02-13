# ROS 기본 명령어

### 1. 파일 및 경로 관련 명령어

현재 ROS 패키지 경로 이동

- 특정 패키지의 루트 디렉토리로 바로 이동함

```bash
roscd [package_name]
```



패키지 파일 검색

* 지정한 패키지의 설치 경로를 반환

```bash
rospack find [package_name]
```



### 2. 실행 관련 명령어

노드 실행
* 특정 패키지의 노드를 실행

```bash
rosrun [package_name] [node_name]
```



런치 파일 실행

* 런치 파일을 통해 여러 노드를 동시에 실행

```bash
roslaunch [package_name] [launch_file]
```



### 3. 상태 확인 명령어

노드 정보 확인

```bash
rosnode list
```



토픽 정보 확인

```bash
rostopic list
```



토픽 메시지 확인

* 특정 토픽의 메시지 내용을 실시간으로 출력

```bash
rostopic echo [topic_name]
```



### 4. 디버깅 명령어

rqt 도구 실행

* GUI 기반의 ROS 디버깅 및 모니터링 도구 실행

```bash
rqt
```



토픽 그래프 확인

* 노드와 토픽 간의 관계를 그래프로 시각화

```bash
rqt_graph
```
