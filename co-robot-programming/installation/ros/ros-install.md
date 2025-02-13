# ROS 설치

Reference: [ROS wiki - Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)

## 1. Setup sources & keys

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```


## 2. Installation

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```


## 3. Environment setup(bashrc setting)

```bash
gedit ~/.bashrc
```

* `~/.bashrc` 내부
  ```bash
  export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libtiff.so.5    # libtiff 버전 충돌 방지(ROS <-> Conda)
  source /opt/ros/noetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
  export PYTHONPATH=~/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages

  # 자동으로 ROS 환경 활성화
  function act_ros {
      source /opt/ros/noetic/setup.bash
      source ~/catkin_ws/devel/setup.bash
      export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:~/catkin_ws/devel/lib/python3/dist-packages
      echo "ROS activated"
  }

  # 별명 선언
  alias re='source ~/.bashrc'
  ```


## 4. Dependencies for building packages

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```


## 5. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```


## 6. Create a ROS Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
