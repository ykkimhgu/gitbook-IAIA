# Install Deep Learning Framework

## 1. Create Environment

```bash
# Update CONDA in Base
conda update -n base -c defaults conda

# Create myEnv=py38
conda create -n py38 python=3.8.10

# Activate myEnv
conda activate py38

# Install Numpy, OpenCV, Matplot, Jupyter
conda install -c anaconda seaborn jupyter
pip install opencv-python
```


## 2. Install CUDA, CUDNN, PyTorch

### 2.1. With GPU

* Check GPU model & CUDA Environment from Website [CUDA Environment Wiki](https://en.wikipedia.org/wiki/CUDA)
* Check GPU Driver (Troubleshooting)

```bash
# Check GPU model & NVIDIA Driver
nvidia-smi

# Install CUDA and cuNN
conda install -c anaconda cudatoolkit=11.8 cudnn 

# Install PyTorch
conda install pytorch=2.1 torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia
pip install torchsummary

# Check Installed Packaged in myENV
conda list all
```


### 2.2. Only CPU

```bash
# CPU Only - PyTorch 2.1
conda install pytorch==2.1.1 torchvision==0.16.1 torchaudio==2.1.1 cpuonly -c pytorch
pip install torchsummary
```


## 3. ROS 패키지 설치 및 환경구축

*   Conda 가상환경에 ROS 패키지 설치

    ```bash
    # Activate myEnv
    conda activate py38

    # install rospkg, catkin_pkg
    conda install rospkg catkin_pkg
    ```
*   환경변수: ROS python 라이브러리 경로 추가

    ```bash
    # 편집기 실행
    gedit ~/.bashrc

    # 환경변수 추가
    export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH
    ```


## 4. Troubleshooting

#### Libtiff 버전 호환 문제

* 상황: 위의 설치과정 이후, `ex_pet_classifier.py` 예제 실행 시, 아래 메세지 출력됨.
* 에러 메시지: `/lib/libgdal.so.26: undefined symbol: TIFFReadRGBATileExt, version LIBTIFF_4.0`
*   conda env에 `libtiff`의 버전이 적절하지 않아 발생하는 문제임.

    ```bash
    # 현재 설치된 libtiff의 버전 확인하기
    conda list libtiff

    # libtiff 4.0으로 설치하기
    conda install -c conda-forge libtiff=4.0
    ```

#### `nvidia-smi` 명령어 없다고 할 때

* NVIDIA 그래픽카드가 없는 노트북이거나 그래픽 드라이버가 설치되지 않은 상황임.
* 그래픽 드라이버 초기화 및 재설치 과정 수행해야함.
* **그래픽 드라이버 재설치 시 recommended가 붙어있는 드라이브 설치 및 재부팅**
* 드라이브 제거 및 재부팅 시 검은화면으로 부팅될 수 있으므로 주의

* 기존 그래픽 드라이버 삭제
  ```bash
  sudo apt-get purge nvidia*
  sudo apt-get autoremove
  sudo apt-get autoclean
  ```

* 설치 가능한 드라이버 버전 확인
  ```bash
  ubuntu-drivers devices
  ```

  ![](https://github.com/user-attachments/assets/1dfd5f25-49b5-48c8-87a8-a43894a7eb77)


* 그래픽 드라이버 설치 및 재부팅
  ```bash
  sudo add-apt-repository ppa:graphics-drivers/ppa
  sudo apt update
  sudo apt-cache search nvidia | grep nvidia-driver
  
  # sudo apt-get install nvidia-driver-[version]
  sudo apt-get install nvidia-driver-535

  sudo reboot
  ```

