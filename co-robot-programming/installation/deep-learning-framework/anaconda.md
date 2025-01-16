# Anaconda 설치

## 1. Anaconda Installer

* Download anaconda Installer: [link](https://www.anaconda.com/download/success)
  
 ![](https://github.com/user-attachments/assets/1a60754e-35f1-4c9d-b42f-154df6e332ef)


## 2. Execute Installer File (shell script)

```bash
cd ~/Downloads			  # 다운로드 위치로 이동
sudo bash Anaconda3-*.sh	# shell script 실행
```

* 프로그램 설치 약관에 대한 내용 출력시, Enter를 계속해서 누르고 있으면 됨

![](https://github.com/user-attachments/assets/08ea250f-35c0-4526-a71d-eafb2868f1f4)

* 라이센스에 동의하는지 물어보면 "yes" 입력 및 엔터

![](https://github.com/user-attachments/assets/96c3ac4c-af01-478d-9986-4da1b2a34151)

* 설치경로 확인
일반적으로 `home/usr/anaconda3` 경로로 생성됨.
한번씩 root 경로에 설치되는데 그럴 때 `home/usr/anaconda3` 경로로 입력 및 엔터 누르기

![](https://github.com/user-attachments/assets/ed6c2b5d-1617-4dd4-88ca-ef75c2c4f2bd)

[reference link](https://latte-is-horse.tistory.com/2)

* 설치 후 conda 버전 확인
```bash
conda -V
```

## 3. Initialize bashrc

```bash
source ~/.bashrc
```

* 프롬프트를 실행할 때마다 디폴트로 conda 환경이 자동으로 활성화되면서, 터미널에 항상 `(base)`가 함께 출력됨
* 터미널을 열때마다 conda 환경을 활성화되는 것을 막기 위해 아래 명령어 입력

```bash
conda config --set auto_activate_base false
```
