# 유틸 프로그램 설치

## 1. terminator 설치

ubuntu의 terminal을 더 편리하게 사용할 수 있는 프로그램이다.

* Install

```bash
sudo apt update
sudo apt install terminator
```

* Preferences
  * Terminator 실행 후, 우클릭하여 preferences 진입
  *   `Keybindings` 탭에서 단축키 설정을 할 수 있다.


  <img src="https://github.com/user-attachments/assets/3a31f6fd-2a37-4e5c-8107-043629f09187" alt="Image" width="300">

&#x20;

## 2. Visual Studio Code 설치

프로그래밍 개발도구로서, 폴더와 파일 목록을 손쉽게 볼 수 있어, ros 입문자에게 매우 편리하다.

*   설치파일 다운로드 `.deb file` - [vs code](https://code.visualstudio.com/download)

    ![](https://github.com/user-attachments/assets/f2718160-304f-44f9-81a7-6c2d5b818829)

*   unpack

    다운로드한 파일은 `home/Downloads` 경로에 존재한다.

```bash
cd ~/Downloads
sudo dpkg -i code_*.deb
```

&#x20;

## 3. github desktop

* 설치파일 다운로드: [Install GithubDesktop](https://github.com/shiftkey/desktop/releases/)
* 설치파일명: GitHubDesktop-linux-amd64-3.4.8-linux1.deb (2024.12.26 기준)
* unpack

```bash
cd ~/Downloads
sudo dpkg -i GitHubDesktop_*.deb
```

&#x20;

## 4. Typora

```bash
wget -qO - https://typora.io/linux/public-key.asc | sudo tee /etc/apt/trusted.gpg.d/typora.asc

# add Typora's repository
sudo add-apt-repository 'deb https://typora.io/linux ./'
sudo apt-get update

# install typora
sudo apt-get install typora
```

&#x20;

## 5. 한/영 키 설정

*   \[settings] - \[Region & Language] - \[Manage Installed Languages] - \[install]

    ![](https://user-images.githubusercontent.com/91526930/234136304-3fa90717-9034-4cff-8337-733da8ebf548.png)

    ![](https://user-images.githubusercontent.com/91526930/234136309-d0f575df-d9b0-4e17-8ed6-a4804dac79a2.png)
* 재부팅

```bash
$ reboot
```

* ibus-setup

```bash
$ ibus-setup
```

* \[Input Method] -\[Add] - \[Korean 검색] - \[Hangul] - \[Add] - \[Close]

![](https://user-images.githubusercontent.com/91526930/234136642-6b78a726-7843-493d-958a-b7caf5b5b151.png)

![](https://user-images.githubusercontent.com/91526930/234136663-7fac9277-4909-414a-8281-4367976b06e5.png)

* \[settings] - \[Region & Language] - \[Add an Input Source] - \[Korean(Hangul) 선택]

![](https://user-images.githubusercontent.com/91526930/234136729-9456e9ce-d9e6-47fc-9b97-b9da291d2f43.png)

![](https://user-images.githubusercontent.com/91526930/234136739-a2e620f6-cd35-4baf-b9b2-d534fd30d41a.png)

![](https://github.com/user-attachments/assets/2d99737e-58dc-4cd4-be41-28dc409a7920)
