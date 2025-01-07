# 터미널 명령어

Ubuntu의 터미널은 강력한 도구로, 명령어를 통해 시스템을 효율적으로 관리하고 제어할 수 있습니다. 아래는 Ubuntu 터미널에서 주로 사용하는 기본 명령어와 그 설명입니다.

### 1. 디렉토리 탐색 명령어

#### **cd (Change Directory)**

디렉토리를 이동하는 명령어로, 특정 폴더로 위치를 변경할 때 사용합니다.

```bash
cd /home/user/Documents  # Documents 디렉토리로 이동
cd ..                    # 상위 디렉토리로 이동
cd ~                     # 홈 디렉토리로 이동
cd /                     # 루트 디렉토리로 이동
```

#### **ls (List)**

현재 디렉토리의 파일 및 폴더를 나열하는 명령어입니다.

```bash
ls                      # 기본 목록 표시
ls -l                   # 자세한 정보 포함
ls -a                   # 숨김 파일 포함
ls -lh                  # 파일 크기를 사람이 읽기 쉬운 형식으로 표시
```

&#x20;

### 2. 파일 및 디렉토리 관리

#### **mkdir (Make Directory)**

새 디렉토리를 생성합니다.

```bash
mkdir new_folder        # new_folder라는 이름의 디렉토리 생성
```

#### **rm (Remove)**

파일 또는 디렉토리를 삭제합니다.

```bash
rm file.txt             # 파일 삭제
rm -r folder            # 폴더와 그 안의 내용 삭제
rm -rf folder           # 강제로 삭제 (주의 필요)
```

#### **cp (Copy)**

파일 또는 디렉토리를 복사합니다.

```bash
cp file1.txt file2.txt  # file1.txt를 file2.txt로 복사
cp -r folder1 folder2   # folder1의 내용을 folder2로 복사
```

#### **mv (Move)**

파일 또는 디렉토리를 이동하거나 이름을 변경합니다.

```bash
mv oldname.txt newname.txt  # 파일 이름 변경
mv file.txt /new/path       # 파일을 다른 경로로 이동
```

&#x20;

### 3. 시스템 관리 명령어

#### **sudo (Super User Do)**

관리자 권한으로 명령어를 실행합니다. 주로 시스템 변경이나 설치 작업에 사용됩니다.

```bash
sudo apt update         # 패키지 목록 업데이트
sudo rm -rf /folder     # 관리자 권한으로 폴더 삭제
```

#### **apt (Advanced Package Tool)**

패키지 관리 도구로 소프트웨어 설치 및 업데이트에 사용됩니다.

```bash
sudo apt update         # 패키지 목록 업데이트
sudo apt upgrade        # 시스템 업그레이드
sudo apt install vim    # vim 편집기 설치
sudo apt remove vim     # vim 편집기 제거
```

#### **chmod (Change Mode)**

파일 또는 디렉토리의 권한을 변경합니다.

```bash
chmod 755 script.sh     # 읽기, 쓰기, 실행 권한 부여
chmod +x script.sh      # 실행 권한 추가
```

&#x20;

### 4. 파일 보기 및 편집

#### **cat (Concatenate)**

파일 내용을 출력합니다.

```bash
cat file.txt            # 파일 내용 보기
```

#### **nano**

터미널 기반 텍스트 편집기입니다.

```bash
nano file.txt           # 파일 열기 및 편집
```

&#x20;

### 5. 시스템 정보 확인

#### **pwd (Print Working Directory)**

현재 작업 디렉토리의 경로를 출력합니다.

```bash
pwd                     # 현재 디렉토리 경로 표시
```

#### **df (Disk Free)**

디스크 공간 사용량을 확인합니다.

```bash
df -h                   # 사람이 읽기 쉬운 형식으로 표시
```

#### **top**

현재 실행 중인 프로세스와 시스템 리소스 사용량을 실시간으로 확인합니다.

```bash
top                     # 프로세스 모니터링
```

&#x20;

### 6. 기타 유용한 명령어

#### **clear**

터미널 화면을 지웁니다.

```bash
clear                   # 화면 정리
```

#### **history**

이전에 실행한 명령어 기록을 확인합니다.

```bash
history                 # 명령어 기록 보기
```

#### **man (Manual)**

명령어의 사용법을 확인할 수 있는 도움말 페이지를 표시합니다.

```bash
man ls                  # ls 명령어의 매뉴얼 보기
```
