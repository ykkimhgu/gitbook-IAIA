# 우분투-윈도우 시간 동기화 설정

* 듀얼부팅시 윈도우 시간 변경되는 문제발생
* 기본적으로 윈도우는 로컬 타임을 저장하고, 리눅스는 UTC 시간을 기준으로 저장함
* 해결을 위해 ubuntu 터미널에서 로컬 타임 설정

```bash
timedatectl set-local-rtc 1 --adjust-system-clock
```
