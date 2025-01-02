# 터미널 환경설정

### 1. bashrc 파일이란 무엇인가?

`bashrc` 파일은 Ubuntu와 같은 Linux 시스템에서 **Bash 셸**을 사용하는 동안 초기화 설정을 저장하는 구성 파일입니다. 이 파일은 사용자가 터미널을 실행할 때 자동으로 실행되어 환경 변수를 설정하거나 사용자 정의 명령어를 초기화합니다.

`bashrc`는 일반적으로 사용자의 홈 디렉토리에 위치하며, 숨김 파일로 저장됩니다:

```
~/.bashrc
```

&#x20;&#x20;

### 2. bashrc의 주요 역할

* **환경 변수 설정**: `PATH`와 같은 환경 변수를 설정하여 터미널에서 명령어 실행 경로를 정의합니다.
* **명령어 별칭(Alias)**: 긴 명령어를 간단한 별칭으로 정의할 수 있습니다.
* **함수 정의**: 자주 사용하는 스크립트나 작업을 함수로 저장할 수 있습니다.
* **시스템 초기화 작업**: 로그인 시 필요한 초기 설정을 자동으로 수행합니다.

&#x20;&#x20;

### 3. bashrc 파일 수정하기

*   **bashrc 파일 열기**: 터미널에서 텍스트 편집기를 사용해 `bashrc` 파일을 엽니다:

    ```bash
    gedit ~/.bashrc
    ```
*   **환경 변수 추가**: 아래와 같이 새로운 환경 변수를 추가할 수 있습니다:

    ```bash
    export MY_VARIABLE="my_value"
    ```
*   **별칭 정의**: 자주 사용하는 명령어를 별칭으로 정의합니다:

    ```bash
    alias ll='ls -alF'
    alias gs='git status'
    ```
*   **함수 추가**: 반복 작업을 함수로 정의할 수 있습니다:

    ```bash
    my_function() {
        echo "이것은 사용자 정의 함수입니다."
    }
    ```
* **파일 저장 후 종료**: `gedit`에서 파일 수정 후 저장 버튼을 클릭하거나 `Ctrl + S`를 눌러 저장하고, 창을 닫습니다.

&#x20;&#x20;

### 4. 변경사항 적용하기

`bashrc` 파일을 수정한 후에는 변경사항을 터미널에 적용해야 합니다. 이를 위해 다음 명령어를 실행합니다:

```bash
source ~/.bashrc
```

이 명령어는 `bashrc` 파일을 다시 읽어 현재 셸에 반영합니다.

&#x20;&#x20;

### 5. source 명령어란?

`source` 명령어는 스크립트를 실행하여 현재 셸 세션에 즉시 적용하는 역할을 합니다. 일반적으로 `bashrc`와 같은 설정 파일의 변경사항을 반영할 때 사용됩니다. 예를 들어:

```bash
source ~/.bashrc  # 새로운 환경 변수를 즉시 적용
```

&#x20;&#x20;

### 6. bashrc 활용 예시

*   **PATH에 새로운 경로 추가**:

    ```bash
    export PATH="$PATH:/home/user/my_programs"
    ```
*   **프롬프트 사용자 정의**: 프롬프트를 사용자 정의하여 터미널 사용성을 향상시킵니다:

    ```bash
    export PS1="\u@\h:\w$ "
    ```
*   **별칭 사용으로 작업 단순화**:

    ```bash
    alias update='sudo apt update && sudo apt upgrade'
    ```

&#x20;&#x20;

### 7. 요약

`bashrc` 파일은 터미널 초기화와 사용자 환경 설정의 핵심 역할을 하며, 환경 변수, 별칭, 사용자 정의 함수 등을 통해 작업을 효율적으로 관리할 수 있습니다. `source` 명령어를 사용하여 수정 사항을 즉시 적용할 수 있으므로, 터미널을 다룰 때 꼭 알아두어야 할 기본 개념입니다.