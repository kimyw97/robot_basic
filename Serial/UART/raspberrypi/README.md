<!-- @format -->

라즈베리 파이 기본 세팅 후 node 설치

```
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
```

설치 후 버전 확인

```
nvm -v
```

nvm이 없는 명령어라고 나오면

```
vi .bashrc # 들어가서 맨 밑에 아래 명령어들 있는지 확인 후 없으면 추가
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion" # This loads nvm bash_completion
# 있어도 안된다면 아래 명령어 터미널에서 실행
source .bashrc
```

그 후 node 설치 및 확인

```
node -v #v22.13.1
npm -v #10.9.2
```

프로젝트 init

```
npm init
npm install serialport -save
```

실행

```

```

라즈베리 파이 설정

1. 터미널에서 `sudo raspi-config`

- Interfacing Options → P6 Serial
- "Would you like a login shell to be accessible over serial?" → No
- "Would you like the serial port hardware to be enabled?" → Yes
- 완료 후 Reboot (sudo reboot)

2. `config.txt` 파일 수정

```
sudo vi /boot/firmware/config.txt
```

아래 내용 추가

```
enable_uart=1
dtoverlay=pi3-miniuart-bt
```
