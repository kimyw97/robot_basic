<!-- @format -->

# ROS2 Practice

로스 공식 문서 기반으로 예제 학습 및 간단한 프로젝트

https://docs.ros.org/en/humble/index.html

## 개발 환경

macOS에서 UTM 사용해 가상 환경에 Ubuntu 22.04 사용

UTM 공유 폴더 기능을 사용해 우분투에 작업 환경 마운트

```sh
sudo mount -t 9p -0 trans-virtio share /home/wook/Desktop/share -oversion=9p2000.L
```

ros2 실행을 위해 루트와 공유 폴더 연결

```
cd ~/
ln -s /원하는_프로젝트_경로/프로젝트_폴더 . # ~/Desktop/share/ros2_ws
cd ~/ros2_ws
colcon build
```

위처럼하면 macos의 vscode에서 소스코드를 수정하고 UTM의 VM 우분투에서 빌드,실행이 가능함

권한 부족 이슈 => chmod src/ 777
디렉토리에서 폴더를 생성하는 권한이 없어서 발생

## Directory Structure

```
ros2_ws/
├── src/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── <package_name>/
│       ├── src/
│       ├── include/
│       ├── launch/
│       ├── config/
│       ├── scripts/
│       └── CMakeLists.txt
├── install/
├── build/
├── log/
└── README.md
```

## Getting Started

1. Clone the repository:

   ```sh
   git clone <repository_url>
   ```

2. Navigate to the workspace directory:

   ```sh
   cd ros2_ws
   ```

3. Build the workspace:

   ```sh
   colcon build
   ```

4. Source the setup file:

   ```sh
   source install/setup.bash
   ```

5. Run the nodes using a launch file:
   ```sh
   ros2 launch <package_name> <launch_file.launch>
   ```

## ROS2 topic, service개발 순서

1. 패키지 생성

2. 코드 생성(package 디렉토리에 src에서 작업해야함)

3. 의존성 주입

- package.xml

- CMakeLists.txt

관련 패키지 찾기, 실행 명령어와 소스코드 연결, 설치 등의 명령을 넣어줌

4. 의존성 확인

```
rosdep install -i --from-path src --rosdistro humble -y
```

5. 빌드

6. 설치

7. 실행

## ROS2 action 개발 순서

1. .action 생성

```
# Request
---
# Result
---
# Feedback
```

2. 코드 생성(package 디렉토리에 src에서 작업해야함)

3. 의존성 주입

- package.xml

- CMakeLists.txt

관련 패키지 찾기, 실행 명령어와 소스코드 연결, 설치 등의 명령을 넣어줌

4. 의존성 확인

```
rosdep install -i --from-path src --rosdistro humble -y
```

5. 빌드

6. 설치

7. 실행
