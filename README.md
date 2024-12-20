# Architecture
Task Manager와 Robot Planner는 Robot Manager로 묶여 **로봇 내에서 동작함**


## 1. Task Manager
Scene Manager로부터 Task를 위한 로봇의 세부동작을 받고, 그 동작을 순서대로 이행하는지 감시하고 관리

## 2. Robot Planner
- Robot Planner는 Task Manager로부터 명령 받은대로 로봇이 실제 동작하도록 함
- OpenCR과 Raspberry Pi(혹은 Robot Planner가 실행되는 PC) 서로 serial 통신하여 이동하고 모듈제어함
<br/>


# Prerequisites

## 1. System
테스트를 진행했던 로봇 환경은 다음으로 구성되어 있습니다. 
- **Hardware**: Turtlebot3 waffle 
- **Operating System**: Ubuntu 20.04.6 LTS
- **Kernel**: Linux 5.4.0-200-generic
- **ROS version**: noetic
- **Manual SBC Setup for TurtleBot3** 완료 ([Robotis Emanual 하단 'expand' 참고](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup))

## 2. ~/.bashrc
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://{server-ip}:11311
export ROS_HOSTNAME={robot-ip}
export TURTLEBOT3_MODEL=waffle
export LDS_MODEL=LDS-02
export OPENCR_MODEL=waffle_noetic
export OPENCR_PORT=/dev/ttyACM0
export ROBOT_NAME={robot-name}
```
예를 들어, 다음의 값으로 대체해서 설정
| 구분 | 값|
|--|--|
|server-ip | 192.168.0.77|
|robot-ip  | 192.168.0.81|
|robot-name| tb3_3       |
<br/>

# Run
```bash
cd ~/catkin_ws/src/
git clone https://github.com/lnalice/robot-task-manager.git
git clone https://github.com/lnalice/robot-planner.git
mv robot-task-manager task_manager
source ~/.bashrc # 혹은 source ~/catkin_ws/devel/setup.bash

## robot manager 실행
ROS_NAMESPACE=$ROBOT_NAME roslaunch task_manager robot_manager.launch multi_robot_name:=$ROBOT_NAME
```
<br/>

# Communication

## 1. Topic

| 토픽 | 설명 | 메시지 데이터 분류
|--|--|--|
|**/{robot_name}/task_manager/ctrl_module_req** | task_manager로부터 모듈제어 요청받음 | Action |
|**/{robot_name}/task_manager/ctrl_module_res** | task_manager의 모듈제어 요청에 응함 | Action |
|**/{robot_name}/task_manager/move_req** | task_manager로부터 이동 요청받음 | Action |
|**/{robot_name}/task_manager/move_res** | task_manager에 이동 완료 알림 | Action |
|**/{robot_name}/cmd_vel** | OpenCR과 serial 통신(이동 요청) | Twist |
|**/{robot_name}/module_pos** | OpenCR과 seirial 통신 (모듈제어 요청) | Float64MultiArray |


## 2. Message Data
아래의 표는 토픽으로 전달되는 데이터에 대해 설명합니다.

| 분류 | 자료형 | 데이터 구조 | 예시 |
|--|--|--|--|
| Action(MOVE) | String | MOVE `로봇` `seconds` `직진속도`  `각속도` `delay(sec)` | "MOVE tb3_0 10 0.07 -0.01 0" |
| Action(MODULE) | String | MODULE `로봇` `수직 모터 각도차` `수평 모터 각도차` `delay(sec)`| "MODULE tb3_0 1380 650 0"|
| Twist | [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) ||
| Float64MultiArray | [std_msgs/Float64MultiArray](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64MultiArray.html) ||


# File Structure
```
├── CMakeLists.txt
├── README.md
├── launch
│   └── robot_manager.launch
├── package.xml
└── src
    ├── helper
    │   └── getLedStatus.py
    ├── publisher
    │   ├── blinkLedPub.py
    │   ├── cmdVelPub.py
    │   └── ctrlModulePub.py
    └── robot_planner.py
```


<br />

---
<br />