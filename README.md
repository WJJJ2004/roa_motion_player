# roa_motion_player

Logitech gamepad 버튼으로 선정된 ROA 팔 모션을 실행하고, 기존
`/hardware_interface/command` 경로로 동일한 명령을 RViz 또는 RobStride
hardware interface에 전달하는 ROS 2 패키지다.

## 모션과 버튼

| 버튼 | evdev code | 모션 |
|---|---:|---|
| A | 304 | 오른팔 인사 |
| X | 307 | 왼팔 인사 |
| Y | 308 | 오른팔 경례 |
| B | 305 | 오른팔 악수 |

버튼은 IDLE 상태에서 rising edge만 받는다. FSM은
`IDLE → INTERPOLATE_TO_MOTION → MOTION_PLAY → INTERPOLATE_TO_IDLE → IDLE`
순서로 동작하며 양쪽 interpolation은 각각 1초다.

악수 중 오른팔 `kp`, `kd`는 기존 arm gain의 50%를 사용한다. 진입 및 복귀
interpolation 동안 position과 gain을 같은 비율로 부드럽게 변경한다.

Recorded trajectory는 인접 sample 사이를 시간 기준으로 선형 보간한다. Command
연속성 검사는 고정된 callback당 변화량 대신 실제 publish 간격과
`max_command_velocity_rad_s`를 사용하므로 timer jitter로 sample이 건너뛰어도
정상 속도 범위의 동작은 중단하지 않는다.

## 빌드

```bash
cd ~/handshake-data
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
colcon build --symlink-install --packages-select roa_motion_player
source install/setup.bash
```

## RViz

```bash
ros2 launch roa_motion_player motion_rviz.launch.py
```

## 실제 하드웨어

RobStride hardware interface는 별도 터미널에서 configure/activate한다.

```bash
# HW interface 활성화 후
ros2 launch roa_motion_player motion_hw.launch.py
```

HW 모드는 첫 idle command를 계속 발행하며 `/walk_initialized=true` 이후에만
IDLE로 들어간다. Motion player와 기존 controller가 동시에
`/hardware_interface/command`를 발행하지 않도록 한다.
