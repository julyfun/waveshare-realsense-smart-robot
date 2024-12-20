## todo

- [x] in docker a fastapi server to send points & open-close
  - port 4060
- [x] transform from robot arm joint to camera (by hand)
  - [x] add to tmux.sh
- [x] Detect the target object and get position in base_link frame
  - [x] track the object
  - [x] kalman filter the position
- [ ] Auto distinguish two realsense

## states

- `find`: find obj data
  - [kalman alive] => `position`
  - cache: { kalman }
  - input: {}
- `position`: arm doesn't move, kalman filter obj center position for some time
  - suppose obj doesn't move
  - [1s] => `get-ready`
  - cache: { state_start_time, ObjectTracker }
- `get-ready`: open gripper
  - [0.5s] => `reach`
  - cache: { state_start_time }
- `reach`: move tcp to 3cm away in front of obj
  - [tcp reached with error <= 2cm] => `catch`
  - cache: { target_pos }
- `catch`: close gripper
  - [1.0s] => `to-box`
  - cache: state_start_time
- `to-box`: move tcp above the box
  - [tcp reached with error <= 2cm] => `release`
  - cache: target_pos
- `release`: open gripper
  - [0.5s] => `swing-back`
  - cache: state_start_time
- `init`: move tcp back to start position, using joint angle control
  - [tcp reached with error <= 2cm] => `position`
  - cache: target_q

## Usage note

- MUST connect to robot arm before tmux starts.

## cmd

```
docker run -dit --privileged -h $(hostname) --net=host -v /dev:/dev \
-e DISPLAY=$DISPLAY -e XAUTHORITY=/tmp/xauth -v ~/.Xauthority:/tmp/xauth -v /tmp/.X11-unix:/tmp/.X11-unix \
-v ~/ws/docker2:/home wave-241215:latest
```

```
docker run -dit --privileged -h $(hostname) --net=host -v /dev:/dev -v /run/udev:/run/udev -v /sys/fs/cgroup:/sys/fs/cgroup \
-v /sys/class/tty:/sys/class/tty -v /sys/devices:/sys/devices \
-e DISPLAY=$DISPLAY -e XAUTHORITY=/tmp/xauth -v ~/.Xauthority:/tmp/xauth -v /tmp/.X11-unix:/tmp/.X11-unix \
-v ~/ws/docker2:/home wave-241215:latest
```

## Problems

- no depth frame
  - [ok, 24.12.20] plug out and plug in the realsense
