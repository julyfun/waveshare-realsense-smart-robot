## todo

- [x] in docker a fastapi server to send points & open-close
  - port 4060
- [x] transform from robot arm joint to camera (by hand)
  - [x] add to tmux.sh
- [ ] Detect the target object and get position in base_link frame
  - [ ] track the object
  - [ ] kalman filter the position
- [ ] Auto distinguish two realsense

## note

- Connect to robot arm before tmux starts.

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
