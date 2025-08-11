# espcam_image_ros_publisher
Repository for espcam to publish in microros
use for microros 
```
export ROS_DISTRO=humble \
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v4
```

use for rqt image
```
xhost +SI:localuser:root \
docker run -it --rm   --name ros_rqt   --net=host   --env="DISPLAY"   --env="XAUTHORITY=/root/.Xauthority"   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"   --volume="$HOME/.Xauthority:/root/.Xauthority:ro"   osrf/ros:humble-desktop-full bash
```
