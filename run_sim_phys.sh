v4l2-ctl --set-ctrl=contrast=128
v4l2-ctl --set-ctrl=saturation=0
v4l2-ctl --set-ctrl=brightness=25
v4l2-ctl --set-ctrl=sharpness=200
v4l2-ctl --set-ctrl=exposure_auto=3

roslaunch mover6_gazebo mover6_gazebo_phys.launch