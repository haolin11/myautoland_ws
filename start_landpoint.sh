#!/bin/bash

cmd1="cd ~/myautoland_ws/; source devel/setup.bash; roslaunch control_accuracy point_offboard_fly.launch"


# 打开终端1
gnome-terminal --geometry=60x19+900+0 -- bash -c "$cmd1; exec bash" &


