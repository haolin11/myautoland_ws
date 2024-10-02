#!/bin/bash

# 终端1的命令
cmd1="cd ; sudo -S chmod 777 /dev/ttyACM0 <<<"nv" ;echo ; roslaunch mavros px4.launch  gcs_url:=udp://:14550@192.168.8.71:14550"

# 终端2的第一个标签页的命令
#cmd2tab1="cd ~/myautoland_ws/; source devel/setup.bash;echo  xxx"
# 终端2的第二个标签页的命令
#cmd2tab2="cd ~/myautoland_ws/; source devel/setup.bash;sleep 3; rosrun control_accuracy t265_to_uav_pose_transformer.py"

# 终端3的命令
cmd3="cd ~/myautoland_ws/; source devel/setup.bash;sleep 3; rosrun control_accuracy t265_to_uav_pose_transformer.py"
#cmd3="cd ~/myautoland_ws/; source devel/setup.bash; roslaunch control_accuracy point_offboard_fly.launch"

# 终端4的命令
cmd4="cd ; rostopic echo /mavros/local_position/pose"

# 打开终端1
gnome-terminal --geometry=60x19+0+0 -- bash -c "$cmd1; exec bash" &

#sleep 4

# 打开终端2，并添加两个标签页
#gnome-terminal --geometry=20x10+900+0 --window  -e "bash -c '$cmd2tab1; exec bash'"  --tab -e "bash -c '$cmd2tab2; exec bash'" &

sleep 4

# 打开终端3
gnome-terminal --geometry=60x19+0+700 -- bash -c "$cmd3; exec bash" &

sleep 2

gnome-terminal --geometry=60x19+900+700 -- bash -c "$cmd4; exec bash" &
# 等待所有终端启动完毕
sleep 2

