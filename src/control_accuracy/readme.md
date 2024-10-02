sudo chmod 777 /dev/ttyACM0

roslaunch mavros px4.launch gcs_url:=udp://:14550@192.168.42.35:14550

roslaunch realsense2_camera rs_t265.launch

rosrun control_accuracy t265_to_uav_pose_transformer.py


rostopic pub -r 30 /mavros/setpoint_position/local geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.2
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" 

