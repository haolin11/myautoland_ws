import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft
import tf2_ros

class T265ToUAVPoseTransformer:
    def __init__(self):
        rospy.init_node('t265_to_uav_pose_transformer')
        # self.t265_subscriber = rospy.Subscriber('/camera/odom/sample', Odometry, self.t265_pose_callback)
        self.t265_subscriber = rospy.Subscriber('/Odometry', Odometry, self.t265_pose_callback)
        self.uav_publisher = rospy.Publisher('/uav/center_pose', Odometry, queue_size=10)
        self.mavros_vision_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
    def t265_pose_callback(self, msg):
        # 提取 T265 位姿
        t265_position = msg.pose.pose.position
        t265_orientation = msg.pose.pose.orientation
        
        # 将位置转换为列表
        t265_pos = [t265_position.x, t265_position.y, t265_position.z]
        t265_quat = [t265_orientation.x, t265_orientation.y, t265_orientation.z, t265_orientation.w]

        # 定义偏移量
        # offset = [-0.305, 0, 0]
        offset = [0, 0, 0]

        # 将偏移量转换为 UAV 的坐标系
        # 将偏移量转化为齐次坐标
        offset_homogeneous = tft.quaternion_matrix(t265_quat)[:3, :3].dot(offset)
        
        # 计算 UAV 的位置
        uav_pos = [t265_pos[i] + offset_homogeneous[i] for i in range(3)]

        # 创建 UAV 位姿消息
        uav_pose_msg = Odometry()
        uav_pose_msg.header = msg.header
        uav_pose_msg.pose.pose.position.x = uav_pos[0]
        uav_pose_msg.pose.pose.position.y = uav_pos[1]
        uav_pose_msg.pose.pose.position.z = uav_pos[2]
        uav_pose_msg.pose.pose.orientation = msg.pose.pose.orientation

        # 发布 UAV 位姿
        self.uav_publisher.publish(uav_pose_msg)

        #发布mavros所需位姿话题
        mavros_pose_msg = PoseStamped()
        mavros_pose_msg.header.stamp = rospy.Time.now()
        mavros_pose_msg.header.frame_id = "mavros_uavpos_frame"
        mavros_pose_msg.pose.position.x = uav_pos[0]
        mavros_pose_msg.pose.position.y = uav_pos[1]
        mavros_pose_msg.pose.position.z = uav_pos[2]
        mavros_pose_msg.pose.orientation = msg.pose.pose.orientation

        # 发布消息
        self.mavros_vision_pub.publish(mavros_pose_msg)

        # 广播 tf
        transform = tf2_ros.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = msg.header.frame_id
        transform.child_frame_id = "uav_frame"
        transform.transform.translation.x = uav_pos[0]
        transform.transform.translation.y = uav_pos[1]
        transform.transform.translation.z = uav_pos[2]
        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    T265ToUAVPoseTransformer()
    rospy.spin()