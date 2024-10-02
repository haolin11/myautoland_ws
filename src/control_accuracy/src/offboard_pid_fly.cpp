#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <yaml-cpp/yaml.h>

mavros_msgs::State current_state;
double position[3] = {0, 0, 0};
double target_position[3] = {0, 0, 0};
double pid_gains[3] = {0.0, 0.0, 0.0}; // Proportional, Integral, Derivative

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    position[0] = msg->pose.position.x;
    position[1] = msg->pose.position.y;
    position[2] = msg->pose.position.z;
}

void load_params(ros::NodeHandle& nh) {
    nh.param<double>("pid/p", pid_gains[0], 1.0);
    nh.param<double>("pid/i", pid_gains[1], 0.0);
    nh.param<double>("pid/d", pid_gains[2], 0.0);
    nh.param<double>("target/x", target_position[0], 0.0);
    nh.param<double>("target/y", target_position[1], 0.0);
    nh.param<double>("target/z", target_position[2], 0.65);
}

geometry_msgs::Twist calculate_velocity() {
    geometry_msgs::Twist vel;
    static double integral[3] = {0.0, 0.0, 0.0};
    static double previous_error[3] = {0.0, 0.0, 0.0};

    for (int i = 0; i < 3; ++i) {
        double error = target_position[i] - position[i];
        integral[i] += error;
        double derivative = error - previous_error[i];
        vel.linear.x = pid_gains[0] * error + pid_gains[1] * integral[i] + pid_gains[2] * derivative;
        previous_error[i] = error;
    }
    return vel;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_velocity_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    load_params(nh_private);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        geometry_msgs::Twist velocity = calculate_velocity();
        velocity_pub.publish(velocity);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
