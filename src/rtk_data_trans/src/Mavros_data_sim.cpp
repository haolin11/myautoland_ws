// Mavros_data_sim.cpp
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/GPSRAW.h>
#include <random>

class SimulatedDataPublisher {
private:
    ros::NodeHandle nh;
    ros::Publisher gps_raw_pub, gps_fix_pub, alt_pub;
    ros::Timer timer;

    std::default_random_engine generator;
    std::normal_distribution<double> lat_dist{48.8566, 0.001};  // Mean latitude of Paris with small variance
    std::normal_distribution<double> lon_dist{2.3522, 0.001};   // Mean longitude of Paris with small variance
    std::uniform_real_distribution<double> alt_dist{30.0, 50.0}; // Altitude between 30m to 50m

public:
    SimulatedDataPublisher() : nh("~"), generator(std::random_device{}()) {
        // Advertise the topics
        gps_raw_pub = nh.advertise<mavros_msgs::GPSRAW>("/mavros/gpsstatus/gpsl/raw", 10);
        gps_fix_pub = nh.advertise<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10);
        alt_pub = nh.advertise<std_msgs::Float64>("/mavros/global_position/rel_alt", 10);

        // Set up a timer to trigger sending
        timer = nh.createTimer(ros::Duration(0.1), &SimulatedDataPublisher::publishData, this);
    }

    void publishData(const ros::TimerEvent&) {
        // Publish GPSRAW
        mavros_msgs::GPSRAW gps_raw_msg;
        gps_raw_msg.fix_type = 3; // Simulate a 3D fix
        gps_raw_pub.publish(gps_raw_msg);

        // Publish NavSatFix with random data
        sensor_msgs::NavSatFix gps_fix_msg;
        gps_fix_msg.latitude = lat_dist(generator);
        gps_fix_msg.longitude = lon_dist(generator);
        gps_fix_msg.altitude = alt_dist(generator);
        gps_fix_pub.publish(gps_fix_msg);

        // Publish Float64 for relative altitude
        std_msgs::Float64 alt_msg;
        alt_msg.data = alt_dist(generator);  // Use the same altitude distribution
        alt_pub.publish(alt_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Mavros_data_sim");
    SimulatedDataPublisher publisher;
    ros::spin();
    return 0;
}
