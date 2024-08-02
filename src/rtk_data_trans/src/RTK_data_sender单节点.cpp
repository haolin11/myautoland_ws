// RTK_data_sender.cpp
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/GPSRAW.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>

using boost::asio::ip::udp;

class DataSender {
private:
    ros::NodeHandle nh;
    ros::Subscriber gps_raw_sub, gps_fix_sub, alt_sub;
    boost::asio::io_service io_service;
    udp::socket socket;
    udp::endpoint remote_endpoint;

public:
    DataSender() : nh("~"), socket(io_service) {
        std::string ip;
        int port;

        // Get IP and port from parameter server
        nh.param<std::string>("remote_ip", ip, "127.0.0.1");
        nh.param("remote_port", port, 14550);

        remote_endpoint = udp::endpoint(boost::asio::ip::address::from_string(ip), port);
        socket.open(udp::v4());
        std::cout<<"发送地址： "<<ip<<":"<<port<<std::endl;

        gps_raw_sub = nh.subscribe("/mavros/gpsstatus/gpsl/raw", 10, &DataSender::GPSRAWCallback, this);
        gps_fix_sub = nh.subscribe("/mavros/global_position/global", 10, &DataSender::gpsFixCallback, this);
        alt_sub = nh.subscribe("/mavros/global_position/rel_alt", 10, &DataSender::altitudeCallback, this);
    }

    template<typename T>
    void send(const T& msg, const std::string& type_identifier) {
        uint32_t serial_size = ros::serialization::serializationLength(msg);
        boost::shared_array<uint8_t> buffer(new uint8_t[serial_size + type_identifier.size() + 1]);
        memcpy(buffer.get(), type_identifier.c_str(), type_identifier.size());
        buffer[type_identifier.size()] = ' '; // Space separator
        ros::serialization::OStream stream(buffer.get() + type_identifier.size() + 1, serial_size);
        ros::serialization::serialize(stream, msg);
        socket.send_to(boost::asio::buffer(buffer.get(), serial_size + type_identifier.size() + 1), remote_endpoint);
    }

    void GPSRAWCallback(const mavros_msgs::GPSRAW& msg) {
        send(msg, "GPSRAW");
        // std::cout<<"GPSRAW发送成功！"<<std::endl;
    }

    void gpsFixCallback(const sensor_msgs::NavSatFix& msg) {
        send(msg, "NavSatFix");
    }

    void altitudeCallback(const std_msgs::Float64& msg) {
        send(msg, "Float64");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "RTK_data_sender");
    DataSender sender;
    ros::spin();
    return 0;
}
