// RTK_data_receiver.cpp
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/GPSRAW.h>
#include <boost/asio.hpp>
#include <vector>
#include <sstream>
#include <locale.h>

using boost::asio::ip::udp;

class DataReceiver {
private:
    ros::NodeHandle nh;
    ros::Publisher gps_raw_pub, gps_fix_pub, alt_pub;
    boost::asio::io_service io_service;
    udp::socket socket;
    udp::endpoint remote_endpoint;
    std::vector<uint8_t> recv_buf;  // Move recv_buf to member variable to ensure its lifetime

public:
    DataReceiver() : nh("~"), socket(io_service, udp::endpoint(udp::v4(), get_port())) {
        gps_raw_pub = nh.advertise<mavros_msgs::GPSRAW>("/new/gpsraw", 10);
        gps_fix_pub = nh.advertise<sensor_msgs::NavSatFix>("/new/gpsfix", 10);
        alt_pub = nh.advertise<std_msgs::Float64>("/new/rel_alt", 10);

        recv_buf.resize(2048);  // Resize buffer to accommodate incoming data
        startReceive();
    }

    int get_port() {
        int port;
        nh.param("remote_port", port, 14550);
        ROS_INFO("Listening on port(监听端口): %d", port);
        return port;
    }

    void startReceive() {
        socket.async_receive_from(
            boost::asio::buffer(recv_buf), remote_endpoint,
            [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
                if (!error && bytes_transferred > 0) {
                    handleReceive(bytes_transferred);
                }
                startReceive(); // Continue to receive more data
            }
        );
    }

    void handleReceive(std::size_t bytes_transferred) {
        std::string type_identifier(recv_buf.begin(), std::find(recv_buf.begin(), recv_buf.begin() + bytes_transferred, ' '));
        std::size_t payload_start = type_identifier.size() + 1; // Account for space separator
        ros::serialization::IStream stream(&recv_buf[payload_start], bytes_transferred - payload_start);

        if (type_identifier == "GPSRAW") {
            mavros_msgs::GPSRAW msg;
            ros::serialization::deserialize(stream, msg);
            gps_raw_pub.publish(msg);
        } else if (type_identifier == "NavSatFix") {
            sensor_msgs::NavSatFix msg;
            ros::serialization::deserialize(stream, msg);
            gps_fix_pub.publish(msg);
        } else if (type_identifier == "Float64") {
            std_msgs::Float64 msg;
            ros::serialization::deserialize(stream, msg);
            alt_pub.publish(msg);
        }
    }

    void run() {
        io_service.run();
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");  // ROSINFO可显示中文

    ros::init(argc, argv, "RTK_data_receiver");
    DataReceiver receiver;
    receiver.run();  // Start processing network events
    return 0;
}
