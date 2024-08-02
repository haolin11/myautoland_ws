// RTK_data_sender.cpp
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/GPSRAW.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <vector>
#include <algorithm>
#include <cctype>
#include <locale>

using boost::asio::ip::udp;

static inline std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
        return !std::isspace(ch);
    }));
    return s;
}

static inline std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
    }).base(), s.end());
    return s;
}

static inline std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}

class DataSender {
private:
    ros::NodeHandle nh;
    ros::Subscriber gps_raw_sub, gps_fix_sub, alt_sub;
    boost::asio::io_service io_service;
    udp::socket socket;
    std::vector<udp::endpoint> remote_endpoints;

public:
    DataSender() : nh("~"), socket(io_service) {
        socket.open(udp::v4());

        // 从参数服务器获取 endpoints 参数
        std::string endpoints;
        nh.getParam("endpoints", endpoints);

        // 解析 IP 和端口
        std::istringstream iss(endpoints);
        std::string segment;
        std::vector<std::string> seglist;

        // // 分割字符串
        // while (std::getline(iss, segment, ',')) {
        //     seglist.push_back(trim(segment));
        // }

        // 分割字符串
        std::string line;
        while (std::getline(iss, line, ';')) { // 以分号为主分割
            std::istringstream lineStream(line);
            
            while (std::getline(lineStream, segment, ',')) { // 以逗号为次分割
                seglist.push_back(trim(segment));
            }
        }

        // 解析IP和端口        
        for (size_t i = 0; i < seglist.size(); i += 2) {
            if (i + 1 < seglist.size()) {
                std::string ip = seglist[i];
                int port = std::stoi(seglist[i + 1]);
                try {
                    remote_endpoints.push_back(udp::endpoint(boost::asio::ip::address::from_string(ip), port));
                    std::cout << "Added endpoint(发送到节点): " << ip << ":" << port << std::endl;
                } catch (const std::exception& e) {
                    std::cout << "Error creating endpoint for " << ip << ":" << port << ". Error: " << e.what() << std::endl;
                }
            }
        }
        

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
        // std::cout<<"数据发送成功1！"<<std::endl;
        for (auto& endpoint : remote_endpoints) {
            socket.send_to(boost::asio::buffer(buffer.get(), serial_size + type_identifier.size() + 1), endpoint);
            // std::cout<<"数据发送成功3！"<<std::endl;
        }
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
    setlocale(LC_ALL, "");  // ROSINFO可显示中文
    ros::init(argc, argv, "RTK_data_sender");
    DataSender sender;
    ros::spin();
    return 0;
}
