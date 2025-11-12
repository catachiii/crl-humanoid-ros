#pragma once

#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <NatNetTypes.h>

#include <optitrack_msgs/msg/mocap_frame_data.hpp>
#include <optitrack_msgs/srv/connect.hpp>
#include <optitrack_msgs/srv/disconnect.hpp>
#include <optitrack_msgs/srv/reset.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "CommunicationNames.h"

class NatNetPublisher : public rclcpp::Node {
    static constexpr int discover_time_ms = 500;

   public:
    NatNetPublisher(const std::string& name);
    ~NatNetPublisher();
    bool connect();
    void disconnect();
    bool reset();

    void get_server_desc();

   private:
    static void NATNET_CALLCONV received_callback(sFrameOfMocapData* frame, void* user_data);
    static void NATNET_CALLCONV log_callback(Verbosity level, const char* message);

   private:
    // NatNet Connections
    std::unique_ptr<NatNetClient> client;
    char server_addr_store[kNatNetIpv4AddrStrLenMax];
    char local_addr_store[kNatNetIpv4AddrStrLenMax];
    char multicast_addr_store[kNatNetIpv4AddrStrLenMax];
    sNatNetClientConnectParams connection_params;
    sServerDescription server_desc;

    // ROS2
    rclcpp::Publisher<optitrack_msgs::msg::MocapFrameData>::SharedPtr frame_publisher;
    rclcpp::Service<optitrack_msgs::srv::Connect>::SharedPtr connect_service;
    rclcpp::Service<optitrack_msgs::srv::Disconnect>::SharedPtr disconnect_service;
    rclcpp::Service<optitrack_msgs::srv::Reset>::SharedPtr reset_service;
};
