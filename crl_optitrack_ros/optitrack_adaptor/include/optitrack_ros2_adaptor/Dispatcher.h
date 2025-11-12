#pragma once

#include <optitrack_msgs/msg/mocap_frame_data.hpp>
#include <optitrack_msgs/msg/rigidbody_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <map>

#include "CommunicationNames.h"

struct RigidbodyPublisher {
    std::string publisher_name;
    rclcpp::Publisher<optitrack_msgs::msg::RigidbodyData>::SharedPtr publisher;

    RigidbodyPublisher(const std::string& publisher_name, const rclcpp::Publisher<optitrack_msgs::msg::RigidbodyData>::SharedPtr& publisher)
        : publisher_name(publisher_name), publisher(publisher) {}
};

class Dispatcher : public rclcpp::Node {
   public:
    Dispatcher(const std::string& name, const std::string& publisher_name);

   private:
    void dispatch(const optitrack_msgs::msg::MocapFrameData::SharedPtr msg) const;

   private:
    rclcpp::Subscription<optitrack_msgs::msg::MocapFrameData>::SharedPtr subscriber;
    std::map<int, RigidbodyPublisher> rigidbody_publishers;
};