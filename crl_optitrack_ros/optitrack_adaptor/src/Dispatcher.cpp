#include "optitrack_ros2_adaptor/Dispatcher.h"

#include <functional>
#include <sstream>
#include <vector>
#include <string>

using std::placeholders::_1;

Dispatcher::Dispatcher(const std::string& name, const std::string& publisher_name)
    : Node(name, rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)) {
    std::stringstream subscriber_name_builder;
    subscriber_name_builder << publisher_name << "/" << Names::NatNetPublisher::mocap_msg_name;
    subscriber = create_subscription<optitrack_msgs::msg::MocapFrameData>(subscriber_name_builder.str(), 10,
                                                                            std::bind(&Dispatcher::dispatch, this, _1));
    std::string cur_node_prefix = name + "/";
    
    // Rigidbodies
    std::vector<std::string> rigidbody_prefixes;
    rigidbody_prefixes.emplace_back(Names::Dispatcher::rigidbody_prefix);
    const auto rigidbody_trackers = list_parameters(rigidbody_prefixes, 0);
    std::string cur_node_rigidbody_prefix = cur_node_prefix + Names::Dispatcher::rigidbody_prefix + "/";
    for(const auto& rigidbody_tracker : rigidbody_trackers.prefixes){
        std::string id_str = rigidbody_tracker.substr(rigidbody_tracker.rfind(".") + 1);
        int id = std::stoi(id_str);
        std::string publisher_name = get_parameter(rigidbody_tracker + ".publisher_name").as_string();
        rigidbody_publishers.insert({id, RigidbodyPublisher(publisher_name, create_publisher<optitrack_msgs::msg::RigidbodyData>(cur_node_rigidbody_prefix + publisher_name, 10))});
        RCLCPP_INFO(get_logger(), "Created rigidbody publisher for ID %d under name %s", id, publisher_name);
    }   
}


void Dispatcher::dispatch(const optitrack_msgs::msg::MocapFrameData::SharedPtr msg) const{
    // Rigidbodies
    for(const auto& rigidbody : msg->rigidbodies){
        auto cur_rigidbody_publisher = rigidbody_publishers.find(rigidbody.id);
        if(cur_rigidbody_publisher != rigidbody_publishers.end()){
            cur_rigidbody_publisher->second.publisher->publish(rigidbody);
        }else{
            RCLCPP_WARN(get_logger(), "No rigidbody publisher found for ID %d", rigidbody.id);
        }
    }
}
