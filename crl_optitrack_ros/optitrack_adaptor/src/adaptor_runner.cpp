#include "optitrack_ros2_adaptor/NatNetPublisher.h"
#include "optitrack_ros2_adaptor/Dispatcher.h"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto adaptor = std::make_shared<NatNetPublisher>("optitrack_adaptor");
    auto dispatcher = std::make_shared<Dispatcher>("optitrack_dispatcher", "optitrack_adaptor");
    executor.add_node(adaptor);
    executor.add_node(dispatcher);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
