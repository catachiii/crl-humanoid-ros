#include <memory>
#include <optitrack_msgs/msg/mocap_frame_data.hpp>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;


class Listener : public rclcpp::Node {
   public:
    Listener()
        : Node("listener") {
        subscription_ = this->create_subscription<optitrack_msgs::msg::MocapFrameData>(
            "/optitrack_adaptor/mocap_frame", 10, std::bind(&Listener::topic_callback, this, _1));
    }

   private:
    void topic_callback(const optitrack_msgs::msg::MocapFrameData::SharedPtr msg) const {
        rclcpp::Time cur = now();
        rclcpp::Time mid_exp = msg->camera_mid_exposure_time;
        RCLCPP_INFO(get_logger(), "Latency: %f", (cur - mid_exp).seconds());
    }

    rclcpp::Subscription<optitrack_msgs::msg::MocapFrameData>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;
}