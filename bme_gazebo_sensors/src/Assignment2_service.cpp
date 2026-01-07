#include "rclcpp/rclcpp.hpp"
#include "assignment2_msgs/srv/threshold.hpp"

using Threshold = assignment2_msgs::srv::Threshold;
using namespace std::chrono_literals;

class ThresholdServiceNode : public rclcpp::Node
{
public:
    ThresholdServiceNode() : Node("threshold_service_node")
    {
        service = this->create_service<Threshold>(
            "generate_threshold",
            std::bind(&ThresholdServiceNode::handle_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "threshold service ready.");
    }

private:
    void handle_service(
        const std::shared_ptr<Threshold::Request> request,
        std::shared_ptr<Threshold::Response> response)
    {
        response->x = request->threshold;
        RCLCPP_INFO(this->get_logger(), "threshold updated: threshold=%.2f", response->x);       
    }

    rclcpp::Service<Threshold>::SharedPtr service;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThresholdServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}