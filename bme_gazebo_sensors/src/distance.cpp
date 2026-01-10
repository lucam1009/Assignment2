#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <string>
#include <limits>
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "assignment2_msgs/srv/threshold.hpp"
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;
using Threshold = assignment2_msgs::srv::Threshold;


class distance_check: public rclcpp::Node
{
    public:
    distance_check(): Node("distance_check")
    {
        sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&distance_check::callback_scan, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&distance_check::control_loop, this));
        sub_position = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&distance_check::callback_position, this, _1));
        pub_safety = this->create_publisher<std_msgs::msg::Int32>("/safety_status", 10);
        pub_emergency_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        server = this->create_service<Threshold>("generate_threshold", std::bind(&distance_check::handle_service, this, _1, _2));


    }

    float angle, ostacolo_x, ostacolo_y;
    bool emergency_mode = false;
    float min_distance = 100.0;
    float current_threshold = 1.5;
    geometry_msgs::msg::Pose current_pose;
    
    private:
    
    void callback_position(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose = msg->pose.pose;
    }

    void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int n_ranges = msg->ranges.size();
        float prev_distance = 0.0;
        float min_distance_loc = 100.0;
        int valid_points = 0;
        for(int i = 0; i < n_ranges; i++){
            float distance = msg->ranges[i];
            bool check = std::isfinite(distance) && distance > msg->range_min && distance < msg->range_max;
            if(check){
                valid_points++;
                if (distance < min_distance_loc)
                    min_distance_loc = distance;
                float diff = std::abs(distance - prev_distance);
                if(diff > 0.75){
                    float angle = msg->angle_min + (i * msg->angle_increment);
                    float ostacolo_x = distance * std::cos(angle);
                    float ostacolo_y = distance * std::sin(angle);
                }
                prev_distance = distance;
            }else{
                prev_distance = 0.0;
            }
        }
        if(valid_points > 0)
            min_distance = min_distance_loc;
        else
            min_distance = 100.0;
    }


    void control_loop()
    {
        std_msgs::msg::Int32 status;
        geometry_msgs::msg::Twist emergency_cmd;

        if (min_distance < current_threshold){
            if(!emergency_mode){
                RCLCPP_INFO(this->get_logger(), "Obstacle too close, go back to safe position");
                emergency_mode = true;
            }
        }else if(min_distance > (current_threshold+0.5)){
            if(emergency_mode){
                emergency_mode = false;
            }
        }

        if(emergency_mode){
            status.data = 1;

            emergency_cmd.linear.x = -1.0;
            emergency_cmd.angular.z = 0.0;
            pub_emergency_vel->publish(emergency_cmd);

        }else{
            status.data = 0;
        }

        pub_safety->publish(status);
    }

    void handle_service(const std::shared_ptr<Threshold::Request> req, std::shared_ptr<Threshold::Response> res){
        current_threshold = req->threshold;
        res->x = current_threshold;
        RCLCPP_INFO(this->get_logger(), "New threshold: %.2f", current_threshold);       
    }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_position;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_safety;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_emergency_vel;
    rclcpp::TimerBase::SharedPtr threshold_timer_;
    rclcpp::Service<Threshold>::SharedPtr server;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<distance_check>());
    rclcpp::shutdown();
    return 0;
}