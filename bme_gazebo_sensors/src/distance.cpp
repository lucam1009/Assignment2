#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <string>
#include <limits>
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "assignment2_msgs/srv/threshold.hpp"
#include "assignment2_msgs/srv/average.hpp"
#include <cmath>
#include "assignment2_msgs/msg/obstacle_position.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using Threshold = assignment2_msgs::srv::Threshold;
using Average = assignment2_msgs::srv::Average;
using Obstacle_position = assignment2_msgs::msg::ObstaclePosition;


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
        pub_obstacle = this->create_publisher<assignment2_msgs::msg::ObstaclePosition>("/obstacle", 10);
        server_average = this->create_service<Average>("generate_average", std::bind(&distance_check::handle_average, this, _1, _2));


    }

    float angle, ostacolo_x, ostacolo_y, distance;
    bool emergency_mode = false;
    float min_distance = 100.0;
    float current_threshold = 1.5;
    float current_lin_average = 0.0;
    float current_ang_average = 0.0;
    geometry_msgs::msg::Pose current_pose;
    
    private:
    
    void callback_position(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose = msg->pose.pose;
    }

    void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int n_ranges = msg->ranges.size();
        float min_distance_loc = 100.0;
        int valid_points = 0;
        int index = 0;
        for(int i = 0; i < n_ranges; i++){
            float distance = msg->ranges[i];
            bool check = std::isfinite(distance) && distance > msg->range_min && distance < msg->range_max;
            if(check){
                valid_points++;
                if (distance < min_distance_loc){
                    min_distance_loc = distance;
                    index = i;
                }
            }
        }
        if(valid_points > 0){
            min_distance = min_distance_loc;
            float angle = msg->angle_min + (index * msg->angle_increment);
            float ostacolo_x = min_distance * std::cos(angle);
            float ostacolo_y = min_distance * std::sin(angle);

            assignment2_msgs::msg::ObstaclePosition ob_msg;
            ob_msg.distance = min_distance;
            ob_msg.pos_x = ostacolo_x;
            ob_msg.pos_y = ostacolo_y;
            ob_msg.threshold = current_threshold;
            pub_obstacle->publish(ob_msg);
        }else
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

    void handle_average(const std::shared_ptr<Average::Request> req, std::shared_ptr<Average::Response> res){
        current_lin_average = req->lin_average;
        current_ang_average = req->ang_average;
        res->lin_average = current_lin_average;
        res->ang_average = current_ang_average;
        RCLCPP_INFO(this->get_logger(), "New lin average: %.2f,\n new ang average: %.2f", current_lin_average, current_ang_average); 
    }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_position;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_safety;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_emergency_vel;
    rclcpp::TimerBase::SharedPtr threshold_timer_;
    rclcpp::Service<Threshold>::SharedPtr server;
    rclcpp::Service<Average>::SharedPtr server_average;
    rclcpp::Publisher<assignment2_msgs::msg::ObstaclePosition>::SharedPtr pub_obstacle;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<distance_check>());
    rclcpp::shutdown();
    return 0;
}