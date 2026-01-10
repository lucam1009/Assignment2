#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "assignment2_msgs/srv/threshold.hpp"
#include <iostream>
#include <limits>
#include <thread>
#include <chrono>

using std::placeholders::_1;
using Threshold = assignment2_msgs::srv::Threshold;

class movement: public rclcpp::Node
{
    public:
    movement(): Node("movement")
    {
        pub_velocity = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&movement::execute, this));
        sub_safety = this->create_subscription<std_msgs::msg::Int32>("/safety_status", 10, std::bind(&movement::callback_safety, this, _1));
        threshold_client = this->create_client<Threshold>("generate_threshold");

        std::thread(&movement::user_input_loop, this).detach();
    }

    double v = 0.0;
    double alpha = 0.0;
    int safety_status = 0;
    geometry_msgs::msg::Twist message;

    private:

    void user_input_loop(){
        int choice;
        while(rclcpp::ok()){
            std::cout << "Insert 1 for change the velocity or 2 for the threshold: ";
            std::cin >> choice;
            if (!std::cin.fail()) {
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                if(choice == 1){
                    get_user_input();
                } else if(choice == 2){
                    float new_threshold;
                    std::cout << "insert the new threshold: ";
                    std::cin >> new_threshold;
                    if(!std::cin.fail()) {
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                        call_service(new_threshold);
                    } else {
                        std::cout << "Invalid threshold input.\n";
                        std::cin.clear();
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    }
                }
            }else {
                std::cout << "Error: insert a number." << '\n';
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
    }

    void call_service(float val){
        if(!threshold_client->wait_for_service(std::chrono::seconds(1))){
            std::cout << "Error, server not responding\n";
            return;
        }

        auto request = std::make_shared<Threshold::Request>();
        request->threshold = val;

        threshold_client->async_send_request(request);
    }

    void execute(){
        if(safety_status == 1){
            return;
        } else {
            message.linear.x = v;
            message.angular.z = alpha;
        }
        pub_velocity->publish(message);
    }
    
    void stop(){
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        pub_velocity->publish(message);
    }

void get_user_input() {
        double temp_v = 0.0;
        double temp_alpha = 0.0;

        while (true) {
            std::cout << "Insert the linear velocity: ";
            std::cin >> temp_v;
            if (!std::cin.fail()) {
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                break; 
            }
            std::cout << "Error: insert a number." << '\n';
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        while (true) {
            std::cout << "Insert the angular velocity: ";
            std::cin >> temp_alpha;
            if (!std::cin.fail()) {
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                break; 
            }
            std::cout << "Error: insert a number." << '\n';
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        v = temp_v;
        alpha = temp_alpha;

        for(int i = 0; i < 20; i++){
            if(safety_status == 1) {
                break; 
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); //aggiungo per far sì che il movimento duri solo per 2 secondi
        }

        v = 0.0;
        alpha = 0.0;
        stop(); 
    }

    void callback_safety(const std_msgs::msg::Int32::SharedPtr msg)
    {
        safety_status = msg->data;
        if(safety_status == 1){
            v = 0.0;
            alpha = 0.0;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_safety;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Threshold>::SharedPtr threshold_client;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<movement>());
    rclcpp::shutdown();
    return 0;
}