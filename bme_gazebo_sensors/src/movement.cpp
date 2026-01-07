#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <limits>

using std::placeholders::_1;

class movement: public rclcpp::Node
{
    public:
    movement(): Node("movement")
    {
        pub_velocity = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&movement::execute, this));
        sub_safety = this->create_subscription<std_msgs::msg::Int32>("/safety_status", 10, std::bind(&movement::callback_safety, this, _1));
    }

    int counter = 0;
    double v = 0.0;
    double alpha = 0.0;
    int safety_status = 0;
    geometry_msgs::msg::Twist message;

    private:

    void execute(){

        if(safety_status == 1){
            counter = 0;
            return;
        }

        if (counter >= 50){
            stop();
                
            get_user_input(); 

            counter = 0; 
        } else {
            counter += 1;
        }
                
        message.linear.x = v;
        message.angular.z = alpha;
        pub_velocity->publish(message);
            
    
        
    }
    
    void stop(){
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        pub_velocity->publish(message);
    }

    void get_user_input() {
        while (true) {
            std::cout << "Insert the linear velocity: ";
            std::cin >> v;
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
            std::cin >> alpha;
            if (!std::cin.fail()) {
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                break; 
            }
            std::cout << "Error: insert a number." << '\n';
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    void callback_safety(const std_msgs::msg::Int32::SharedPtr msg)
    {
        safety_status = msg->data;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_safety;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<movement>());
    rclcpp::shutdown();
    return 0;
}