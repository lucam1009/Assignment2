#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "assignment2_msgs/srv/threshold.hpp"
#include "assignment2_msgs/msg/obstacle_position.hpp"
#include <iostream>
#include <limits>
#include <thread>
#include <chrono>
#include <deque>

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
        sub_obstacle = this->create_subscription<assignment2_msgs::msg::ObstaclePosition>("/obstacle", 10, std::bind(&movement::callback_obstacle,this, _1));

        std::thread(&movement::user_input_loop, this).detach();
    }

    double v = 0.0;
    double alpha = 0.0;
    int safety_status = 0;
    float distance, pos_x, pos_y, threshold, lin_average, ang_average;
    std::deque<float> lin_vel;
    std::deque<float> ang_vel;
    geometry_msgs::msg::Twist message;

    private:

    void user_input_loop(){
        int choice;
        while(rclcpp::ok()){
            std::cout << "The closest obstacle is at a distance: " << distance << "\n at position:\n x = " <<pos_x<< "\n y = " <<pos_y<< "\n with threshold: "<<threshold<< std::endl;
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
        lin_vel.push_back(v);
        ang_vel.push_back(alpha);
        float lin_sum = 0.0;
        float ang_sum = 0.0;
        if(lin_vel.size() == 5 || ang_vel.size() == 5){
            for(int i = 0; i < 5; i++){
                lin_sum += lin_vel[i];
                ang_sum += ang_vel[i];
            }
            lin_average = lin_sum/5;
            ang_average = ang_sum/5;
            lin_vel.pop_front();
            ang_vel.pop_front();
        }

        for(int i = 0; i < 20; i++){
            if(safety_status == 1) {
                break; 
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); //aggiungo per far sì che il movimento duri solo per 4 secondi
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

    void callback_obstacle(const assignment2_msgs::msg::ObstaclePosition::SharedPtr obstacle)
    {
        distance = obstacle->distance;
        pos_x = obstacle->pos_x;
        pos_y = obstacle->pos_y;
        threshold = obstacle->threshold;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_safety;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Threshold>::SharedPtr threshold_client;
    rclcpp::Subscription<assignment2_msgs::msg::ObstaclePosition>::SharedPtr sub_obstacle;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<movement>());
    rclcpp::shutdown();
    return 0;
}