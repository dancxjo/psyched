#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class PsychedCppNode : public rclcpp::Node
{
public:
    PsychedCppNode()
        : Node("psyched_cpp_node"), counter_(0)
    {
        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("psyched_cpp_topic", 10);
        
        // Create timer
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&PsychedCppNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Psyched C++ node has been started");
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Psyched C++ framework message " + std::to_string(counter_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t counter_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PsychedCppNode>());
    rclcpp::shutdown();
    return 0;
}