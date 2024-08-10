#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class NumberCounterNode : public rclcpp::Node 
{
public:
    NumberCounterNode() : Node("Number_Counter"), counter_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int64>("number_counter", 10);
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>("number", 10, 
                                                    std::bind(&NumberCounterNode::numCount, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Smartphone has been started");
    }
    
private:
    void numCount(const std_msgs::msg::Int64::SharedPtr msg)
    {
        counter_ +=msg->data;
        auto new_msg = std_msgs::msg::Int64();
        new_msg.data = std::int64_t(counter_);
        publisher_->publish(new_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    int counter_;
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
