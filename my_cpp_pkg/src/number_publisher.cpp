#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
    
class NumberPublisherNode : public rclcpp::Node 
{
public:
    // NumberPublisherNode() : Node("number_counter"), number_(2) 
    NumberPublisherNode() : Node("number_counter") 
    {
        this->declare_parameter("number_to_publish", 2);
        this->declare_parameter("publish_timePeriod", 1.0);
        number_ = this->get_parameter("number_to_publish").as_int();
        double pub_period = this->get_parameter("publish_timePeriod").as_double();

        publisher_ = this->create_publisher<std_msgs::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int) (pub_period*1000.0)),
                                    std::bind(&NumberPublisherNode::publishNum, this));
        RCLCPP_INFO(this->get_logger(), "Publisher has been started");
    }
    
private:
    void publishNum()
    {
        auto msg = std_msgs::msg::Int64();
        msg.data = std::int64_t(number_);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int number_;
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
