#include <publishToMqtt.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MQTTPublisher : public rclcpp::Node
{
  public:
    MQTTPublisher()
    : Node("mqtt_publisher"), count_(0)
    {
      //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10); // TODO REMOVE
      statusTimer_ = this->create_wall_timer(1000ms, std::bind(&MQTTPublisher::StatusCallback, this));
      worldModelTimer_ = this->create_wall_timer(4000ms, std::bind(&MQTTPublisher::WorldModelCallback, this));
      subscription_ = this->create_subscription<ulisse_msgs::msg::NavFilterData>("/ulisse/nav_filter/data", 10, std::bind(&MQTTPublisher::NavFilterCallback, this, _1));
      mqttPub = std::make_shared<mqttt::MQTTPublisher>("ulisseStatusPub", "catl/unige/ulisse/status",  "127.0.0.1", 1883); // TODO CHECK ARGUMENTS
    }

  private:
    void StatusCallback() {
      TestPubStatus(*mqttPub);
    }
    void WorldModelCallback() {
      TestWorldModel(*mqttPub);
    }
    void NavFilterCallback(const ulisse_msgs::msg::NavFilterData::SharedPtr msg) const {
      //std::cerr << "msg->bodyframe_linear_velocity[0] = " << msg->bodyframe_linear_velocity[0] << std::endl; // TODO REMOVE
    }


    rclcpp::TimerBase::SharedPtr statusTimer_;
    rclcpp::TimerBase::SharedPtr worldModelTimer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MQTTPublisher>());
  rclcpp::shutdown();
  return 0;
}
