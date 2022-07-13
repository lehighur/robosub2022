#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/manual_control.hpp"

typedef std_msgs::msg::Header RHeader;

class Movement {
  public:
    Movement();
    RManualControl create_manual_msg(int x, int y, int z, int r, int buttons);
    bool publish_manual_msg(RManualControl *msg);
  private:
    rclcpp::Publisher<RManualControl> mc_pub;
    //rclcpp::Publisher<RState> state_pub;
};
