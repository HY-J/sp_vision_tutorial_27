#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include "sp_controller_server/controller_plugin.hpp"

namespace pid_controller {
class PidController : public sp_controller_server::ControllerPlugin {
public:
  PidController() = default;
  ~PidController() override = default;

  void configure(
    const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

private:
  double normalizeAngle(double angle);
  size_t findNearestIndex(const geometry_msgs::msg::PoseStamped & pose);

  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Node::SharedPtr node_;
  nav_msgs::msg::Path global_plan_;
  std::string base_frame_id_;

  // PID参数与状态
  double kp_;
  double ki_;
  double kd_;
  double integral_;
  double prev_error_;
  double max_integral_;

  // 运动约束
  double max_linear_vel_;
  double min_linear_vel_;
  double lookahead_dist_;
  double goal_tolerance_;
};
}
#endif
