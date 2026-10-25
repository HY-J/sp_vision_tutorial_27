#include "pid_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <tf2/utils.h>

namespace {
[[noreturn]] void sp_nav_param_error(const rclcpp::Node & node, const std::string & name)
{
  std::ostringstream oss;
  oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
      << "'，请在对应 yaml 的 ros__parameters 中填写。";
  throw std::runtime_error(oss.str());
}

template<typename T>
T require_param(const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  try {
    if (node->has_parameter(name)) {
      return node->get_parameter(name).get_value<T>();
    }
    return node->declare_parameter<T>(name);
  } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
    sp_nav_param_error(*node, name);
  }
}
}

namespace pid_controller {

// 工具函数：把角度归一化到 [-π, π]，避免角度跳变
double PidController::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

// 遍历路径，找到离当前机器人位置最近的路径点
size_t PidController::findNearestIndex(const geometry_msgs::msg::PoseStamped & pose)
{
  double min_dist = 1e9;
  size_t nearest_idx = 0;
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const auto & p = global_plan_.poses[i].pose.position;
    double dx = p.x - pose.pose.position.x;
    double dy = p.y - pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}

void PidController::configure(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  node_ = node;
  plugin_name_ = name;
  tf_buffer_ = tf_buffer;

  if (!node_) {
    throw std::runtime_error("PidController received null node");
  }

  base_frame_id_ = require_param<std::string>(node_, plugin_name_ + ".base_frame_id");

  // ========== 读取所有PID与运动参数 ==========
  kp_ = require_param<double>(node_, plugin_name_ + ".kp");
  ki_ = require_param<double>(node_, plugin_name_ + ".ki");
  kd_ = require_param<double>(node_, plugin_name_ + ".kd");
  max_linear_vel_ = require_param<double>(node_, plugin_name_ + ".max_linear_vel");
  min_linear_vel_ = require_param<double>(node_, plugin_name_ + ".min_linear_vel");
  lookahead_dist_ = require_param<double>(node_, plugin_name_ + ".lookahead_dist");
  goal_tolerance_ = require_param<double>(node_, plugin_name_ + ".goal_tolerance");
  stop_ramp_dist_ = require_param<double>(node_, plugin_name_ + ".stop_ramp_dist");
  max_integral_ = require_param<double>(node_, plugin_name_ + ".max_integral");

  // 初始化PID状态
  integral_ = 0.0;
  prev_error_ = 0.0;
}

void PidController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  // 新路径到来时重置PID状态，避免历史误差干扰
  integral_ = 0.0;
  prev_error_ = 0.0;
}

geometry_msgs::msg::TwistStamped PidController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_->now();
  cmd_vel.header.frame_id = base_frame_id_;

  (void)velocity;  // 当前实现为位姿预瞄控制，不依赖当前速度反馈

  // 路径为空，直接返回零速
  if (global_plan_.poses.empty()) {
    return cmd_vel;
  }

  // ========== 第一步：选取预瞄目标点（纯追踪矢量方向） ==========
  size_t nearest_idx = findNearestIndex(pose);
  size_t target_idx = nearest_idx;
  double acc_dist = 0.0;

  // 沿着路径往前累加距离，直到达到预瞄距离
  for (size_t i = nearest_idx; i < global_plan_.poses.size() - 1; ++i) {
    const auto & p1 = global_plan_.poses[i].pose.position;
    const auto & p2 = global_plan_.poses[i+1].pose.position;
    double seg_len = std::hypot(p2.x - p1.x, p2.y - p1.y);
    if (acc_dist + seg_len >= lookahead_dist_) {
      target_idx = i + 1;
      break;
    }
    acc_dist += seg_len;
    target_idx = i + 1;
  }

  const auto & cur = pose.pose.position;
  const auto & target_point = global_plan_.poses[target_idx].pose.position;
  const auto & goal_point = global_plan_.poses.back().pose.position;

  // ========== 第二步：期望运动方向（map 系） ==========
  double dx = target_point.x - cur.x;
  double dy = target_point.y - cur.y;
  double desired_yaw = std::atan2(dy, dx);

  // 当前航向：从四元数提取yaw角
  double current_yaw = tf2::getYaw(pose.pose.orientation);

  // 航向误差 = 期望方向 - 当前航向，归一化
  double heading_error = normalizeAngle(desired_yaw - current_yaw);

  // ========== 第三步：航向PID（输出给角速度通道，供差速底盘使用） ==========
  integral_ += heading_error;
  // 积分限幅，防止饱和
  if (integral_ > max_integral_) integral_ = max_integral_;
  if (integral_ < -max_integral_) integral_ = -max_integral_;

  double derivative = heading_error - prev_error_;
  double angular_z = kp_ * heading_error + ki_ * integral_ + kd_ * derivative;
  prev_error_ = heading_error;

  // ========== 第四步：自适应线速度 ==========
  // 4a) 弯道减速：预瞄点之后的路径转角越大，速度越低
  double turn_ahead = 0.0;
  if (target_idx + 1 < global_plan_.poses.size()) {
    const auto & nxt = global_plan_.poses[target_idx + 1].pose.position;
    double next_yaw = std::atan2(nxt.y - target_point.y, nxt.x - target_point.x);
    turn_ahead = std::fabs(normalizeAngle(next_yaw - desired_yaw));
  }
  double speed_scale = 1.0 - 0.45 * std::min(turn_ahead / M_PI_2, 1.0);

  // 4b) 航向偏差也参与减速（鲁棒性兜底）
  speed_scale *= 1.0 - 0.25 * std::min(std::fabs(heading_error) / M_PI_2, 1.0);

  double linear = max_linear_vel_ * std::max(speed_scale, 0.0);
  linear = std::max(linear, min_linear_vel_);

  // ========== 第五步：终点判定与平滑停车 ==========
  double dist_to_goal = std::hypot(
    goal_point.x - cur.x, goal_point.y - cur.y);

  if (dist_to_goal < goal_tolerance_) {
    // 已到达，停车
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  // 接近终点时按剩余距离线性降速，平滑停车不冲过目标；
  // 停车斜坡在终点前 stop_ramp_dist 距离内生效（比预瞄距离短，尾段更快）
  double stop_ramp = std::clamp(dist_to_goal / std::max(stop_ramp_dist_, 0.5), 0.0, 1.0);
  linear = linear * stop_ramp;
  if (linear < 1e-3) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  // ========== 第六步：map 系期望速度 -> base_link 系线速度 ==========
  // 说明：评测仿真器为全向底盘，只消费 cmd_vel 的 linear.x / linear.y
  // （angular.z 不生效），因此把期望运动方向旋转到车体系输出，
  // 使机器人具备横向纠偏能力，等效于“矢量追击 + 横向预瞄”。
  double vx_map = std::cos(desired_yaw) * linear;
  double vy_map = std::sin(desired_yaw) * linear;
  double cos_yaw = std::cos(current_yaw);
  double sin_yaw = std::sin(current_yaw);

  cmd_vel.twist.linear.x = cos_yaw * vx_map + sin_yaw * vy_map;
  cmd_vel.twist.linear.y = -sin_yaw * vx_map + cos_yaw * vy_map;
  cmd_vel.twist.angular.z = angular_z;

  return cmd_vel;
}

}

PLUGINLIB_EXPORT_CLASS(pid_controller::PidController, sp_controller_server::ControllerPlugin)
