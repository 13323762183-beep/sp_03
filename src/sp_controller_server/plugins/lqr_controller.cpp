#include "lqr_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {

// 与 pid_controller.cpp 里同名的 helper 保持一致的可选参数读取策略：
// yaml 没写就用默认值，不因为一个调参项没配就让整条导航链起不来。
template<typename T>
T optional_param(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & def)
{
  try {
    if (!node->has_parameter(name)) {
      return node->declare_parameter<T>(name, def);
    }
    return node->get_parameter(name).get_value<T>();
  } catch (const std::exception & e) {
    RCLCPP_WARN(node->get_logger(), "[LqrController] 参数 '%s' 读取失败(%s)，使用默认值",
      name.c_str(), e.what());
    return def;
  }
}

}

namespace lqr_controller {

void LqrController::configure(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  // 轨迹层参数、TF、marker 发布器全部沿用基类
  PidController::configure(node, name, tf_buffer);

  const std::string p = plugin_name_ + ".";
  tau_ = optional_param<double>(node_, p + "tau", tau_);
  q_pos_ = optional_param<double>(node_, p + "q_pos", q_pos_);
  q_vel_ = optional_param<double>(node_, p + "q_vel", q_vel_);
  r_ctrl_ = optional_param<double>(node_, p + "r_ctrl", r_ctrl_);
  ctrl_dt_ = optional_param<double>(node_, p + "ctrl_dt", ctrl_dt_);
  look_gain_ = optional_param<double>(node_, p + "look_gain", look_gain_);
  cmd_smooth_ = optional_param<double>(node_, p + "cmd_smooth", cmd_smooth_);
  pos_err_clip_ = optional_param<double>(node_, p + "pos_err_clip", pos_err_clip_);

  tau_ = std::clamp(tau_, 0.02, 1.0);
  q_pos_ = std::max(q_pos_, 1e-3);
  q_vel_ = std::max(q_vel_, 1e-3);
  r_ctrl_ = std::max(r_ctrl_, 1e-3);
  ctrl_dt_ = std::clamp(ctrl_dt_, 0.005, 0.1);
  cmd_smooth_ = std::clamp(cmd_smooth_, 0.0, 0.5);

  solveLqr();

  RCLCPP_INFO(node_->get_logger(),
    "[%s] LqrController configured: tau=%.3f Q=diag(%.1f,%.1f,%.1f,%.1f) R=%.2f dt=%.3f "
    "-> K = [%.3f %.3f %.3f %.3f ; %.3f %.3f %.3f %.3f]",
    plugin_name_.c_str(), tau_, q_pos_, q_pos_, q_vel_, q_vel_, r_ctrl_, ctrl_dt_,
    K_(0, 0), K_(0, 1), K_(0, 2), K_(0, 3),
    K_(1, 0), K_(1, 1), K_(1, 2), K_(1, 3));
}

/**
 * 离散 Riccati 迭代（DARE）求 4x4 状态反馈增益。
 *
 * 连续模型（每轴独立）：
 *   d/dt [e_p]   [ 0   1 ] [e_p]   [  0  ]
 *         [e_v] = [ 0 -1/tau] [e_v] + [1/tau] u'
 *
 * 状态顺序取 [ex, ey, evx, evy]，两个轴拼成 4 维，A、B 是分块对角的。
 * 用前向欧拉离散（dt = 控制周期），迭代
 *   K = (R + B'PB)^-1 B'PA
 *   P = A'PA - A'PB K + Q
 * 直到 P 收敛。规模很小，configure 里跑一次即可，运行时没有任何矩阵运算。
 */
void LqrController::solveLqr()
{
  Eigen::Matrix4d Ad = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 4, 2> Bd = Eigen::Matrix<double, 4, 2>::Zero();

  const double decay = 1.0 - ctrl_dt_ / tau_;
  Ad(0, 2) = ctrl_dt_;
  Ad(1, 3) = ctrl_dt_;
  Ad(2, 2) = decay;
  Ad(3, 3) = decay;
  Bd(2, 0) = ctrl_dt_ / tau_;
  Bd(3, 1) = ctrl_dt_ / tau_;

  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q(0, 0) = q_pos_;
  Q(1, 1) = q_pos_;
  Q(2, 2) = q_vel_;
  Q(3, 3) = q_vel_;
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * r_ctrl_;

  P_ = Q;
  K_ = Eigen::Matrix<double, 2, 4>::Zero();
  const Eigen::Matrix4d AdT = Ad.transpose();

  for (int it = 0; it < 1000; ++it) {
    const Eigen::Matrix2d S = R + Bd.transpose() * P_ * Bd;
    const Eigen::Matrix<double, 2, 4> Kd = S.inverse() * Bd.transpose() * P_ * Ad;
    const Eigen::Matrix4d Pn = AdT * P_ * Ad - AdT * P_ * Bd * Kd + Q;
    const double diff = (Pn - P_).cwiseAbs().maxCoeff();
    P_ = Pn;
    K_ = Kd;
    if (diff < 1e-10) {
      break;
    }
  }
}

double LqrController::profileAt(double s) const
{
  if (v_profile_.empty() || s_.empty()) {
    return 0.0;
  }
  if (s <= s_.front()) {
    return v_profile_.front();
  }
  if (s >= s_.back()) {
    return v_profile_.back();
  }
  std::size_t lo = 0;
  std::size_t hi = s_.size() - 1;
  while (hi - lo > 1) {
    const std::size_t mid = (lo + hi) / 2;
    if (s_[mid] <= s) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  const double seg = s_[hi] - s_[lo];
  const double t = seg > 1e-9 ? (s - s_[lo]) / seg : 0.0;
  return v_profile_[lo] + t * (v_profile_[hi] - v_profile_[lo]);
}

geometry_msgs::msg::TwistStamped LqrController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  const rclcpp::Time now = node_->now();
  cmd_vel.header.stamp = now;
  cmd_vel.header.frame_id = base_frame_id_;

  if (plan_dirty_) {
    buildPath();
  }
  if (pts_.size() < 2) {
    return cmd_vel;
  }

  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double yaw = tf2::getYaw(pose.pose.orientation);

  const double dist_end = std::hypot(pts_.back().x - x, pts_.back().y - y);
  if (dist_end < goal_tolerance_) {          // 到位就停住，别在终点画圈
    has_prev_cmd_ = false;
    return cmd_vel;
  }

  double s_now = 0.0;
  double e_lat = 0.0;
  double nx = 0.0;
  double ny = 0.0;
  std::size_t seg = 0;
  projectOntoPath(x, y, s_now, seg, e_lat, nx, ny);

  // 参考速度取**当前位置**的剖面值。速度剖面本身已经过刹车反传，
  // 保证在每一点都停得下来，所以不需要再额外"往前看一段"去提前减速。
  constexpr double kTangentSpan = 0.05;
  double ax = 0.0;
  double ay = 0.0;
  double bx = 0.0;
  double by = 0.0;
  pointAt(std::max(0.0, s_now - kTangentSpan), ax, ay);
  pointAt(std::min(s_.back(), s_now + kTangentSpan), bx, by);
  double tx = bx - ax;
  double ty = by - ay;
  const double tn = std::hypot(tx, ty);
  if (tn > 1e-9) {
    tx /= tn;
    ty /= tn;
  } else {
    tx = 1.0;
    ty = 0.0;
  }

  const double v_ref = profileAt(s_now);
  const Eigen::Vector2d v_d(v_ref * tx, v_ref * ty);

  // 实测速度在 map 系下的表达（Twist 是 base_link 系）
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double vx_map = cy * velocity.linear.x - sy * velocity.linear.y;
  const double vy_map = sy * velocity.linear.x + cy * velocity.linear.y;

  // 位置误差必须相对**投影点**（车在路径上的最近点），不能相对前视点。
  // 若拿前视点当参考，e_p 恒含 -L*tangent 分量，控制量会被额外放大 K*L
  // （K_p=1.36、L=0.45 时约 +0.6 m/s，实测直接冲过弯撞墙）。
  // 前视点只用来提供前馈方向（预瞄），不参与误差计算。
  double pj_x = 0.0;
  double pj_y = 0.0;
  pointAt(s_now, pj_x, pj_y);

  double epx = x - pj_x;
  double epy = y - pj_y;
  const double epn = std::hypot(epx, epy);
  if (epn > pos_err_clip_ && epn > 1e-9) {
    epx *= pos_err_clip_ / epn;
    epy *= pos_err_clip_ / epn;
  }

  Eigen::Vector4d e;
  e << epx, epy, (vx_map - v_d.x()), (vy_map - v_d.y());

  // 前馈 + 最优反馈
  Eigen::Vector2d u = v_d - K_ * e;

  const double spd = u.norm();
  if (spd > max_linear_vel_) {
    u *= max_linear_vel_ / spd;
  }

  // 一阶低通：底盘的速度矢量转向能力约 2.8 rad/s，
  // 指令方向跳变太快时它追不上，会退化成"边转边走"的 S 形。
  if (cmd_smooth_ > 0.0 && has_prev_cmd_) {
    const double a = std::exp(-ctrl_dt_ / cmd_smooth_);
    u = a * prev_cmd_ + (1.0 - a) * u;
  }
  prev_cmd_ = u;
  has_prev_cmd_ = true;

  // ---- 卡住脱困（兜底）----
  // 指令还在、车却不动 = 挤在障碍/墙角上了。沿路径反方向倒一段再重新跟线，
  // 一次任务最多 3 次，防止"蹭墙→倒车→再蹭"的死循环。
  if (stuck_state_ == 0 && u.norm() > 0.5 &&
    std::hypot(velocity.linear.x, velocity.linear.y) < 0.08)
  {
    if (stuck_since_.nanoseconds() == 0) {
      stuck_since_ = now;
    } else if ((now - stuck_since_).seconds() > 2.0 && escape_count_ < 3) {
      stuck_state_ = 2;
      escape_start_ = now;
      ++escape_count_;
      RCLCPP_WARN(node_->get_logger(), "[%s] 检测到卡住，倒车脱困（第 %d 次）",
        plugin_name_.c_str(), escape_count_);
    }
  } else if (stuck_state_ == 0) {
    stuck_since_ = rclcpp::Time(0);
  }

  if (stuck_state_ == 2) {
    if ((now - escape_start_).seconds() > 0.9) {
      stuck_state_ = 0;
      stuck_since_ = rclcpp::Time(0);
      has_prev_cmd_ = false;
    } else {
      const double bx = -tx * 0.30;
      const double by = -ty * 0.30;
      cmd_vel.twist.linear.x = cy * bx + sy * by;
      cmd_vel.twist.linear.y = -sy * bx + cy * by;
      publishMarkers(pose, cmd_vel, yaw);
      return cmd_vel;
    }
  }

  cmd_vel.twist.linear.x = cy * u.x() + sy * u.y();
  cmd_vel.twist.linear.y = -sy * u.x() + cy * u.y();

  publishMarkers(pose, cmd_vel, yaw);
  return cmd_vel;
}

}

PLUGINLIB_EXPORT_CLASS(lqr_controller::LqrController, sp_controller_server::ControllerPlugin)
