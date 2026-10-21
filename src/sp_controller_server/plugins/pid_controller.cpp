#include "pid_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

// 可选参数：yaml 没写也不会让整个节点崩掉，直接用默认值
template<typename T>
T optional_param(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & def)
{
  try {
    if (!node->has_parameter(name)) {
      return node->declare_parameter<T>(name, def);
    }
    return node->get_parameter(name).get_value<T>();
  } catch (const std::exception & e) {
    RCLCPP_WARN(node->get_logger(), "[PidController] 参数 '%s' 读取失败(%s)，使用默认值",
      name.c_str(), e.what());
    return def;
  }
}

double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

}

namespace pid_controller {

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

  const std::string p = plugin_name_ + ".";
  max_linear_vel_ = optional_param<double>(node_, p + "max_linear_vel", max_linear_vel_);
  lat_acc_ = optional_param<double>(node_, p + "lat_acc", lat_acc_);
  dec_acc_ = optional_param<double>(node_, p + "dec_acc", dec_acc_);
  lookahead_min_ = optional_param<double>(node_, p + "lookahead_min", lookahead_min_);
  lookahead_max_ = optional_param<double>(node_, p + "lookahead_max", lookahead_max_);
  lookahead_gain_ = optional_param<double>(node_, p + "lookahead_gain", lookahead_gain_);
  goal_tolerance_ = optional_param<double>(node_, p + "goal_tolerance", goal_tolerance_);
  min_speed_ = optional_param<double>(node_, p + "min_speed", min_speed_);
  smooth_passes_ = optional_param<int>(node_, p + "smooth_passes", smooth_passes_);
  fine_traj_ = optional_param<bool>(node_, p + "fine_traj", fine_traj_);
  ref_dev_tol_ = optional_param<double>(node_, p + "ref_dev_tol", ref_dev_tol_);
  lat_gain_ = optional_param<double>(node_, p + "lat_gain", lat_gain_);
  lat_damp_ = optional_param<double>(node_, p + "lat_damp", lat_damp_);
  predict_dt_ = optional_param<double>(node_, p + "predict_dt", predict_dt_);
  turn_rate_max_ = optional_param<double>(node_, p + "turn_rate_max", turn_rate_max_);
  dir_adapt_gain_ = optional_param<double>(node_, p + "dir_adapt_gain", dir_adapt_gain_);
  publish_marker_ = optional_param<bool>(node_, p + "publish_marker", publish_marker_);

  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/controller_marker", rclcpp::QoS(1));

  if (max_linear_vel_ <= 0.0) {
    max_linear_vel_ = 1.35;
    RCLCPP_WARN(node_->get_logger(), "[%s] max_linear_vel 非法，重置为 1.35", plugin_name_.c_str());
  }
  lookahead_min_ = std::max(0.05, lookahead_min_);
  lookahead_max_ = std::max(lookahead_min_, lookahead_max_);
  lat_acc_ = std::max(0.1, lat_acc_);
  dec_acc_ = std::max(0.1, dec_acc_);
  smooth_passes_ = std::clamp(smooth_passes_, 0, 8);
  ref_dev_tol_ = std::max(0.1, ref_dev_tol_);

  RCLCPP_INFO(node_->get_logger(),
    "[%s] PidController configured: v_max=%.2f lat_acc=%.2f dec=%.2f look=[%.2f,%.2f]+%.2f*v "
    "tol=%.2f smooth=%d fine=%d ref_dev=%.2f lat=%.2f/%.2f pred=%.2f turn=%.2f adapt=%.2f "
    "base_frame=%s",
    plugin_name_.c_str(), max_linear_vel_, lat_acc_, dec_acc_,
    lookahead_min_, lookahead_max_, lookahead_gain_,
    goal_tolerance_, smooth_passes_, fine_traj_ ? 1 : 0, ref_dev_tol_, lat_gain_, lat_damp_, predict_dt_,
    turn_rate_max_,
    dir_adapt_gain_,
    base_frame_id_.c_str());
}

void PidController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  std::vector<Point2d> cand;
  cand.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    cand.push_back({ps.pose.position.x, ps.pose.position.y});
  }
  if (cand.size() < 2) {
    return;
  }

  // 参考路径保持稳定：上层每 100ms 从"当前栅格中心"重规划一次，若每次都采信，
  // 前视点会在两格之间来回跳（±5cm -> 约 ±10° 方向噪声），喂给转向有滞后的底盘
  // 就会画龙。只有"真的换了路线"（偏离超过 ref_dev_tol）才更新参考。
  if (!has_ref_ || planDeviates(cand)) {
    const bool rebuild = has_ref_;
    raw_pts_ = cand;
    plan_dirty_ = true;
    has_ref_ = true;
    prog_idx_ = 0;
    if (rebuild) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[%s] 参考路径更新（新规划偏离超过 %.2f m）", plugin_name_.c_str(), ref_dev_tol_);
    }
  }
}

double PidController::distToRef(const Point2d & p) const
{
  if (pts_.size() < 2) {
    return 1e9;
  }
  double best = 1e9;
  for (std::size_t i = 0; i + 1 < pts_.size(); ++i) {
    const double ax = pts_[i].x;
    const double ay = pts_[i].y;
    const double bx = pts_[i + 1].x - ax;
    const double by = pts_[i + 1].y - ay;
    const double l2 = bx * bx + by * by;
    double t = 0.0;
    if (l2 > 1e-12) {
      t = std::clamp(((p.x - ax) * bx + (p.y - ay) * by) / l2, 0.0, 1.0);
    }
    const double dx = p.x - (ax + t * bx);
    const double dy = p.y - (ay + t * by);
    best = std::min(best, dx * dx + dy * dy);
  }
  return std::sqrt(best);
}

bool PidController::planDeviates(const std::vector<Point2d> & cand) const
{
  if (pts_.size() < 2) {
    return true;
  }
  // 终点明显变了（换了目标点）→ 必须换参考
  if (std::hypot(cand.back().x - pts_.back().x, cand.back().y - pts_.back().y) > 0.50) {
    return true;
  }
  // 只看前 6 m：这一段决定接下来怎么走
  double acc = 0.0;
  for (std::size_t i = 0; i < cand.size(); ++i) {
    if (i > 0) {
      acc += std::hypot(cand[i].x - cand[i - 1].x, cand[i].y - cand[i - 1].y);
    }
    if (acc > 6.0) {
      break;
    }
    if (distToRef(cand[i]) > ref_dev_tol_) {
      return true;
    }
  }
  return false;
}

void PidController::buildPath()
{
  pts_.clear();
  s_.clear();
  v_profile_.clear();
  plan_dirty_ = false;

  if (raw_pts_.empty()) {
    return;
  }

  // 去掉重复点
  pts_.push_back(raw_pts_.front());
  for (std::size_t i = 1; i < raw_pts_.size(); ++i) {
    const double dx = raw_pts_[i].x - pts_.back().x;
    const double dy = raw_pts_[i].y - pts_.back().y;
    if (std::hypot(dx, dy) > 1e-3) {
      pts_.push_back(raw_pts_[i]);
    }
  }
  if (pts_.size() < 2) {
    return;
  }

  // ---- 轨迹平滑 ----
  // 两个模式由参数 fine_traj 切换：
  //   稳健模式（默认 false，基线的三轮实测配置）：4 点均值平滑 + 原始点 + 夹角法曲率
  //   精细模式（加分项 true）：Chaikin 细分 + 等弧长重采样 + 三点外接圆曲率
  // 精细模式的曲率更连续、估计更准，但圆角会把直角弯的内角"啃掉"一块，
  // 配合放开的过弯限速（lat_acc 2.4）时两轮里有一轮在急弯撞墙，所以不作默认。
  if (fine_traj_) {
    for (int it = 0; it < smooth_passes_ && pts_.size() >= 3; ++it) {
      std::vector<Point2d> next;
      next.reserve(pts_.size() * 2);
      next.push_back(pts_.front());                       // 起点钉死，保证起点精度
      for (std::size_t i = 0; i + 1 < pts_.size(); ++i) {
        const Point2d & a = pts_[i];
        const Point2d & b = pts_[i + 1];
        next.push_back({0.75 * a.x + 0.25 * b.x, 0.75 * a.y + 0.25 * b.y});
        next.push_back({0.25 * a.x + 0.75 * b.x, 0.25 * a.y + 0.75 * b.y});
      }
      next.push_back(pts_.back());                        // 终点钉死
      pts_.swap(next);
    }
  } else {
    for (int it = 0; it < smooth_passes_ && pts_.size() >= 3; ++it) {
      std::vector<Point2d> sm = pts_;
      for (std::size_t i = 1; i + 1 < pts_.size(); ++i) {
        sm[i].x = 0.25 * (pts_[i - 1].x + 2.0 * pts_[i].x + pts_[i + 1].x);
        sm[i].y = 0.25 * (pts_[i - 1].y + 2.0 * pts_[i].y + pts_[i + 1].y);
      }
      pts_.swap(sm);
    }
  }

  // ---- 等弧长重采样 ----
  // 原来直接对折线上的原始点算曲率，点距从 0.05 m 到 0.4 m 不等，
  // 同样的转弯在不同点距下算出的曲率差好几倍，限速就会忽高忽低。
  // 重采样到固定 0.05 m 后，曲率与速度剖面的分辨率才一致。
  if (fine_traj_) {
    constexpr double kResample = 0.05;
    std::vector<double> arc(pts_.size(), 0.0);
    for (std::size_t i = 1; i < pts_.size(); ++i) {
      arc[i] = arc[i - 1] + std::hypot(pts_[i].x - pts_[i - 1].x, pts_[i].y - pts_[i - 1].y);
    }
    const double total = arc.back();
    if (total > 4.0 * kResample) {
      std::vector<Point2d> res;
      res.reserve(static_cast<std::size_t>(total / kResample) + 2);
      std::size_t k = 0;
      for (double tgt = 0.0; tgt < total; tgt += kResample) {
        while (k + 2 < pts_.size() && arc[k + 1] < tgt) {
          ++k;
        }
        const double seg = arc[k + 1] - arc[k];
        const double t = seg > 1e-9 ? (tgt - arc[k]) / seg : 0.0;
        res.push_back({
          pts_[k].x + t * (pts_[k + 1].x - pts_[k].x),
          pts_[k].y + t * (pts_[k + 1].y - pts_[k].y)});
      }
      res.push_back(pts_.back());
      pts_.swap(res);
    }
  }

  const std::size_t n = pts_.size();
  s_.resize(n, 0.0);
  for (std::size_t i = 1; i < n; ++i) {
    s_[i] = s_[i - 1] + std::hypot(pts_[i].x - pts_[i - 1].x, pts_[i].y - pts_[i - 1].y);
  }

  // ---- 曲率 -> 速度上限 ----
  std::vector<double> v_lim(n, max_linear_vel_);
  for (std::size_t i = 1; i + 1 < n; ++i) {
    if (!fine_traj_) {
      // 稳健模式：前后段夹角 / 平均段长
      const double ax = pts_[i].x - pts_[i - 1].x;
      const double ay = pts_[i].y - pts_[i - 1].y;
      const double bx = pts_[i + 1].x - pts_[i].x;
      const double by = pts_[i + 1].y - pts_[i].y;
      const double la = std::hypot(ax, ay);
      const double lb = std::hypot(bx, by);
      if (la < 1e-6 || lb < 1e-6) {
        continue;
      }
      const double cross = (ax * by - ay * bx) / (la * lb);
      const double ang = std::asin(std::clamp(cross, -1.0, 1.0));
      const double kappa = std::fabs(ang) / (0.5 * (la + lb));
      v_lim[i] = std::min(max_linear_vel_, std::sqrt(lat_acc_ / std::max(kappa, 1e-6)));
      continue;
    }
    // 精细模式：三点外接圆曲率 kappa = 2*|cross| / (a*b*c)
    const double ax = pts_[i].x - pts_[i - 1].x;
    const double ay = pts_[i].y - pts_[i - 1].y;
    const double bx = pts_[i + 1].x - pts_[i].x;
    const double by = pts_[i + 1].y - pts_[i].y;
    const double cx = pts_[i + 1].x - pts_[i - 1].x;
    const double cy = pts_[i + 1].y - pts_[i - 1].y;
    const double la = std::hypot(ax, ay);
    const double lb = std::hypot(bx, by);
    const double lc = std::hypot(cx, cy);
    if (la < 1e-6 || lb < 1e-6 || lc < 1e-6) {
      continue;
    }
    const double cross = std::fabs(ax * by - ay * bx);
    const double kappa = 2.0 * cross / (la * lb * lc);
    v_lim[i] = std::min(max_linear_vel_, std::sqrt(lat_acc_ / std::max(kappa, 1e-6)));
  }

  // 反向传播刹车约束：v[i]^2 <= v[i+1]^2 + 2*a*ds
  v_profile_ = v_lim;
  v_profile_.back() = 0.0;
  for (std::size_t i = n - 1; i-- > 0; ) {
    const double ds = s_[i + 1] - s_[i];
    const double allowed = std::sqrt(v_profile_[i + 1] * v_profile_[i + 1] + 2.0 * dec_acc_ * ds);
    v_profile_[i] = std::min(v_profile_[i], allowed);
  }
}

void PidController::projectOntoPath(
  double x, double y, double & s_out, std::size_t & seg_out, double & e_out,
  double & nx_out, double & ny_out)
{
  e_out = 0.0;
  nx_out = 0.0;
  ny_out = 0.0;
  const std::size_t n = pts_.size();
  if (n < 2) {
    s_out = 0.0;
    seg_out = 0;
    return;
  }
  const std::size_t nseg = n - 1;

  auto seg_cost = [&](std::size_t i) {
      const double ax = pts_[i].x;
      const double ay = pts_[i].y;
      const double bx = pts_[i + 1].x - ax;
      const double by = pts_[i + 1].y - ay;
      const double l2 = bx * bx + by * by;
      double t = 0.0;
      if (l2 > 1e-12) {
        t = std::clamp(((x - ax) * bx + (y - ay) * by) / l2, 0.0, 1.0);
      }
      const double dx = x - (ax + t * bx);
      const double dy = y - (ay + t * by);
      return dx * dx + dy * dy;
    };

  // 只在进度附近搜（前进 6m / 后退 1m），既快又不会跳到路径的另一段上
  const std::size_t back_i = (prog_idx_ > 20) ? prog_idx_ - 20 : 0;
  const std::size_t fwd_i = std::min(nseg - 1, prog_idx_ + 160);

  std::size_t best_i = back_i;
  double best_d = seg_cost(back_i);
  for (std::size_t i = back_i; i <= fwd_i; i += 4) {
    const double d = seg_cost(i);
    if (d < best_d) {
      best_d = d;
      best_i = i;
    }
  }
  const std::size_t lo = (best_i > 4) ? best_i - 4 : 0;
  const std::size_t hi = std::min(fwd_i + 3, nseg - 1);
  for (std::size_t i = lo; i <= hi; ++i) {
    const double d = seg_cost(i);
    if (d < best_d) {
      best_d = d;
      best_i = i;
    }
  }
  prog_idx_ = best_i;

  const double ax = pts_[best_i].x;
  const double ay = pts_[best_i].y;
  const double bx = pts_[best_i + 1].x - ax;
  const double by = pts_[best_i + 1].y - ay;
  const double l2 = bx * bx + by * by;
  double t = 0.0;
  if (l2 > 1e-12) {
    t = std::clamp(((x - ax) * bx + (y - ay) * by) / l2, 0.0, 1.0);
  }
  seg_out = best_i;
  s_out = s_[best_i] + t * std::sqrt(l2);
  // 有符号横向误差：>0 表示在路径前进方向的左侧
  const double ln = std::sqrt(l2);
  if (ln > 1e-9) {
    const double tx = bx / ln;
    const double ty = by / ln;
    const double qx = ax + t * bx;
    const double qy = ay + t * by;
    e_out = (x - qx) * (-ty) + (y - qy) * tx;
    nx_out = -ty;
    ny_out = tx;
  }
}

void PidController::pointAt(double s, double & x, double & y) const
{
  const std::size_t n = pts_.size();
  if (n == 0) {
    x = 0.0;
    y = 0.0;
    return;
  }
  if (s <= 0.0) {
    x = pts_.front().x;
    y = pts_.front().y;
    return;
  }
  if (s >= s_.back()) {
    x = pts_.back().x;
    y = pts_.back().y;
    return;
  }
  const std::size_t i = std::min<std::size_t>(
    static_cast<std::size_t>(std::upper_bound(s_.begin(), s_.end(), s) - s_.begin()) - 1,
    n - 2);
  const double ds = s_[i + 1] - s_[i];
  const double t = (ds > 1e-9) ? (s - s_[i]) / ds : 0.0;
  x = pts_[i].x * (1.0 - t) + pts_[i + 1].x * t;
  y = pts_[i].y * (1.0 - t) + pts_[i + 1].y * t;
}

geometry_msgs::msg::TwistStamped PidController::computeVelocityCommands(
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

  const double dx_end = pts_.back().x - x;
  const double dy_end = pts_.back().y - y;
  const double dist_end = std::hypot(dx_end, dy_end);

  // 已到终点：输出零速（保持停住，不再抖动）
  if (dist_end < goal_tolerance_) {
    stuck_state_ = 0;
    return cmd_vel;
  }

  // 底盘的速度方向响应有滞后 (~0.15s)，直接用当前位姿做纯跟踪会产生极限环摆动。
  // 用实测速度把控制点往前推一点，等于把这段滞后补偿掉。
  double cx_p = x;
  double cy_p = y;
  if (predict_dt_ > 0.0) {
    cx_p += velocity.linear.x * predict_dt_;
    cy_p += velocity.linear.y * predict_dt_;
  }

  double s_now = 0.0;
  std::size_t seg = 0;
  double e_lat = 0.0;
  double nx = 0.0;
  double ny = 0.0;
  projectOntoPath(cx_p, cy_p, s_now, seg, e_lat, nx, ny);

  // 期望速度：当前进度点的速度剖面值
  double v_cmd = v_profile_[std::min(seg, v_profile_.size() - 1)];
  if (dist_end > 0.25) {
    v_cmd = std::max(v_cmd, min_speed_);
  }
  v_cmd = std::clamp(v_cmd, 0.0, max_linear_vel_);

  // 前视距离随速度变化：慢的时候看得近（贴线），快的时候看得远（平顺）
  const double look = std::clamp(
    lookahead_min_ + lookahead_gain_ * v_cmd, lookahead_min_, lookahead_max_);

  double tx = 0.0;
  double ty = 0.0;
  pointAt(s_now + look, tx, ty);
  double ux = tx - cx_p;
  double uy = ty - cy_p;
  const double un = std::hypot(ux, uy);
  if (un < 1e-6) {
    return cmd_vel;
  }

  double dir = std::atan2(uy, ux);

  // 横向误差反馈（PD）：纯跟踪 + 执行器滞后会形成极限环（画龙）。
  // e 是横向偏差，e_dot 是实测横向速度，两者一起构成阻尼项。
  if (lat_gain_ > 0.0 || lat_damp_ > 0.0) {
    const double e_dot = velocity.linear.x * nx + velocity.linear.y * ny;
    const double v_scale = std::max(v_cmd, 0.30);
    double corr = -(lat_gain_ * e_lat + lat_damp_ * e_dot) / v_scale;
    corr = std::clamp(corr, -0.50, 0.50);
    dir += corr;
  }

  // 方向偏差自适应：底盘实际速度方向相对指令有系统性偏差时自动抵消
  if (dir_adapt_gain_ > 0.0) {
    dir -= dir_bias_;
  }

  // 指令方向限速：底盘转向有滞后，别让它追一个转得太快的指令
  double dt = 0.02;
  if (last_cmd_stamp_.nanoseconds() != 0) {
    dt = std::clamp((now - last_cmd_stamp_).seconds(), 0.001, 0.2);
  }
  last_cmd_stamp_ = now;
  if (turn_rate_max_ > 0.0 && has_last_ang_) {
    const double step = std::clamp(wrapAngle(dir - last_cmd_ang_),
        -turn_rate_max_ * dt, turn_rate_max_ * dt);
    dir = last_cmd_ang_ + step;
  }
  has_last_ang_ = true;

  const double cmd_speed = v_cmd;
  const double meas_speed = std::hypot(velocity.linear.x, velocity.linear.y);

  // 观测器：实测方向 - 指令方向（应该在 0 附近；有系统偏差就补掉）
  if (dir_adapt_gain_ > 0.0 && v_cmd > 0.35 && meas_speed > 0.25) {
    const double act_ang = std::atan2(velocity.linear.y, velocity.linear.x);
    const double e = wrapAngle(act_ang - dir);
    dir_bias_ += dir_adapt_gain_ * wrapAngle(e - dir_bias_);
    dir_bias_ = std::clamp(dir_bias_, -0.6, 0.6);
  }
  last_cmd_ang_ = dir;

  // ---------- 卡住保护：命令有速度但实际几乎不动 -> 沿路径倒车一下 ----------
  if (stuck_state_ == 0) {
    if (cmd_speed > 0.50 && meas_speed < 0.08) {
      stuck_state_ = 1;
      stuck_since_ = now;
    }
  } else if (stuck_state_ == 1) {
    if (meas_speed > 0.15 || cmd_speed < 0.05) {
      stuck_state_ = 0;
    } else if ((now - stuck_since_).seconds() > 2.0 && escape_count_ < 3) {
      stuck_state_ = 2;
      escape_start_ = now;
      escape_count_++;
      RCLCPP_WARN(node_->get_logger(),
        "[%s] 检测到卡住，执行倒车脱困", plugin_name_.c_str());
    }
  }
  if (stuck_state_ == 2) {
    if ((now - escape_start_).seconds() < 0.9) {
      const double bx = pts_[seg + 1].x - pts_[seg].x;
      const double by = pts_[seg + 1].y - pts_[seg].y;
      const double bn = std::hypot(bx, by);
      if (bn > 1e-6) {
        const double back = std::atan2(-by, -bx);
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        cmd_vel.twist.linear.x = (c * std::cos(back) + s * std::sin(back)) * 0.30;
        cmd_vel.twist.linear.y = (-s * std::cos(back) + c * std::sin(back)) * 0.30;
        cmd_vel.twist.angular.z = 0.0;
        publishMarkers(pose, cmd_vel, yaw);
        return cmd_vel;
      }
    } else {
      stuck_state_ = 0;
    }
  }

  ux = std::cos(dir);
  uy = std::sin(dir);

  // map 系速度 -> base_link 系（底盘全向，直接旋转即可）
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  cmd_vel.twist.linear.x = (c * ux + s * uy) * v_cmd;
  cmd_vel.twist.linear.y = (-s * ux + c * uy) * v_cmd;
  cmd_vel.twist.linear.z = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  RCLCPP_DEBUG(node_->get_logger(),
    "[%s] s=%.2f/%.2f v=%.2f look=%.2f dir=%.2f bias=%.2f cmd=(%.2f, %.2f) yaw=%.2f",
    plugin_name_.c_str(), s_now, s_.back(), v_cmd, look, dir, dir_bias_,
    cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, yaw);

  publishMarkers(pose, cmd_vel, yaw);
  return cmd_vel;
}

void PidController::publishMarkers(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::TwistStamped & cmd, double yaw)
{
  if (!publish_marker_ || !marker_pub_) {
    return;
  }
  const rclcpp::Time now = node_->now();
  if (marker_stamp_.nanoseconds() != 0 && (now - marker_stamp_).seconds() < 0.1) {
    return;   // 10 Hz 足够，别刷爆 rviz
  }
  marker_stamp_ = now;

  visualization_msgs::msg::MarkerArray arr;

  visualization_msgs::msg::Marker body;
  body.header.frame_id = pose.header.frame_id.empty() ? "map" : pose.header.frame_id;
  body.header.stamp = now;
  body.ns = "robot_body";
  body.id = 0;
  body.type = visualization_msgs::msg::Marker::CUBE;
  body.action = visualization_msgs::msg::Marker::ADD;
  body.pose = pose.pose;
  body.pose.position.z = 0.05;
  body.scale.x = 0.5;
  body.scale.y = 0.5;
  body.scale.z = 0.1;
  body.color.r = 0.10f;
  body.color.g = 0.75f;
  body.color.b = 0.90f;
  body.color.a = 0.85f;
  arr.markers.push_back(body);

  // 控制器输出的速度矢量（map 系）
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double wx = c * cmd.twist.linear.x - s * cmd.twist.linear.y;
  const double wy = s * cmd.twist.linear.x + c * cmd.twist.linear.y;
  const double speed = std::hypot(wx, wy);

  visualization_msgs::msg::Marker arrow;
  arrow.header = body.header;
  arrow.ns = "cmd_vel";
  arrow.id = 1;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;
  arrow.pose.orientation.w = 1.0;
  arrow.scale.x = 0.05;
  arrow.scale.y = 0.10;
  arrow.scale.z = 0.12;
  arrow.color.r = 1.00f;
  arrow.color.g = 0.35f;
  arrow.color.b = 0.10f;
  arrow.color.a = 0.95f;

  geometry_msgs::msg::Point p0 = pose.pose.position;
  p0.z = 0.10;
  geometry_msgs::msg::Point p1 = p0;
  const double len = 0.25 + 0.45 * speed;
  if (speed > 1e-3) {
    p1.x += wx / speed * len;
    p1.y += wy / speed * len;
  }
  arrow.points.push_back(p0);
  arrow.points.push_back(p1);
  arr.markers.push_back(arrow);

  marker_pub_->publish(arr);
}

}

PLUGINLIB_EXPORT_CLASS(pid_controller::PidController, sp_controller_server::ControllerPlugin)
