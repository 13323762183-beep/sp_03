#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>

#include "sp_controller_server/controller_plugin.hpp"

namespace pid_controller {

struct Point2d
{
  double x{0.0};
  double y{0.0};
};

/**
 * 全向底盘路径跟踪控制器。
 *
 * 思路（纯跟踪 / pure-pursuit + 曲率限速）：
 *   1. setPlan() 收到 /global_path 后对折线做轻度平滑，计算弧长、曲率；
 *   2. 由曲率得到每个路径点的限速，再从终点往回做一次“刹车”反向传播，
 *      得到沿路径的期望速度剖面 v(s)；
 *   3. 每个控制周期在路径上取当前投影点前方 L = v * T 处的引导点，
 *      速度方向指向引导点，幅值取 v(s)；
 *   4. 把 map 系下的速度旋到 base_link 系后下发。
 */
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
  double distToRef(const Point2d & p) const;

protected:
  void publishMarkers(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::TwistStamped & cmd, double yaw);
  // 轨迹层（路径平滑 / 投影 / 参考点查询）对子类开放：
  // LqrController 复用同一套轨迹层、只替换控制律，这样对比实验是干净的
  // （控制变量：同一条参考轨迹，只差「纯跟踪」与「LQR」）。
  void buildPath();
  bool planDeviates(const std::vector<Point2d> & cand) const;
  void projectOntoPath(
    double x, double y, double & s, std::size_t & seg, double & e,
    double & nx, double & ny);
  void pointAt(double s, double & x, double & y) const;

  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Node::SharedPtr node_;
  nav_msgs::msg::Path global_plan_;
  std::string base_frame_id_;

  // ---- 可调参数（nav_params.yaml 中 <plugin_name>.* 覆盖）----
  double max_linear_vel_{1.35};     // 跟踪最大线速度 m/s（实测底盘可达约 1.4）
  double lat_acc_{2.2};             // 允许的向心加速度 m/s^2（决定过弯速度）
  double dec_acc_{1.2};             // 规划用减速度 m/s^2
  double lookahead_min_{0.30};      // 最小前视距离 m
  double lookahead_max_{0.75};      // 最大前视距离 m
  double lookahead_gain_{0.30};     // 前视距离随速度的增益 s
  double goal_tolerance_{0.05};     // 终点判定容差 m（小于此值输出零速）
  double min_speed_{0.05};          // 防止速度剖面为 0 导致原地不动
  int smooth_passes_{2};            // 平滑迭代次数（0 表示不平滑）
  bool fine_traj_{false};           // 精细轨迹模式（加分项）：Chaikin + 等弧长重采样 + 三点曲率
  double ref_dev_tol_{0.60};        // 新规划与本控制器参考路径的最大偏离，超过才换参考
  double lat_gain_{0.8};            // 横向误差反馈增益（P）
  double lat_damp_{0.6};            // 横向速度阻尼（D），压纯跟踪的摆动
  double predict_dt_{0.15};         // 位置预测时间（补偿底盘速度响应滞后），0=关闭
  double turn_rate_max_{0.0};       // 指令方向最大转速 rad/s（0=不限）
  double dir_adapt_gain_{0.0};      // 方向偏差自适应增益（0=关闭）

  // ---- 参考路径（固定，不被 10Hz 重规划的抖动带着走）----
  std::vector<Point2d> raw_pts_;
  std::vector<Point2d> pts_;
  std::vector<double> s_;
  std::vector<double> v_profile_;
  bool plan_dirty_{true};
  bool has_ref_{false};
  std::size_t prog_idx_{0};         // 进度索引：投影只在这个窗口内搜索
  double last_cmd_ang_{0.0};        // 上一周期的指令方向（用于限速转动）
  bool has_last_ang_{false};
  rclcpp::Time last_cmd_stamp_;
  double dir_bias_{0.0};            // 实测方向相对指令方向的偏差（自适应补偿）

  // ---- 可视化 / 脱困 ----
  bool publish_marker_{true};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Time marker_stamp_;

  int stuck_state_{0};          // 0=正常跟线, 1=检测到卡住, 2=倒车脱困
  rclcpp::Time stuck_since_;
  rclcpp::Time escape_start_;
  int escape_count_{0};      // 一次任务最多脱困 3 次，避免来回打转
};

}

#endif
