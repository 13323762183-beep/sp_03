#ifndef LQR_CONTROLLER_HPP_
#define LQR_CONTROLLER_HPP_

#include <memory>
#include <string>

#include <Eigen/Dense>

#include "pid_controller.hpp"

namespace lqr_controller {

/**
 * 基于 LQR 的全向底盘路径跟踪控制器（加分项：自研优于 PID 的控制器）。
 *
 * 与 PidController 共用同一套轨迹层（Chaikin 平滑 / 等弧长重采样 / 曲率限速 /
 * 刹车反传），只把控制律换掉，所以两者的差异可以干净地归因到控制律本身。
 *
 * 控制律推导：
 *   实测底盘的速度环是一个一阶滞后环节（时间常数 tau ≈ 0.08~0.15 s）：
 *
 *       dp/dt = v,      dv/dt = (u - v) / tau
 *
 *   其中 u 是指令速度、v 是实际速度。传统纯跟踪忽略了这一项，只能用
 *   "把控制点沿速度方向前推" 这种土办法补偿（predict_dt）；而 LQR 直接把
 *   tau 写进状态方程，让 Riccati 解出来的增益自带阻尼。
 *
 *   取误差状态 e = [ex, ey, evx, evy]^T（位置误差、速度误差），
 *   参考速度作为前馈 u = v_ref - K e，在代价
 *       J = ∫ (e^T Q e + u^T R u) dt
 *   下最优。Q 里位置权重 q_pos 越大越"贴线"，R 越大越平顺。
 *
 *   K 在 configure() 里用离散 Riccati 迭代算一次（4x4，几十微秒），
 *   运行时只是两次 4 维点积，不增加控制周期的计算负担。
 */
class LqrController : public pid_controller::PidController {
public:
  LqrController() = default;
  ~LqrController() override = default;

  void configure(
    const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

private:
  /// 离散 Riccati 迭代求反馈增益 K
  void solveLqr();
  /// 速度剖面按弧长线性插值
  double profileAt(double s) const;

  // ---- LQR 参数 ----
  double tau_{0.10};          // 底盘速度响应时间常数 s
  double q_pos_{4.0};         // 位置误差权重
  double q_vel_{1.0};         // 速度误差权重
  double r_ctrl_{2.0};        // 控制量权重（越大越平顺）
  double ctrl_dt_{0.02};      // 离散化步长（与控制周期一致 50 Hz）
  double look_gain_{0.10};    // 参考点前视距离随速度的增益 s
  double cmd_smooth_{0.06};   // 指令一阶低通时间常数（抑制方向抖动）
  double pos_err_clip_{0.50}; // 位置误差限幅 m：偏差再大也不让它把控制量顶爆

  Eigen::Matrix4d P_{Eigen::Matrix4d::Zero()};
  Eigen::Matrix<double, 2, 4> K_{Eigen::Matrix<double, 2, 4>::Zero()};

  bool has_prev_cmd_{false};
  Eigen::Vector2d prev_cmd_{Eigen::Vector2d::Zero()};
};

}

#endif
