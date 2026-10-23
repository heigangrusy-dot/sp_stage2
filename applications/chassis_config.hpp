#ifndef CHASSIS_CONFIG_HPP
#define CHASSIS_CONFIG_HPP

#include <cstdint>

namespace chassis_config
{
// ---------- 几何与传动 ----------
// 长度m，减速比为电机轴转速/输出轴转速。
constexpr float WHEEL_RADIUS = 0.077f;
constexpr float HALF_LENGTH = 0.165f;
constexpr float HALF_WIDTH = 0.185f;
constexpr float GEAR_RATIO = 13.7f;

// ---------- 遥控 ----------
// 线速度m/s，角速度rad/s；输入阈值为归一化比例。
constexpr float MAX_LINEAR_SPEED = 1.0f;
constexpr float MAX_LATERAL_SPEED = 1.0f;
constexpr float MAX_ANGULAR_SPEED = 1.0f;
constexpr float INPUT_DEADBAND = 0.05f;
constexpr float STICK_AXIS_SNAP_RATIO = 0.20f;  // 小幅次轴输入吸附到主轴。

// ---------- 前后加减速 ----------
// 单位m/s²；数值越大，速度变化越快。
constexpr float FORWARD_ACCELERATION = 8.0f;
constexpr float FORWARD_DECELERATION = 5.0f;

// ---------- 上档自动直行 ----------
// 下档停稳后切上档，只执行一次；距离由四轮编码器估算，打滑会产生误差。
constexpr float AUTO_DISTANCE_M = 3.5f;
constexpr float AUTO_SPEED_MPS = 0.5f;     // 首次验证速度，独立于遥控最高速度。
constexpr float AUTO_ACCELERATION = 0.8f;  // m/s²。
constexpr float AUTO_DECELERATION = 1.0f;  // m/s²，实际制动仍受功率和转矩约束。
constexpr float AUTO_POSITION_KP = 2.0f;   // 末段距离误差m -> 目标速度m/s。
constexpr float AUTO_STOP_TOLERANCE_M = 0.02f;
constexpr uint32_t AUTO_STOP_SETTLE_MS = 200;
constexpr uint32_t AUTO_TIMEOUT_MS = 20000;
constexpr uint32_t AUTO_MAX_GAP_MS = 50;  // 自动过程中调度长时间中断则退出。

// ---------- IMU航向保持 ----------
// C板X朝左、Y朝后；水平安装时Z仍向上，顺时针Yaw减小，方向系数取+1。
constexpr bool HEADING_HOLD_ENABLED = true;
constexpr float HEADING_YAW_SIGN = 1.0f;         // 仅用于核对方向；同时作用于Yaw和角速度。
constexpr float HEADING_KP = 2.5f;               // 航向误差rad -> 修正角速度rad/s。
constexpr float HEADING_KD = 0.20f;              // 航向角速度阻尼，不是四轮速度环KD。
constexpr float HEADING_MAX_CORRECTION = 0.25f;  // 修正幅值上限，rad/s。
constexpr float HEADING_DEADBAND = 0.01f;        // 约0.57°，小角差不反复修正。
constexpr float HEADING_RATE_DEADBAND = 0.02f;   // rad/s。
constexpr float HEADING_SETTLE_RATE = 0.05f;     // 主动转弯结束后的停稳阈值，rad/s。
constexpr uint32_t HEADING_SETTLE_MS = 150;
constexpr uint32_t IMU_CALIBRATION_SAMPLES = 2000;  // 开机保持静止约2秒。
constexpr uint32_t IMU_HEADING_TIMEOUT_MS = 50;

// ---------- 四轮速度环 ----------
// 反馈为轮侧rad/s，输出和积分限幅为轮侧N·m。
constexpr float SPEED_KP = 0.40f;
constexpr float SPEED_KI = 0.10f;
constexpr float SPEED_KD = 0.0f;
constexpr float MAX_WHEEL_TORQUE = 0.8f;
constexpr float MAX_INTEGRAL_TORQUE = 0.45f;

// ---------- 停车 ----------
// 四轮目标全零后淡出积分，停稳后清除历史；重新运动时恢复积分权重。
constexpr float STOPPED_WHEEL_SPEED = 0.10f;  // 轮侧rad/s。
constexpr uint32_t STOPPED_RESET_DELAY_MS = 100;
constexpr float STOP_INTEGRAL_FADE_S = 0.15f;

// ---------- 控制与观测周期 ----------
// 分别使用固定周期控制和独立绘图任务。
constexpr uint32_t CONTROL_PERIOD_MS = 1;
constexpr float CONTROL_PERIOD_S = CONTROL_PERIOD_MS * 0.001f;
constexpr uint32_t PLOT_PERIOD_MS = 50;

// ---------- 功率预测 ----------
// 输入轮侧N·m、rad/s，输出W。
constexpr float POWER_K1 = 4.4f;    // 转矩平方损耗系数。
constexpr float POWER_K2 = 0.013f;  // 轮速平方损耗系数。
constexpr float POWER_K3 = 3.8f;    // 静态功耗，W。

// ---------- 功率限制 ----------
// 正式上限取PM02报文与校内赛80W中的较小值。
constexpr bool POWER_LIMIT_ENABLED = true;
constexpr float COMPETITION_POWER_LIMIT_W = 80.0f;
constexpr float POWER_TEST_LIMIT_W = 0.0f;  // 首次验证用；正式测试改为0，取消额外测试限制。
constexpr float POWER_MARGIN_W = 5.0f;      // 暂留模型误差余量，需按实测调整。
constexpr uint32_t PM02_TIMEOUT_MS = 1000;  // 未收到有效上限或状态超时，禁止底盘输出。
}  // namespace chassis_config

#endif  // CHASSIS_CONFIG_HPP
