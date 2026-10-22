#ifndef TRACKING_SCORE_HPP
#define TRACKING_SCORE_HPP

void yaw_detect(float gimbal_angle, float plan_angle, float* fire_ratio);
void pitch_detect(float gimbal_angle, float plan_angle, float* fire_ratio);

extern float yaw_ratio;
extern float pitch_ratio;
extern float yaw_score;    // ratio * 100，停止测试后保留
extern float pitch_score;

#endif
