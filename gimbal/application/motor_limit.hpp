#ifndef MOTOR_LIMIT_HPP
#define MOTOR_LIMIT_HPP

#include <stdint.h>

enum class MotorLimitState : uint32_t { NORMAL, OUT_OF_RANGE, RECOVERING };
extern volatile MotorLimitState motor_limit_state;

#endif
