#include "robot_config.h"
#include <Arduino.h>

RobotConfiguration::RobotConfiguration(Adafruit_PWMServoDriver* pPwm_ctrl)
{
  pwm_controller = pPwm_ctrl;
}
