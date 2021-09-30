#pragma once
#ifndef robot_config_h
#define robot_config_h
#include <Adafruit_PWMServoDriver.h>
#include "Servo.h"

//Class holding robot config values. Maybe this will be read from a file one day. For now it's defined here. 
class RobotConfiguration
{
    public:
        RobotConfiguration(Adafruit_PWMServoDriver* pPwm_ctrl);
        //these are pin numbers on the mega
        const int SERVO_FL_PIN = 44;
        const int SERVO_FR_PIN = 45;
        const int SERVO_BL_PIN = 46;
        const int SERVO_BR_PIN = 47;  
        //pin numbers on the I2C PWM controller
        const int MOTOR_FL_FWD_PIN = 1;
        const int MOTOR_FL_BKWD_PIN = 0;
        const int MOTOR_FR_FWD_PIN = 3;
        const int MOTOR_FR_BKWD_PIN = 2;
        const int MOTOR_BL_FWD_PIN = 5;
        const int MOTOR_BL_BKWD_PIN = 4;
        const int MOTOR_BR_FWD_PIN = 7;
        const int MOTOR_BR_BKWD_PIN = 6;
        //Encoder pin numbers
        const int ENCODER_BL_1 = 2;
        const int ENCODER_BL_2 = 4;

        const int ENCODER_FL_1 = 3;
        const int ENCODER_FL_2 = 5;

        const int ENCODER_BR_1 = 19;
        const int ENCODER_BR_2 = 22;

        const int ENCODER_FR_1 = 18;
        const int ENCODER_FR_2 = 17; 

        Adafruit_PWMServoDriver* pwm_controller;
};


#endif