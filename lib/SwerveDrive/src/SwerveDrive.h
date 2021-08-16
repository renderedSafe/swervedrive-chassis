#pragma once
#ifndef SwerveDrive_h
#define SwerveDrive_h
#include "Servo.h"
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>




enum Direction {BACKWARD = -1, FORWARD = 1};
class DcMotor
{
    
    public:
        Adafruit_PWMServoDriver &pwm_controller;
        DcMotor(int pin_fwd, int pin_bkwd, Adafruit_PWMServoDriver&, Direction dir);
        void setDirection(Direction dir);
        void setPower(float power);
        float getPower();

    private:
        float power;
        int direction;
        int _pin_fwd, _pin_bkwd;

};

class SwerveDrive
{
    public:
    Servo servo_fl;
    Servo servo_fr;
    Servo servo_bl;
    Servo servo_br;
    DcMotor motor_fl;
    DcMotor motor_fr;
    DcMotor motor_bl;
    DcMotor motor_br;
    void headingTurn(int angle);
    void translationTurn(int angle);
    void setDrivePower(double power);
    SwerveDrive(Servo, Servo, Servo, Servo, DcMotor, DcMotor, DcMotor, DcMotor);
};
#endif