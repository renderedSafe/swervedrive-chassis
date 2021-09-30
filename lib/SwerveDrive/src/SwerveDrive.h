#pragma once
#ifndef SwerveDrive_h
#define SwerveDrive_h
#include "Servo.h"
#include <Encoder.h>
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>
#include "robot_config.h"
//#include <robot_config.h>




enum Direction {BACKWARD = -1, FORWARD = 1};
class DcMotor
{
    public:
        //This is a pointer object because many DC motor objects will all use a single PWM controller object representing a single piece of hardware
        Adafruit_PWMServoDriver *pPwm_controller;
        DcMotor(int pin_fwd, int pin_bkwd, Adafruit_PWMServoDriver* pPwm_ctrl, Direction dir);
        DcMotor(const DcMotor&); //copy constructor because of the pointer member
        DcMotor & operator= (const DcMotor old_DcMotor);
        void setDirection(Direction dir);
        void setPower(float power);
        float getPower();

    private:
        float power;
        int direction;
        int _pin_fwd, _pin_bkwd;

};

//Encoder abstraction class. Provides speed calculation. TODO: Add pulse to distance conversion
class Odometer
{
    public:
        Encoder encoder;
        //Update function manages calculations. Call this every loop. 
        bool update();
        float last_speed;
        Odometer(int enc_pin_1, int enc_pin_2, int spd_update_interval_ms=5);
    private:
        long prev_pos;
        unsigned long prev_millis;
        int calc_interval_ms;
        //Calculates speed in pulses per second. Accessed by update().  
        float getSpeed();
};

class EncoderDcMotor
{
    public:
        int max_speed;  //Max speed in pulses per second the motor is allowed to run. Unimplemented. 
        DcMotor dc_motor;
        Odometer odometer;
        PID pid_controller;
        EncoderDcMotor(int pin_fwd, int pin_bkwd, Adafruit_PWMServoDriver* pPwm_ctrl, Direction dir, \
                       int enc_pin_1, int enc_pin_2, int maxspeed=6000, float kp=1.5, float ki=0.001, float kd=0.0001);
        //Updates current speed and computes PID output. Call this every loop. 
        void update(); 
        //Sets a target number of pulses per second to see on the encoder. 
        void setSpeed(float speed);
        //Gets the PID output value. 
        double getOutput() {return _output;}
    private:
        double _setpoint, _input, _output;  //PID variables
        

};

class SwerveDrive
{
    public:
        RobotConfiguration robot_configuration;
        Servo servo_fl;
        Servo servo_fr;
        Servo servo_bl;
        Servo servo_br;
        EncoderDcMotor motor_fl;
        EncoderDcMotor motor_fr;
        EncoderDcMotor motor_bl;
        EncoderDcMotor motor_br;
        //updates the members that need updating TODO: Implement
        void update();
        //performs configutaion tasks that must be done in the setup block.
        void initilialize();
        //turns the front and back wheels in opposite directions, changing the heading of the vehicle on wheel movement.
        void headingTurn(int angle);
        //turns all the wheels the same direction, translating the 
        void translationTurn(int angle);
        //sets the speed of the drive motors
        void setDrivePower(double power);
        //Bring your own objects constructor.
        //SwerveDrive(Servo servo_fl, Servo servo_fr, Servo servo_bl, Servo servo_br, 
        //            EncoderDcMotor motor_fl, EncoderDcMotor motor_fr, EncoderDcMotor motor_bl, EncoderDcMotor motor_br);
        //Configuration object constructor
        SwerveDrive(RobotConfiguration robot_config);
};
#endif