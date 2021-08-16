#include "SwerveDrive.h"
#include <Arduino.h>
#include <math.h>

//TODO: See about builder constructor pattern
/*
*Description?
**/
SwerveDrive::SwerveDrive(Servo servo_fl, 
                         Servo servo_fr,
                         Servo servo_bl,
                         Servo servo_br,
                         DcMotor motor_fl,
                         DcMotor motor_fr,
                         DcMotor motor_bl,
                         DcMotor motor_br) : servo_fl(servo_fl),
                                             servo_fr(servo_fr),
                                             servo_bl(servo_bl),
                                             servo_br(servo_br),
                                             motor_fl(motor_fl),
                                             motor_fr(motor_fr),
                                             motor_bl(motor_bl),
                                             motor_br(motor_br)
{
  
};


//TODO: implement Ackermann turns eventually
void SwerveDrive::headingTurn(int angle)
{
    if(abs(angle) > 90)
    {
        angle = copysign(90, angle);
    }
    //turn the front and back wheels opposite directions
    int front_angle = 90 + angle;
    int back_angle = 90 - angle;
    servo_fl.write(front_angle);
    servo_fr.write(front_angle);
    servo_bl.write(back_angle);
    servo_br.write(back_angle);
}

//turns all wheels the same direction to move stright that way without turning the chassis
void SwerveDrive::translationTurn(int angle)
{
    if(abs(angle) > 90)
    {
        angle = copysign(90, angle);
    }
    servo_fl.write(90 + angle);
    servo_fr.write(90 + angle);
    servo_bl.write(90 + angle);
    servo_br.write(90 + angle);
}

void SwerveDrive::setDrivePower(double power)
{
    motor_fl.setPower(power);
    motor_fr.setPower(power);
    motor_bl.setPower(power);
    motor_br.setPower(power);
}         




//**********************DC Motor Class*******************

DcMotor::DcMotor(int pin_fwd, int pin_bkwd, 
                 Adafruit_PWMServoDriver &pwm_controller, Direction dir=FORWARD):pwm_controller(pwm_controller)
{
    _pin_fwd = pin_fwd;
    _pin_bkwd = pin_bkwd;
    direction = dir;
}

void DcMotor::setDirection(Direction dir)
{
    direction = dir;
}

//Meant to take -1.0 to 1.0
void DcMotor::setPower(float pwr)
{
    float scaled_power = pwr * 4095;

    //handle a pwm value overflow. 12 bits
    if (abs(scaled_power) >= 4095)
    {
        scaled_power = copysign(4095, scaled_power); 
    }


    if (scaled_power > 0)
    {
        pwm_controller.setPin(_pin_fwd, abs((int)scaled_power));
        pwm_controller.setPin(_pin_bkwd, 0);
    }
    else
    {
        pwm_controller.setPin(_pin_fwd, 0);
        pwm_controller.setPin(_pin_bkwd, abs((int)scaled_power));
    }
}

float DcMotor::getPower()
{
    return power;
}