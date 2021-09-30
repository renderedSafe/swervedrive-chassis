#include "SwerveDrive.h"
#include <Arduino.h>
#include <math.h>

//TODO: See about builder constructor pattern
/*
*Description?
**/
/* SwerveDrive::SwerveDrive(Servo servo_fl, 
                         Servo servo_fr,
                         Servo servo_bl,
                         Servo servo_br,
                         EncoderDcMotor motor_fl,
                         EncoderDcMotor motor_fr,
                         EncoderDcMotor motor_bl,
                         EncoderDcMotor motor_br) : servo_fl(servo_fl),
                                             servo_fr(servo_fr),
                                             servo_bl(servo_bl),
                                             servo_br(servo_br),
                                             motor_fl(motor_fl),
                                             motor_fr(motor_fr),
                                             motor_bl(motor_bl),
                                             motor_br(motor_br)
{
  
}; */

SwerveDrive::SwerveDrive(RobotConfiguration robot_config)
    : servo_fl(Servo()),
    servo_fr(Servo()),
    servo_bl(Servo()),
    servo_br(Servo()),
    motor_fl(EncoderDcMotor(robot_config.MOTOR_FL_FWD_PIN,
                              robot_config.MOTOR_FL_BKWD_PIN,
                              robot_config.pwm_controller,
                              FORWARD,
                              robot_config.ENCODER_FL_1,
                              robot_config.ENCODER_FL_2
                              )),
    motor_fr(EncoderDcMotor(robot_config.MOTOR_FR_FWD_PIN,
                              robot_config.MOTOR_FR_BKWD_PIN,
                              robot_config.pwm_controller,
                              FORWARD,
                              robot_config.ENCODER_FR_1,
                              robot_config.ENCODER_FR_2
                              )),
    motor_bl(EncoderDcMotor(robot_config.MOTOR_BL_FWD_PIN,
                              robot_config.MOTOR_BL_BKWD_PIN,
                              robot_config.pwm_controller,
                              FORWARD,
                              robot_config.ENCODER_BL_1,
                              robot_config.ENCODER_BL_2
                              )),      
    motor_br(EncoderDcMotor(robot_config.MOTOR_BR_FWD_PIN,
                              robot_config.MOTOR_BR_BKWD_PIN,
                              robot_config.pwm_controller,
                              FORWARD,
                              robot_config.ENCODER_BR_1,
                              robot_config.ENCODER_BR_2
                              )),
    robot_configuration(robot_config)
{
    
};

void SwerveDrive::update()
{
    SwerveDrive::motor_bl.update();
    SwerveDrive::motor_br.update();
    SwerveDrive::motor_fl.update();
    SwerveDrive::motor_fr.update();
}

void SwerveDrive::initilialize()
{
    servo_fl.attach(robot_configuration.SERVO_FL_PIN);
    servo_fr.attach(robot_configuration.SERVO_FR_PIN);
    servo_bl.attach(robot_configuration.SERVO_BL_PIN);
    servo_br.attach(robot_configuration.SERVO_BR_PIN);
    //start pwm controller
    //robot_configuration.pwm_controller->begin();
    //configure pwm controller. PWM freq for DC motors should be higher but this is high as we can go with this controller
    //robot_configuration.pwm_controller->setOscillatorFrequency(27000000);
    //robot_configuration.pwm_controller->setPWMFreq(1500);  
    //delay accounts for I2C write
    //delay(10);
}


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
    motor_fl.setSpeed(power);
    motor_fr.setSpeed(power);
    motor_bl.setSpeed(power);
    motor_br.setSpeed(power);
}         




//**********************DC Motor Class*******************

DcMotor::DcMotor(int pin_fwd, int pin_bkwd, 
                 Adafruit_PWMServoDriver* pPwm_ctrl, Direction dir=FORWARD)
{
    pPwm_controller = pPwm_ctrl;
    _pin_fwd = pin_fwd;
    _pin_bkwd = pin_bkwd;
    direction = dir;
}

DcMotor::DcMotor(const DcMotor& old_DcMotor)
{
    pPwm_controller = old_DcMotor.pPwm_controller;
    power = old_DcMotor.power;
    direction = old_DcMotor.direction;
    _pin_fwd = old_DcMotor._pin_fwd;
    _pin_bkwd = old_DcMotor._pin_bkwd;
}

DcMotor & DcMotor::operator= (const DcMotor old_DcMotor)
{
    power = old_DcMotor.power;
    pPwm_controller = old_DcMotor.pPwm_controller;
    direction = old_DcMotor.direction;
    _pin_bkwd = old_DcMotor._pin_bkwd;
    _pin_fwd = old_DcMotor._pin_fwd;

    return *this;
}

void DcMotor::setDirection(Direction dir)
{
    direction = dir;
}

//Meant to take -1.0 to 1.0
void DcMotor::setPower(float pwr)
{
    power = pwr;
    float scaled_power = pwr * 4095;

    //handle a pwm value overflow. 12 bits
    if (abs(scaled_power) >= 4095)
    {
        scaled_power = copysign(4095, scaled_power); 
    }


    if (scaled_power > 0)
    {
        pPwm_controller->setPin(_pin_fwd, abs((int)scaled_power));
        pPwm_controller->setPin(_pin_bkwd, 0);
    }
    else
    {
        pPwm_controller->setPin(_pin_fwd, 0);
        pPwm_controller->setPin(_pin_bkwd, abs((int)scaled_power));
    }
}

float DcMotor::getPower()
{
    return power;
}

//*********************Odometer class************************

Odometer::Odometer(int enc_pin_1, int enc_pin_2, int spd_update_interval_ms) 
    : encoder(enc_pin_1, enc_pin_2)
{
    calc_interval_ms = spd_update_interval_ms;
    //these two values must be initialed to prevent issues with referencing them below
    prev_pos = encoder.read();
    prev_millis = millis();
    last_speed = 0;
}

bool Odometer::update()
{
    //early exit for most common situation (too early to recalc speed)
    if ((millis() - prev_millis) < calc_interval_ms)
    {
        return false;
    }

    else
    {
        last_speed = getSpeed();
        return true;
    }
}

float Odometer::getSpeed()
{
    float spd = float(encoder.read() - prev_pos)/float(millis() - prev_millis) * 1000;  //convert to pulses/second
    prev_millis = millis();
    prev_pos = encoder.read();
    return spd;
}

//************************Encoder DC Motor Class*****************
EncoderDcMotor::EncoderDcMotor(int pin_fwd, int pin_bkwd, Adafruit_PWMServoDriver* controller, Direction dir, 
                                    int enc_pin_1, int enc_pin_2, int maxspeed, float kp, float ki, float kd) : 
                                    dc_motor(DcMotor(pin_fwd, pin_bkwd, controller, dir)),
                                    odometer(Odometer(enc_pin_1, enc_pin_2)),
                                    pid_controller(PID(&_input, &_output, &_setpoint, kp, ki, kd, DIRECT))
{
    max_speed = maxspeed;
    pid_controller.SetMode(AUTOMATIC);
    pid_controller.SetOutputLimits(-1000, 1000);
    pid_controller.SetSampleTime(5);
}

void EncoderDcMotor::update()
{
    //TODO: only do this stuff if we've gotten new odo data to work with
    odometer.update();
    _input = odometer.last_speed;
    pid_controller.Compute();


    //!!!!!!!!!!!!!!!HARDWARE SAFETY CODE BELOW!!!!!!!!!!!!!!!!

    //When geared brushed motors are in motion, a current applied in the opposite 
    //direction of rotation is VERY BAD for the motor. This makes sure we only apply 
    //current in the direction we're trying to move and relies on passive braking
    //rather than active control to slow the motor down. Some amount of negative 
    //current may be acceptable to control more effectuvely, but for now we're playing it
    //safe. 
    if ((_output<0)==(_setpoint<0))
    {
        dc_motor.setPower(_output/1000);
    }
    else
    {
        dc_motor.setPower(0);
    }
    
    
}

void EncoderDcMotor::setSpeed(float speed)
{
    _setpoint = speed * max_speed;
}