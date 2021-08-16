#include <Arduino.h>
#include "Servo.h"
#include <Adafruit_PWMServoDriver.h>
#include "SwerveDrive.h"

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
//Servo Objects
Servo servo_br;
Servo servo_bl;
Servo servo_fl;
Servo servo_fr;
//PWM controller object
Adafruit_PWMServoDriver pwm_controller = Adafruit_PWMServoDriver();
//Creating the SwerveDrive object. It crntly req lots of objects in the constructor
SwerveDrive swerve_drive = SwerveDrive(servo_fl,servo_fr,servo_bl, servo_br,
                                         DcMotor(MOTOR_FL_FWD_PIN, MOTOR_FL_BKWD_PIN, pwm_controller, FORWARD),
                                         DcMotor(MOTOR_FR_FWD_PIN, MOTOR_FR_BKWD_PIN, pwm_controller, FORWARD),
                                         DcMotor(MOTOR_BL_FWD_PIN, MOTOR_BL_BKWD_PIN, pwm_controller, FORWARD),
                                         DcMotor(MOTOR_BR_FWD_PIN, MOTOR_BR_BKWD_PIN, pwm_controller, FORWARD));


//for drive power watchdog timer
long lastDriveCommandMillis = 0;

void setup() 
{
  //Setting up servo objects
  servo_br.attach(SERVO_BR_PIN);
  servo_bl.attach(SERVO_BL_PIN);
  servo_fl.attach(SERVO_FL_PIN);
  servo_fr.attach(SERVO_FR_PIN);
  //start pwm controller
  pwm_controller.begin();
  //configure pwm controller. PWM freq for DC motors should be higher but this is high as we can go with this controller
  pwm_controller.setOscillatorFrequency(27000000);
  pwm_controller.setPWMFreq(1500);  
  //delay accounts for I2C write
  delay(10);
  Serial.begin(9600);
}

void loop() 
{
  
      lastDriveCommandMillis = millis();
      break;

      case DRIVE_BKWD:
      all_stop = false;
      commandedSpeed = copysign(commandedSpeed, -1);  //we're going backwards, change commandedSpeed to reflect 
      lastDriveCommandMillis = millis();
      break;

      case POWER_FLAG:
      int rawPower = 0;
      int mappedPower = 0;
      rawPower = Serial.parseInt();
      mappedPower = map(rawPower, 0, 255, 0, 1000);
      commandedSpeed = (double)mappedPower / 1000.0;
      break;
    }  //end switch

    if (Serial.available() > 0)
    {
       Serial.flush();  //we need to get the freshest serial data so we don't build up old commands
    }
  }  //end if serial available


  //sets active drive power from flag. checks watchdog timer. 
  if (!all_stop && (millis() - lastDriveCommandMillis) < 220)
  {
    swerve_drive.setDrivePower(commandedSpeed);
  }
  else  //ALL STOP!!
  {
    all_stop = true;
    swerve_drive.setDrivePower(0);
  }

  //swerve_drive.translationTurn(commandedTurnAngle);
  swerve_drive.headingTurn(commandedTurnAngle);


}

