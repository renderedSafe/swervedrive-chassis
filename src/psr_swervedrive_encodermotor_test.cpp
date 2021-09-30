#include <Arduino.h>
#include "Servo.h"
#include <Adafruit_PWMServoDriver.h>
#include "SwerveDrive.h"
#include "SerialTransfer.h"
#include "robot_config.h"

SerialTransfer myTransfer;
//TODO: move these struct definitions out of the main file. Only do initialization here. 
struct __attribute__((__packed__)) STATE_COMMAND_STRUCT 
{
  float turn_angle;
  float drive_power;
} struct_state_command;
struct __attribute__((__packed__)) TELEMETRY_STRUCT
{
  float target_speed;
  float actual_speed;
  unsigned long time;
} struct_telemetry;

Adafruit_PWMServoDriver pwm_ctrl = Adafruit_PWMServoDriver();
RobotConfiguration* robot_config;
SwerveDrive* swerve_drive;
void setup() 
{
  Serial3.begin(9600);
  Serial.begin(115200);
  myTransfer.begin(Serial);
  
  Serial3.println("Starting setup.");
  pwm_ctrl.begin();
  robot_config = new RobotConfiguration(&pwm_ctrl);
  swerve_drive = new SwerveDrive(*robot_config);
  swerve_drive->initilialize();
  //swerve_drive->robot_configuration.pwm_controller->begin();
  //swerve_drive.initilialize();  //this function does stuff that can't happen on definition above
  
  Serial3.println("Setup function complete.");
}

void loop() 
{ 
  if (myTransfer.available())
  {
    myTransfer.rxObj(struct_state_command);
    //myTransfer.sendDatum(struct_telemetry);  //telemetry. The python script receiving must expect the same data format sent from here
  }
  swerve_drive->update();
  swerve_drive->translationTurn(struct_state_command.turn_angle);
  swerve_drive->setDrivePower(struct_state_command.drive_power);
  //Serial3.print("Commanded speed: ");
  //Serial3.println(swerve_drive.motor_bl.max_speed * struct_state_command.drive_power);
}

