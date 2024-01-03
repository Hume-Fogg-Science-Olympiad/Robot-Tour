#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_SmartRobotCarxxx0;

void setup() {
  Serial.begin(9600);

  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000);
  for (Application_SmartRobotCarxxx0.Motion_Control = 0; Application_SmartRobotCarxxx0.Motion_Control < 9; Application_SmartRobotCarxxx0.Motion_Control = Application_SmartRobotCarxxx0.Motion_Control + 1)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(Application_SmartRobotCarxxx0.Motion_Control /*direction*/, 200 /*speed*/);
    delay(1000);
  }
}

void loop() {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ 100,
                                             /*direction_B*/ direction_just, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
}