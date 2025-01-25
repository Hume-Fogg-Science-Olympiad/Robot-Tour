#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_ConquerorCarxxx0;

void setup() {
  AppMotor.DeviceDriverSet_Motor_Init();
  delay(2000);

  Application_ConquerorCarxxx0.Motion_Control = 1;

}

void loop() {
  ApplicationFunctionSet_ConquerorCarMotionControl(Application_ConquerorCarxxx0.Motion_Control /*direction*/, 150 /*speed*/);
  delay(1000);

  if (Application_ConquerorCarxxx0.Motion_Control == 1) {
    Application_ConquerorCarxxx0.Motion_Control = 2;
  } else {
    Application_ConquerorCarxxx0.Motion_Control = 1;
  }
}
