/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:55:26
 * @LastEditors: Changhua
 * @Description: conqueror robot tank
 * @FilePath: 
 */
#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"

DeviceDriverSet_Motor AppMotor;
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_IRrecv AppIRrecv;

ConquerorCarMotionControl status = stop_it;

void setup() {
  // put your setup code here, to run once:
  AppMotor.DeviceDriverSet_Motor_Init();
  AppMPU6050getdata.MPU6050_dveInit();
  delay(2000);
  AppMPU6050getdata.MPU6050_calibration();
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);

  Serial.begin(9600);
  AppIRrecv.DeviceDriverSet_IRrecv_Init();
}

void loop() {
  uint8_t IRrecv_button;
  static bool IRrecv_en = false;

  AppIRrecv.DeviceDriverSet_IRrecv_Get(&IRrecv_button /*out*/);
  Serial.println(IRrecv_button);
  switch (IRrecv_button) {
    case /* constant-expression */ 1:
      /* code */
      status = Forward;
      break;
    case /* constant-expression */ 2:
      /* code */
      status = Backward;
      break;
    case /* constant-expression */ 3:
      /* code */
      status = Left;
      break;
    case /* constant-expression */ 4:
      /* code */
      status = Right;
      break;
    case /* constant-expression */ 5:
      /* code */
      status = stop_it;
      break;
    case /* constant-expression */ 6:
      Serial.println("TraceBased_mode");
      break;
    case /* constant-expression */ 7:
      Serial.println("ObstacleAvoidance_mode");
      break;
    case /* constant-expression */ 8:
      Serial.println("Follow_mode");
      break;
    default:
      status = stop_it;
      break;
  }

  if (status != Left && status != Right) {
    ApplicationFunctionSet_ConquerorCarMotionControl(status, 250);
  } else if (status == Left) {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ 75,
                                             /*direction_B*/ direction_back, /*speed_B*/ 75, /*controlED*/ control_enable);
  } else if (status == Right) {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ 75,
                                             /*direction_B*/ direction_just, /*speed_B*/ 75, /*controlED*/ control_enable);
  }
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
}
