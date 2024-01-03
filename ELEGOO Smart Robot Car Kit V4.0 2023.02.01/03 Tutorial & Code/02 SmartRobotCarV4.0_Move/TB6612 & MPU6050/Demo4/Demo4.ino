#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"
#include "test.cpp"

enum Directions {
  North,
  East,
  South,
  West,
  Movement,
  Default
};

String grid[9] = {
".S.-.-.-.",
"| B | | |",
".-.-.-.-.",
"| BX| | |",
".-.-.-.-.",
"| | | | |",
".-.-.-.-.",
"| | | | |",
".-.-.-.-."};

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_ConquerorCarxxx0;
MPU6050_getdata AppMPU6050getdata;
int timer = 0;
ConquerorCarMotionControl status = stop_it;
Directions carDirections[V*2];

const float speed = 1300;

int src = 0;
int target = 0;
Directions startingDirection = East;

int rotations[4] = {0, 90, -180, -90};

void setup() {
  for (int i = 0; i < V*2; i++)
    carDirections[i] = Default;

  Serial.begin(9600);

	int graph[V][V] = { 
            { 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, //Connects to 1 and 4
						{ 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, //Connects to 0, 5, and 2
						{ 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, //Connects to 1, 6, 3
						{ 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 }, //Connects to 2, 7
						{ 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 }, //Connects to 0, 5, and 8
						{ 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0 }, //Connects to 4, 9, 1, and 6
						{ 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0 }, //Connects to 2, 5, 10, and 7
						{ 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0 }, //Connects to 3, 6, and 11
						{ 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0 }, //Connects to 12, 9, and 4
						{ 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0 }, //Connects to 5, 8, 13, and 10
						{ 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0 }, //Connects to 9, 14, 11, and 6
						{ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1 }, //Connects to 10, 15, and 7
						{ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0 }, //Connects to 8 and 13
						{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0 }, //Connects to 12, 9, and 14
						{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1 }, //Connects to 13, 10, and 15
						{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0 } }; //Connects to 14 and 11

  for (int y = 1; y < 9; y += 2) {
    for (int x = 1; x < 9; x += 2) {
      char currentChar = grid[y][x];

      char upChar = grid[y - 1][x];
      char downChar = grid[y + 1][x];
      char leftChar = grid[y][x - 1];
      char rightChar = grid[y][x + 1];

      int realY = 0.5*((float) y)-0.5;
      int realX = 0.5*((float) x)-0.5;

      int place = (4 * realY) + realX;

      if ((int) upChar == 83) {
        src = place;
        startingDirection = South;
      } else if ((int) downChar == 83) {
        src = place;
        startingDirection = North;
      } else if ((int) leftChar == 83) {
        src = place;
        startingDirection = East;
      } else if ((int) rightChar == 83) {
        src = place;
        startingDirection = West;
      }

      if ((int) upChar == 66) {
        graph[place][place - 4] = 0;
      } else if ((int) downChar == 66) {
        graph[place][place + 4] = 0;
      } else if ((int) leftChar == 66) {
        graph[place][place - 1] = 0;
      } else if ((int) rightChar == 66) {
        graph[place][place + 1] = 0;
      }

      if ((int) currentChar == 88) {
        target = place;
      }
    }
  }

  switch (startingDirection) {
    case South:
      rotations[0] = -180;
      rotations[1] = -90;
      rotations[2] = 0;
      rotations[3] = 90;
      break;
    case West:  
      rotations[0] = 90;
      rotations[1] = -180;
      rotations[2] = 0;
      rotations[3] = -90;
      break;
    case East:
      rotations[0] = -90;
      rotations[1] = 0;
      rotations[2] = 90;
      rotations[3] = -180;
      break;
    default:
      break;
  }

  AppMotor.DeviceDriverSet_Motor_Init();
  AppMPU6050getdata.MPU6050_dveInit();
  delay(2000);
  AppMPU6050getdata.MPU6050_calibration();

  dijkstra(graph, src);

  int lastCounter = 0;
  for (int i = 1; i < V; i++) {
    int currentNode = pathArray[target][i - 1];
    int nextNode = pathArray[target][i];

    if (nextNode == -1) break;

    Directions orientation;
    if (currentNode - nextNode == -4) {
      orientation = South;
    } else if (currentNode - nextNode == 4) {
      orientation = North;
    } else if (currentNode - nextNode == -1) {
      orientation = East;
    } else if (currentNode - nextNode == 1) {
      orientation = West;
    }

    carDirections[lastCounter] = orientation;

    lastCounter++;
    startingDirection = orientation;

    carDirections[lastCounter] = Movement;
    lastCounter++;
  }

  for (int i = 0; i < V*2; i++) {
    if (carDirections[i] != Default) Serial.println(carDirections[i]);
  }
}

void turn(Directions direction) {
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);

  int desiredYaw = 0;
  desiredYaw = rotations[(int) direction];

  while (abs(Yaw - desiredYaw) > 3) {
    if (desiredYaw > 0) {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ 75,
                                             /*direction_B*/ direction_just, /*speed_B*/ 75, /*controlED*/ control_enable); //Motor control
    } else if (desiredYaw < 0) {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ 75,
                                             /*direction_B*/ direction_back, /*speed_B*/ 75, /*controlED*/ control_enable); //Motor control
    }
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
  }

  AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                         /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
}

int currentTime = 0;

int formerCounter = -1;
int counter = 0;
bool finished = false;
void loop() {
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);

  ApplicationFunctionSet_ConquerorCarMotionControl(status, 250);

  if (finished) {
    delay(1000);
    finished = false;
  }

  if (carDirections[counter] == Default) return;

  timer = millis();

  if (counter != formerCounter) {
    formerCounter = counter;

    Directions direction = carDirections[counter];
    switch (direction) {
      case North:
        turn(North);
        break;
      case South:
        turn(South);
        break;
      case West:
        turn(West);
        break;
      case East:
        turn(East);
        break;
      case Movement:
        status = Forward;
        break;
      default:
        status = stop_it;
        break;
    }

    currentTime = millis();
  }

  if (abs(timer - currentTime) > speed/2.2) {
    status = stop_it;
    counter++;
    finished = true;
  }
}
