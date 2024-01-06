#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"
#include "test.cpp"

String grid[9] = {
".-.S.-.-.",
"| | | | |",
".B.-.-.-.",
"| B | B |",
".-.B.-.-.",
"| B | B |",
".-.-.-.-.",
"|X| | B |",
".-.-.-.-."};

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_ConquerorCarxxx0;
MPU6050_getdata AppMPU6050getdata;
int timer = 0;
ConquerorCarMotionControl status = stop_it;
Directions carDirections[V*2];

float getTimeForDistance(float distance) {
  return distance/0.079;
}

int src = 0;
int target = 0;
Directions startingDirection = East;

int graph[V][V] = { 
            { 0, 2, 0, 0, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, //Connects to 1, 4, and 5
						{ 2, 0, 2, 0, 3, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, //Connects to 0, 5, 4, 6 and 2
						{ 0, 2, 0, 2, 0, 3, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0 }, //Connects to 1, 6, 3, 5, and 7
						{ 0, 0, 2, 0, 0, 0, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0 }, //Connects to 2, 7, and 6
						{ 2, 3, 0, 0, 0, 2, 0, 0, 2, 3, 0, 0, 0, 0, 0, 0 }, //Connects to 0, 5, 1, 9 and 8
						{ 3, 2, 3, 0, 2, 0, 2, 0, 3, 2, 3, 0, 0, 0, 0, 0 }, //Connects to 4, 9, 1, 0, 2, 8, 10 and 6
						{ 0, 3, 2, 3, 0, 2, 0, 2, 0, 3, 2, 3, 0, 0, 0, 0 }, //Connects to 2, 5, 10, 1, 3, 9, 11 and 7
						{ 0, 0, 3, 2, 0, 0, 2, 0, 0, 0, 3, 2, 0, 0, 0, 0 }, //Connects to 3, 6, 2, 10 and 11
						{ 0, 0, 0, 0, 2, 3, 0, 0, 0, 2, 0, 0, 2, 3, 0, 0 }, //Connects to 12, 9, 5, 13 and 4
						{ 0, 0, 0, 0, 3, 2, 3, 0, 2, 0, 2, 0, 3, 2, 3, 0 }, //Connects to 5, 8, 14, 4, 6, 12, 13 and 10
						{ 0, 0, 0, 0, 0, 3, 2, 3, 0, 2, 0, 2, 0, 3, 2, 3 }, //Connects to 9, 14, 11, 5, 7, 13, 15 and 6
						{ 0, 0, 0, 0, 0, 0, 3, 2, 0, 0, 2, 0, 0, 0, 3, 2 }, //Connects to 10, 15, 6, 14 and 7
						{ 0, 0, 0, 0, 0, 0, 0, 0, 2, 3, 0, 0, 0, 2, 0, 0 }, //Connects to 8, 9 and 13
						{ 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 3, 0, 2, 0, 2, 0 }, //Connects to 12, 9, 8, 10 and 14
						{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 3, 0, 2, 0, 2 }, //Connects to 13, 10, 9, 11 and 15
						{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 2, 0, 0, 2, 0 } }; //Connects to 14, 10 and 11

void setup() {
  for (int i = 0; i < V*2; i++)
    carDirections[i] = Default;

  Serial.begin(9600);

  for (int y = 1; y < 9; y += 2) {
    for (int x = 1; x < 9; x += 2) {

      char currentChar = grid[y][x];

      char upChar = grid[y - 1][x];
      char downChar = grid[y + 1][x];
      char leftChar = grid[y][x - 1];
      char rightChar = grid[y][x + 1];

      bool onLeftSide = x - 2 < 0;
      bool onRightSide = x + 2 >= 9;
      bool onTopSide = y - 2 < 0;
      bool onBottomSide = y + 2 >= 9;

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

        if (!onLeftSide) {
          graph[place - 4][place - 1] = 0;
        }

        if (!onRightSide) {
          graph[place - 4][place + 1] = 0;
        }
      } else if (!onTopSide) {
        int upY = y - 2;
        
        char topLeftChar = grid[upY][x - 1];
        char topRightChar = grid[upY][x + 1];

        if ((int) topLeftChar == 66) {
          if (!onLeftSide) {
            graph[place - 4][place - 1] = 0;
          }
        }

        if ((int) topRightChar == 66) {
          if (!onRightSide) {
            graph[place - 4][place + 1] = 0;
          }
        }
      }
      if ((int) downChar == 66) {
        graph[place][place + 4] = 0;

        if (!onLeftSide) {
          graph[place + 4][place - 1] = 0;
        }

        if (!onRightSide) {
          graph[place + 4][place + 1] = 0;
        }
      } else if (!onBottomSide) {
        int downY = y + 2;
        
        char downLeftChar = grid[downY][x - 1];
        char downRightChar = grid[downY][x + 1];

        if ((int) downLeftChar == 66) {
          if (!onLeftSide) {
            graph[place + 4][place - 1] = 0;
          }
        }

        if ((int) downRightChar == 66) {
          if (!onRightSide) {
            graph[place + 4][place + 1] = 0;
          }
        }
      }
      if ((int) leftChar == 66) {
        graph[place][place - 1] = 0;

        if (!onBottomSide) {
          graph[place - 4][place - 1] = 0;
        }

        if (!onTopSide) {
          graph[place + 4][place - 1] = 0;
        }
      } else if (!onLeftSide) {
        int leftX = x - 2;
        
        char leftUpChar = grid[y - 1][leftX];
        char leftDownChar = grid[y + 1][leftX];

        if ((int) leftUpChar == 66) {
          if (!onTopSide) {
            graph[place + 4][place - 1] = 0;
          }
        }

        if ((int) leftDownChar == 66) {
          if (!onBottomSide) {
            graph[place - 4][place - 1] = 0;
          }
        }
      }
      if ((int) rightChar == 66) {
        graph[place][place + 1] = 0;

        if (!onBottomSide) {
          graph[place - 4][place + 1] = 0;
        }

        if (!onTopSide) {
          graph[place + 4][place + 1] = 0;
        }
      } else if (!onRightSide) {
        int rightX = x + 2;
        
        char rightUpChar = grid[y - 1][rightX];
        char rightDownChar = grid[y + 1][rightX];

        if ((int) rightUpChar == 66) {
          if (!onTopSide) {
            graph[place + 4][place + 1] = 0;
          }
        }

        if ((int) rightDownChar == 66) {
          if (!onBottomSide) {
            graph[place - 4][place + 1] = 0;
          }
        }
      }

      if ((int) currentChar == 88) {
        target = place;
      }
    }
  }

  currentDirection = startingDirection;

  switch (startingDirection) {
    case South:
      rotations[0] = -180;
      rotations[1] = -90;
      rotations[2] = 0;
      rotations[3] = 90;
      rotations[4] = -135;
      rotations[5] = -45;
      rotations[6] = 45;
      rotations[7] = 135;
      break;
    case West:  
      rotations[0] = 90;
      rotations[1] = -180;
      rotations[2] = -90;
      rotations[3] = 0;
      rotations[4] = 135;
      rotations[5] = -135;
      rotations[6] = -45;
      rotations[7] = 45;
      break;
    case East:
      rotations[0] = -90;
      rotations[1] = 0;
      rotations[2] = 90;
      rotations[3] = -180;
      rotations[4] = -45;
      rotations[5] = 45;
      rotations[6] = 135;
      rotations[7] = -135;
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

    Serial.println(nextNode);

    if (nextNode == -1) break;

    int difference = currentNode - nextNode;

    Directions orientation;
    if (difference == -4) {
      orientation = South;
    } else if (difference == 4) {
      orientation = North;
    } else if (difference == -1) {
      orientation = East;
    } else if (difference == 1) {
      orientation = West;
    } else if (difference == 3) {
      orientation = Northeast;
    } else if (difference == -3) {
      orientation = Southwest;
    } else if (difference == 5) {
      orientation = Northwest;
    } else if (difference == -5) {
      orientation = Southeast;
    }

    if (orientation != startingDirection) {
      carDirections[lastCounter] = orientation;

      lastCounter++;
      startingDirection = orientation;
    }

    carDirections[lastCounter] = Movement;
    lastCounter++;
  }

  Serial.println();

  for (int i = 0; i < V*2; i++) {
    if (carDirections[i] != Default) Serial.println(carDirections[i]);
  }
}

void turn(Directions direction) {
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);

  int desiredYaw = 0;
  desiredYaw = rotations[(int) direction];

  if (desiredYaw == -180) {
    if (abs(180 - Yaw) < abs(-180 - Yaw)) desiredYaw = 180;
  }

  Serial.println(desiredYaw);
  Serial.println(Yaw);

  bool turnDirection = Yaw < desiredYaw;
  while (abs(Yaw - desiredYaw) > 3) {
    if (turnDirection) {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ 75,
                                             /*direction_B*/ direction_just, /*speed_B*/ 75, /*controlED*/ control_enable); //Motor control
      
      Serial.println(Yaw);
    } else if (!turnDirection) {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ 75,
                                             /*direction_B*/ direction_back, /*speed_B*/ 75, /*controlED*/ control_enable); //Motor control
      Serial.println(Yaw);
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
  ApplicationFunctionSet_ConquerorCarMotionControl(status, 250);

  if (finished) {
    finished = false;
  }

  if (carDirections[counter] == Default) return;

  timer = millis();

  if (counter != formerCounter) {
    formerCounter = counter;

    Directions direction = carDirections[counter];
    switch (direction) {
      case Movement:
        status = Forward;
        break;
      case Default:
        status = stop_it;
        break;
      default:
        turn(direction);
        currentDirection = direction;
        break;
    }

    currentTime = millis();
  }

  float distance = 0;
  if (status == Forward && (currentDirection == Northeast || currentDirection == Southeast || currentDirection == Southwest || currentDirection == Northwest)) {
    distance = 70.7106781187;
  } else if (status == Forward) {
    distance = 50;
  }

  if (abs(timer - currentTime) > getTimeForDistance(distance)) {
    status = stop_it;
    counter++;
    finished = true;
  }
}
