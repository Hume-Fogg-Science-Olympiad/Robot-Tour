#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"
#include "test.cpp"

//https://www.youtube.com/watch?v=LrsTBWf6Wsc Helpful youtube video about odometry (how to get position and orientation of a robot based on simple values)

const char string_0[] PROGMEM = ".-.-.-.-.";
const char string_1[] PROGMEM = "|G| BX| S";
const char string_2[] PROGMEM = ".-.-.-.B.";
const char string_3[] PROGMEM = "| | B | |";
const char string_4[] PROGMEM = ".B.-.B.-.";
const char string_5[] PROGMEM = "|GB | | |";
const char string_6[] PROGMEM = ".-.-.-.B.";
const char string_7[] PROGMEM = "|G| B |G|";
const char string_8[] PROGMEM = ".-.-.-.-.";

const char *const grid[] PROGMEM = {string_0, string_1, string_2, string_3, string_4, string_5, string_6, string_7, string_8};

DeviceDriverSet_Motor AppMotor;
Application_xxx Application_ConquerorCarxxx0;
MPU6050_getdata AppMPU6050getdata;
int timer = 0;
ConquerorCarMotionControl status = stop_it;
Directions* carDirections = (Directions*)malloc((V*5) * sizeof(int));

int src = 0;
int target = 0;
int* gates = (int*)malloc((4) * sizeof(int));
Directions startingDirection = East;

int (*graph)[V] = malloc(sizeof(int[V][V]));
char buffer[0];
bool onBottomSide;
bool onTopSide;
char currentChar;
char leftChar;
char rightChar;
char downChar;
char upChar;
int place;
bool onLeftSide;
bool onRightSide;

int currentTime = 0;

int formerCounter = -1;
int counter = 0;
bool finished = false;

float speed = 125;
float targetTime = 75;
float delayTime = 0;
bool delayBool = false;

DeviceDriverSet_ULTRASONIC myUltrasonic;
int ultraSonicDistance = 0;

int startingDistance = 0;
int previousDistance = 0;

int useUltrasonic = 0;

int useOtherUltrasonic = false;

float getTimeForDistance(float distance) {
  float slope;
  if (speed == 150) {
    slope = 0.0278679;
  } else if (speed == 70) {
    slope = 0.0164571;
  } else if (speed < 60) {
    slope = 0.00027*(speed);
  } else if (speed < 100) {
    slope = 0.00026*(speed);
  } else {
    slope = 0.000316*(speed);
  }
  return distance/slope;
}

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < V; i++) 
    for (int j = 0; j < V; j++)
      graph[i][j] = 0;

  for (int i = 0; i < 32; i++)
    pathArray[i] = -1;

  for (int i = 0; i < 4; i++) 
    gates[i] = -1;

  for (int y = 1; y < 9; y += 2) {
    for (int x = 1; x < 9; x += 2) {

      strcpy_P(buffer, (char *)pgm_read_ptr(&(grid[y])));  // Necessary casts and dereferencing, just copy.
      currentChar = buffer[x];


      leftChar = buffer[x - 1];
      rightChar = buffer[x + 1];

      strcpy_P(buffer, (char *)pgm_read_ptr(&(grid[y + 1])));  // Necessary casts and dereferencing, just copy.
      downChar = buffer[x];

      strcpy_P(buffer, (char *)pgm_read_ptr(&(grid[y - 1])));  // Necessary casts and dereferencing, just copy.
      upChar = buffer[x];

      onLeftSide = x - 2 < 0;
      onRightSide = x + 2 >= 9;
      onTopSide = y - 2 < 0;
      onBottomSide = y + 2 >= 9;

      place = (4 * (0.5*((float) y)-0.5)) + (0.5*((float) x)-0.5);

      if (!onLeftSide) graph[place][place - 1] = 2;
      if (!onRightSide) graph[place][place + 1] = 2;
      if (!onTopSide) graph[place][place - 4] = 2;
      if (!onBottomSide) graph[place][place + 4] = 2;


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
          graph[place][place - 4 - 1] = 0;
        }

        if (!onRightSide) {
          graph[place][place - 4 + 1] = 0;
        }
      } else if (!onTopSide) {
        strcpy_P(buffer, (char *)pgm_read_ptr(&(grid[y - 2])));  // Necessary casts and dereferencing, just copy.
        char topLeftChar = buffer[x - 1];
        char topRightChar = buffer[x + 1];

        if ((int) topLeftChar == 66) {
          if (!onLeftSide) {
            graph[place][place - 4 - 1] = 0;
          }
        }

        if ((int) topRightChar == 66) {
          if (!onRightSide) {
            graph[place][place - 4 + 1] = 0;
          }
        }
      }

      if ((int) downChar == 66) {
        graph[place][place + 4] = 0;

        if (!onLeftSide) {
          graph[place][place + 4 - 1] = 0;
        }

        if (!onRightSide) {
          graph[place][place + 4 + 1] = 0;
        }
      } else if (!onBottomSide) {
        strcpy_P(buffer, (char *)pgm_read_ptr(&(grid[y + 2])));  // Necessary casts and dereferencing, just copy.
        char downLeftChar = buffer[x - 1];
        char downRightChar = buffer[x + 1];

        if ((int) downLeftChar == 66) {
          if (!onLeftSide) {
            graph[place][place + 4 - 1] = 0;
          }
        }

        if ((int) downRightChar == 66) {
          if (!onRightSide) {
            graph[place][place + 4 + 1] = 0;
          }
        }
      }
      if ((int) leftChar == 66) {
        graph[place][place - 1] = 0;

        if (!onBottomSide) {
          graph[place][place - 1 + 4] = 0;
        }

        if (!onTopSide) {
          graph[place][place - 1 - 4] = 0;
        }
      } else if (!onLeftSide) {
        strcpy_P(buffer, (char *)pgm_read_ptr(&(grid[y - 1])));  // Necessary casts and dereferencing, just copy.
        char leftUpChar = buffer[x - 2];

        strcpy_P(buffer, (char *)pgm_read_ptr(&(grid[y + 1])));  // Necessary casts and dereferencing, just copy.
        char leftDownChar = buffer[x - 2];

        if ((int) leftUpChar == 66) {
          if (!onTopSide) {
            graph[place][place - 1 - 4] = 0;
          }
        }

        if ((int) leftDownChar == 66) {
          if (!onBottomSide) {
            graph[place][place - 1 + 4] = 0;
          }
        }
      }
      if ((int) rightChar == 66) {
        graph[place][place + 1] = 0;

        if (!onBottomSide) {
          graph[place][place + 1 - 4] = 0;
        }

        if (!onTopSide) {
          graph[place][place + 1 + 4] = 0;
        }
      } else if (!onRightSide) {
        strcpy_P(buffer, (char *)pgm_read_ptr(&(grid[y - 1])));  // Necessary casts and dereferencing, just copy.
        char rightUpChar = buffer[x + 2];

        strcpy_P(buffer, (char *)pgm_read_ptr(&(grid[y + 1])));  // Necessary casts and dereferencing, just copy.
        char rightDownChar = buffer[x + 2];

        if ((int) rightUpChar == 66) {
          if (!onTopSide) {
            graph[place][place + 1 - 4] = 0;
          }
        }

        if ((int) rightDownChar == 66) {
          if (!onBottomSide) {
            graph[place][place + 1 + 4] = 0;
          }
        }
      }

      if ((int) currentChar == 88) {
        target = place;
      } else if ((int) currentChar == 71) {
        for (int j = 0; j < 4; j++) {
          if (gates[j] == -1) {
            gates[j] = place; 
            break;
          }
        }
      }
    }
  }

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

      currentDirection = South;
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

      currentDirection = West;
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

      currentDirection = East;
      break;
    default:
      currentDirection = North;
      break;
  }

  myUltrasonic.DeviceDriverSet_ULTRASONIC_Init();
  AppMotor.DeviceDriverSet_Motor_Init();
  AppMPU6050getdata.MPU6050_dveInit();
  delay(2000);
  AppMPU6050getdata.MPU6050_calibration();
  myUltrasonic.DeviceDriverSet_ULTRASONIC_Get(&ultraSonicDistance);

  dijkstra(graph, src);

  int lowest = -1;
  int currentCounter = 0;
  int lowestIndex = 0;
  for (int i = 0; i < 4; i++) {
    int currentGate = gates[i];

    if (lowest = -1) {
      lowest = currentGate;
      lowestIndex = i;
    } else if (dist[lowest] > dist[currentGate]) {
      lowest = currentGate;
      lowestIndex = i;
    }
  }
  gates[lowestIndex] = -1;

  for (int j = 0; j < V; j++) {
    if (tempPathArray[lowest][j] == -1) break;

    pathArray[currentCounter] = tempPathArray[lowest][j];
    currentCounter++;
  }

  dijkstra(graph, lowest);

  lowest = -1;
  for (int i = 0; i < 4; i++) {
    int currentGate = gates[i];

    if (currentGate == -1) continue;

    if (lowest = -1) {
      lowest = currentGate;
      lowestIndex = i;
    } else if (dist[lowest] > dist[currentGate]) {
      lowest = currentGate;
      lowestIndex = i;
    }
  }

  gates[lowestIndex] = -1;

  for (int j = 0; j < V; j++) {
    if (tempPathArray[lowest][j] == -1) break;

    pathArray[currentCounter] = tempPathArray[lowest][j];
    currentCounter++;
  }

  dijkstra(graph, lowest);

  lowest = -1;
  for (int i = 0; i < 4; i++) {
    int currentGate = gates[i];

    if (currentGate == -1) continue;

    if (lowest = -1) {
      lowest = currentGate;
      lowestIndex = i;
    } else if (dist[lowest] > dist[currentGate]) {
      lowest = currentGate;
      lowestIndex = i;
    }
  }

  gates[lowestIndex] = -1;

  for (int j = 0; j < V; j++) {
    if (tempPathArray[lowest][j] == -1) break;

    pathArray[currentCounter] = tempPathArray[lowest][j];
    currentCounter++;
  }

  dijkstra(graph, lowest);

  for (int i = 0; i < 4; i++) {
    int currentGate = gates[i];
    if (currentGate != -1) {
      lowest = currentGate;
      break;
    } 
  }

  for (int j = 0; j < V; j++) {
    if (tempPathArray[lowest][j] == -1) break;

    pathArray[currentCounter] = tempPathArray[lowest][j];
    currentCounter++;
  }

  dijkstra(graph, lowest);

  for (int j = 0; j < V; j++) {
    if (tempPathArray[target][j] == -1) break;

    pathArray[currentCounter] = tempPathArray[target][j];
    currentCounter++;
  }  

  for (int i = 0; i < V*4; i++) {
    if (i == 0) {
      carDirections[i] = Movement;
    } else carDirections[i] = Default;
  }

  free(gates);

  int ultrasonicMovement = 0;
  int lastCounter = 1;
  int tempDirection = startingDirection;
  int totalRotations = 0;
  for (int i = 1; i < V*4; i++) {
    int currentNode = pathArray[i - 1];
    int nextNode = pathArray[i];

    if (nextNode > 15 || nextNode < 0) break;

    int difference = currentNode - nextNode;

    Directions orientation;
    if (difference == -4) {
      orientation = South;
      int tempNode = nextNode;

      int ultrasonicCounter = 1;
      while (tempNode + 4 <= 15) {
        if (graph[tempNode][tempNode + 4] == 0) {
          ultrasonicMovement = ultrasonicCounter;
          break;
        }
        ultrasonicCounter++;
        tempNode += 4;
      }
    } else if (difference == 4) {
      orientation = North;
      int tempNode = nextNode;

      int ultrasonicCounter = 1;
      while (tempNode - 4 >= 0) {
        if (graph[tempNode][tempNode - 4] == 0) {
          ultrasonicMovement = ultrasonicCounter;
          break;
        }
        ultrasonicCounter++;
        tempNode -= 4;
      }
    } else if (difference == -1) {
      orientation = East;
      int tempNode = nextNode;

      int ultrasonicCounter = 1;
      while ((tempNode - 3) % 4 != 0) {
        if (graph[tempNode][tempNode + 1] == 0) {
          ultrasonicMovement = ultrasonicCounter;
          break;
        }
        ultrasonicCounter++;
        tempNode++;
      }
    } else if (difference == 1) {
      orientation = West;
      int tempNode = nextNode;

      int ultrasonicCounter = 1;
      while (tempNode % 4 != 0 && tempNode != 0) {
        if (graph[tempNode][tempNode - 1] == 0) {
          ultrasonicMovement = ultrasonicCounter;
          break;
        }
        ultrasonicCounter++;
        tempNode--;
      }
    } else if (difference == 0) {
      continue;
    }

    if (orientation != tempDirection && abs(rotations[orientation] - rotations[tempDirection]) != 180) {
      if (abs(rotations[orientation]) == 90) {
        targetTime -= 1.0518;
      } else if (abs(rotations[orientation]) == 45) {
        targetTime -= 0.5327;
      } else if (abs(rotations[orientation]) == 135) {
        targetTime -= 1.4737;
      }

      carDirections[lastCounter] = orientation;

      lastCounter++;
      tempDirection = orientation;
      totalRotations++;
    }

    if (abs(rotations[orientation] - rotations[tempDirection]) == 180) {
      carDirections[lastCounter] = BackwardsMovement;
    } else if (ultrasonicMovement == 1) {
      carDirections[lastCounter] = OneUltrasonicMovement;
    } else if (ultrasonicMovement == 2) {
      carDirections[lastCounter] = TwoUltrasonicMovement;
    } else if (ultrasonicMovement == 3) {
      carDirections[lastCounter] = ThreeUltrasonicMovement;
    } else if (ultrasonicMovement == 4) {
      carDirections[lastCounter] = FourUltrasonicMovement;
    } else {
      carDirections[lastCounter] = Movement;
    }

    lastCounter++; 
    ultrasonicMovement = 0;
  }

  currentDirection = startingDirection;

  free(graph);
  free(pathArray);

  int totalMovement = 0;
  for (int i = 0; i < V*4; i++) {
    if (carDirections[i] == Default) break;

    Serial.println(carDirections[i]);

    if (carDirections[i] == Movement || carDirections[i] == BackwardsMovement) {
      totalMovement++;
    }
  }

  speed = 150;

  delayTime = (targetTime - (((48/0.0278679)*totalMovement)/1000))/(totalMovement+totalRotations) * 1000;
  if (delayTime < 0) {
    delayTime = 0;
  }

  counter = 0;
  formerCounter = -1;
  finished = false;
  delayBool = false;
  useOtherUltrasonic = 0;

  startingDistance = 0;
  previousDistance = 0;

  useUltrasonic = false;
}

void turn(Directions direction) {
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);

  int desiredYaw = 0;
  desiredYaw = rotations[(int) direction];

  if (desiredYaw == -180) {
    if (abs(180 - Yaw) < abs(-180 - Yaw)) {
      rotations[(int) direction] = 180;  
      desiredYaw = 180;
    }
  } else if (desiredYaw == 180) {
    if (abs(180 - Yaw) > abs(-180 - Yaw)) {
      rotations[(int) direction] = -180;  
      desiredYaw = -180;
    }
  }

  if (abs(180 - Yaw) < 5 || abs(-180 - Yaw) < 5) {
    if (abs(desiredYaw - -Yaw) < abs(desiredYaw - Yaw)) {
      AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw, true);
    }
  }

  bool turnDirection = Yaw < desiredYaw;
  while (abs(Yaw - desiredYaw) > 3) {
    if (turnDirection) { //Right
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ 100,
                                             /*direction_B*/ direction_just, /*speed_B*/ 100, /*controlED*/ control_enable); //Motor control
    } else if (!turnDirection) { //Left
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ 100,
                                             /*direction_B*/ direction_back, /*speed_B*/ 100, /*controlED*/ control_enable); //Motor control
    }
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
  }

  AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                         /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control

  delayBool = true;
  currentTime = millis();
}

void loop() {
  ApplicationFunctionSet_ConquerorCarMotionControl(status, 150);
  myUltrasonic.DeviceDriverSet_ULTRASONIC_Get(&ultraSonicDistance);

  if (abs(ultraSonicDistance - previousDistance) > 20 && previousDistance != 0) {
    ultraSonicDistance = previousDistance;
  } else {
    previousDistance = ultraSonicDistance;
  }

  if (finished) {
    finished = false;
  }

  if (carDirections[counter] == Default) return;

  timer = millis();

  if (delayBool) {
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    if (delayTime == 0) {
      previousDistance = 0;
      delayBool = false;
      counter++;
    } else if (abs(currentTime - timer) >= delayTime) {
      previousDistance = 0;
      counter++;
      delayBool = false;
    }
  }

  if (counter != formerCounter && !delayBool) {
    formerCounter = counter;
    Directions direction = carDirections[counter];
    switch (direction) {
      case Movement:

        startingDistance = ultraSonicDistance;
        previousDistance = startingDistance;

        if (startingDistance > 150) {
          useUltrasonic = false;
        } else useUltrasonic = true;

        status = Forward;
        break;
      case OneUltrasonicMovement:
        useOtherUltrasonic = 1;
        status = Forward;
        break;
      case TwoUltrasonicMovement:
        useOtherUltrasonic = 2;
        status = Forward;
        break;
      case ThreeUltrasonicMovement:
        useOtherUltrasonic = 3;
        status = Forward;
        break;
      case FourUltrasonicMovement:
        useOtherUltrasonic = 4;
        status = Forward;
        break;
      case BackwardsMovement:
        startingDistance = ultraSonicDistance;
        previousDistance = startingDistance;

        if (startingDistance > 150) {
          useUltrasonic = false;
        } else useUltrasonic = true;

        status = Backward;
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
  if (status == Forward) {
    if (useOtherUltrasonic == 0) {
      if (counter == 0) {
        distance = 40/1.25;
      } else {
        distance = 50/1.25;
      }
    } else {
      distance = 15 + (50 * (useOtherUltrasonic - 1));
    }
  } else if (status == Backward) {
    distance = 48/1.25;
  }

  if (useOtherUltrasonic != 0 && !delayBool) {
    if ((ultraSonicDistance < distance) && (abs(timer - currentTime) > 1000)) {
      status = stop_it;
      finished = true;
      delayBool = true;
      currentTime = millis(); 
      previousDistance = 0;
      useOtherUltrasonic = 0;
    }
  } else if (!delayBool) {
    if ((abs(timer - currentTime) > getTimeForDistance(distance) || (useUltrasonic && abs(startingDistance - ultraSonicDistance) > 45))) {
      status = stop_it;
      finished = true;
      delayBool = true;
      currentTime = millis();
      previousDistance = 0;
      useOtherUltrasonic = 0;
    }
  }
}
