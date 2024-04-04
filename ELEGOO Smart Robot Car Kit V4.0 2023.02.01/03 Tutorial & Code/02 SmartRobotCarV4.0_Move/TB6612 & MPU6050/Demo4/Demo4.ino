#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"
#include "test.cpp"

//https://www.youtube.com/watch?v=LrsTBWf6Wsc Helpful youtube video about odometry (how to get position and orientation of a robot based on simple values)

//Input values (Grid, target time, etc...)
const char string_0[] PROGMEM = ".-.-.-.-.";
const char string_1[] PROGMEM = "|G| BGBG|";
const char string_2[] PROGMEM = ".B.-.-.-.";
const char string_3[] PROGMEM = "| | | | |";
const char string_4[] PROGMEM = ".-.-.B.B.";
const char string_5[] PROGMEM = "| | | |X|";
const char string_6[] PROGMEM = ".-.-.-.-.";
const char string_7[] PROGMEM = "| BGB B S";
const char string_8[] PROGMEM = ".-.-.-.-.";

const char *const grid[] PROGMEM = {string_0, string_1, string_2, string_3, string_4, string_5, string_6, string_7, string_8};

float targetTime = 50;
bool useLongUltrasonic = false;

//Intialization of default values
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
float delayTime = 0;
bool delayBool = false;

DeviceDriverSet_ULTRASONIC myUltrasonic;

int ultraSonicDistance1 = 0;
int ultraSonicDistance2 = 0;

int startingDistance = 0;

int previousDistance1 = 0;
int previousDistance2 = 0;

int useUltrasonic = 0;
int useOtherUltrasonic = false;

float getTimeForDistance(float distance) {
  float slope;
  if (speed == 150) {
    slope = 0.0394048;
  }
  return distance/slope;
}

void setup() {
  Serial.begin(9600);

  //Intialization of default values
  {
    for (int i = 0; i < V; i++) 
      for (int j = 0; j < V; j++)
        graph[i][j] = 0;

    for (int i = 0; i < 48; i++) {
      pathArray[i] = -1;
    }

    for (int i = 0; i < 4; i++) { 
      gates[i] = -1;
    }
  }

  //Setup of the grid/gates/directions
  {
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
              if (gates[2] == 0) gates[2] = -1;

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
  }

  //Setup of the device drivers used (Motors, Ultrasonic, etc...)
  {
    myUltrasonic.DeviceDriverSet_ULTRASONIC_Init();
    AppMotor.DeviceDriverSet_Motor_Init();
    AppMPU6050getdata.MPU6050_dveInit();
    delay(2000);
    AppMPU6050getdata.MPU6050_calibration();
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    myUltrasonic.DeviceDriverSet_ULTRASONIC_1_Get(&ultraSonicDistance1);
    myUltrasonic.DeviceDriverSet_ULTRASONIC_2_Get(&ultraSonicDistance2);
  }

  int lowest = -1;
  int currentCounter = 0;
  int lowestIndex = 0;
  //Creation of the node-to-node path using djikstra's
  {
    dijkstra(graph, target);

    for (int i = 0; i < 4; i++) {
      int currentGate = gates[i];

      if (lowest == -1) {
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

      if (lowest == -1) {
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

      if (currentCounter != 0 && pathArray[currentCounter - 1] == tempPathArray[lowest][j]) continue;

      pathArray[currentCounter] = tempPathArray[lowest][j];
      currentCounter++;
    }  

    dijkstra(graph, lowest);

    lowest = -1;
    for (int i = 0; i < 4; i++) {
      int currentGate = gates[i];

      if (currentGate == -1) continue;

      if (lowest == -1) {
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

      if (currentCounter != 0 && pathArray[currentCounter - 1] == tempPathArray[lowest][j]) continue;

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

      if (currentCounter != 0 && pathArray[currentCounter - 1] == tempPathArray[lowest][j]) continue;

      pathArray[currentCounter] = tempPathArray[lowest][j];
      currentCounter++;
    }  

    dijkstra(graph, lowest);

    for (int j = 0; j < V; j++) {
      if (tempPathArray[src][j] == -1) break;

      if (currentCounter != 0 && pathArray[currentCounter - 1] == tempPathArray[src][j]) continue;

      pathArray[currentCounter] = tempPathArray[src][j];
      currentCounter++;
    }  

    for (int i = 0; i < V*4; i++) {
      if (i == 0) {
        carDirections[i] = Movement;
      } else carDirections[i] = Default;
    }

    free(gates);
  }

  int ultrasonicMovement = 0;
  int lastCounter = 1;
  int tempDirection = startingDirection;
  int totalRotations = 0;
  int ultrasonicCounter = 1;
  //Creation of the actual directions array that the robot can act on
  {
    for (int i = currentCounter - 1; i >= 0; i--) {
      int currentNode = pathArray[i];
      int nextNode = pathArray[i - 1];

      Serial.println(currentNode);

      if (nextNode > 15 || nextNode < 0) break;

      int difference = currentNode - nextNode;

      Directions orientation;
      if (difference == -4) {
        orientation = South;
        int tempNode = nextNode;

        if (!useLongUltrasonic) {
          if (tempNode + 4 <= 15) {
            if (graph[tempNode][tempNode + 4] == 0) {
              ultrasonicMovement = ultrasonicCounter;
            }
          }
        } else {
          while (tempNode + 4 <= 15) {
            if (graph[tempNode][tempNode + 4] == 0) {
              ultrasonicMovement = ultrasonicCounter;
              break;
            }

            ultrasonicCounter++;
            tempNode += 4;
          } 
        }
      } else if (difference == 4) {
        orientation = North;
        int tempNode = nextNode;
        
        if (!useLongUltrasonic) {
          if (tempNode - 4 >= 0) {
            if (graph[tempNode][tempNode - 4] == 0) {
              ultrasonicMovement = ultrasonicCounter;
            }
          }
        } else {
          while (tempNode - 4 >= 0) {
            if (graph[tempNode][tempNode - 4] == 0) {
              ultrasonicMovement = ultrasonicCounter;
              break;
            }

            ultrasonicCounter++;
            tempNode -= 4;
          } 
        }
      } else if (difference == -1) {
        orientation = East;
        int tempNode = nextNode;

        if (!useLongUltrasonic) {
          if ((tempNode - 3) % 4 != 0) {
            if (graph[tempNode][tempNode + 1] == 0) {
              ultrasonicMovement = ultrasonicCounter;
            }
          }
        } else {
          while ((tempNode - 3) % 4 != 0) {
            if (graph[tempNode][tempNode + 1] == 0) {
              ultrasonicMovement = ultrasonicCounter;
              break;
            }

            ultrasonicCounter++;
            tempNode += 1;
          } 
        }
      } else if (difference == 1) {
        orientation = West;
        int tempNode = nextNode;

        if (!useLongUltrasonic) {
          if (tempNode % 4 != 0 && tempNode != 0) {
            if (graph[tempNode][tempNode - 1] == 0) {
              ultrasonicMovement = ultrasonicCounter;
            }
          }
        } else {
          while (tempNode % 4 != 0 && tempNode != 0) {
            if (graph[tempNode][tempNode - 1] == 0) {
              ultrasonicMovement = ultrasonicCounter;
              break;
            }

            ultrasonicCounter++;
            tempNode -= 1;
          } 
        }
      } else if (difference == 0) {
        continue;
      }

      if (orientation != tempDirection && abs(rotations[orientation] - rotations[tempDirection]) != 180) {
        if (abs(abs(rotations[orientation]) - abs(rotations[tempDirection])) == 90) {
          targetTime -= 1.0274;
        }

        carDirections[lastCounter] = orientation;

        lastCounter++;
        tempDirection = orientation;
        totalRotations++;
      }

      if (abs(rotations[orientation] - rotations[tempDirection]) == 180) {
        if (ultrasonicMovement == 1) {
          carDirections[lastCounter] = OneBackwardsUltrasonicMovement;
        } else if (ultrasonicMovement == 2) {
          carDirections[lastCounter] = TwoBackwardsUltrasonicMovement;
        } else if (ultrasonicMovement == 3) {
          carDirections[lastCounter] = ThreeBackwardsUltrasonicMovement;
        } else carDirections[lastCounter] = BackwardsMovement;
      } else if (ultrasonicMovement == 1) {
        carDirections[lastCounter] = OneUltrasonicMovement;
      } else if (ultrasonicMovement == 2) {
        carDirections[lastCounter] = TwoUltrasonicMovement;
      } else if (ultrasonicMovement == 3) {
        carDirections[lastCounter] = ThreeUltrasonicMovement;
      } else {
        carDirections[lastCounter] = Movement;
      }

      lastCounter++; 
      ultrasonicMovement = 0;
      ultrasonicCounter = 1;
    }

    currentDirection = startingDirection;

    free(graph);
    free(pathArray);
  }

  Serial.println();

  //Calculation of the time needed to reach the target time
  {
    int totalMovement = 0;
    for (int i = 0; i < V*4; i++) {
      if (carDirections[i] == Default) break;

      Serial.print(i);
      Serial.print(" ");
      Serial.println(carDirections[i]);

      if (8 <= carDirections[i] && carDirections[i] <= 15) {
        totalMovement++;
      }
    }

    speed = 150;

    delayTime = (targetTime - (((50/0.0394048)*totalMovement)/1000))/(totalMovement - 1 + totalRotations) * 1000;
    if (delayTime < 0) {
      delayTime = 0;
    } else if (delayTime > 3000) {
      delayTime = 2500;
    }
  }

  //Double initialization of default values because Arduino is dumb
  {
    counter = 0;
    formerCounter = -1;
    finished = false;
    delayBool = false;
    useOtherUltrasonic = 0;

    startingDistance = 0;
    previousDistance1 = 0;
    previousDistance2 = 0;

    useUltrasonic = false;
  }
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
  while (abs(Yaw - desiredYaw) > 1) {
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

void freeTurn(float degrees) {
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);

  float desiredYaw = Yaw + degrees;

  bool turnDirection = Yaw < desiredYaw;
  while (abs(Yaw - desiredYaw) > 1) {
    if (turnDirection) { //Right
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ 75,
                                             /*direction_B*/ direction_just, /*speed_B*/ 75, /*controlED*/ control_enable); //Motor control
    } else if (!turnDirection) { //Left
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ 75,
                                             /*direction_B*/ direction_back, /*speed_B*/ 75, /*controlED*/ control_enable); //Motor control
    }
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
  }

  AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                         /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
}

void loop() {
  ApplicationFunctionSet_ConquerorCarMotionControl(status, 150);

  //Handling of Ultrasonic values
  {
    myUltrasonic.DeviceDriverSet_ULTRASONIC_1_Get(&ultraSonicDistance1);
    myUltrasonic.DeviceDriverSet_ULTRASONIC_2_Get(&ultraSonicDistance2);

    if (abs(ultraSonicDistance1 - previousDistance1) > 20 && previousDistance1 != 0) {
      ultraSonicDistance1 = previousDistance1;
    } else {
      previousDistance1 = ultraSonicDistance1;
    }

    if (abs(ultraSonicDistance2 - previousDistance2) > 20 && previousDistance2 != 0) {
      ultraSonicDistance2 = previousDistance2;
    } else {
      previousDistance2 = ultraSonicDistance2;
    }
  }

  if (carDirections[counter] == Default) return;

  timer = millis();

  //Handling of calibrating stops
  {
    if (delayBool) {
      AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
      if (delayTime == 0) {
        previousDistance1 = 0;
        delayBool = false;
        counter++;
      } else if (abs(currentTime - timer) >= delayTime) {
        previousDistance1 = 0;
        counter++;
        delayBool = false;
      }
    }
  }

  //Handling of each individual car direction
  {
    if (counter != formerCounter && !delayBool) {
      formerCounter = counter;
      Directions direction = carDirections[counter];
      switch (direction) {
        case Movement:

          previousDistance1 = ultraSonicDistance1;
          previousDistance2 = ultraSonicDistance2;

          status = Forward;
          break;
        case OneUltrasonicMovement:
          previousDistance1 = ultraSonicDistance1;
          previousDistance2 = ultraSonicDistance2;
          useOtherUltrasonic = 1;
          status = Forward;
          break;
        case TwoUltrasonicMovement:
          previousDistance1 = ultraSonicDistance1;
          previousDistance2 = ultraSonicDistance2;
          useOtherUltrasonic = 2;
          status = Forward;
          break;
        case ThreeUltrasonicMovement:
          previousDistance1 = ultraSonicDistance1;
          previousDistance2 = ultraSonicDistance2;
          useOtherUltrasonic = 3;
          status = Forward;
          break;
        case BackwardsMovement:
          previousDistance1 = ultraSonicDistance1;
          previousDistance2 = ultraSonicDistance2;

          status = Backward;
          break;
        case OneBackwardsUltrasonicMovement:
          previousDistance1 = ultraSonicDistance1;
          previousDistance2 = ultraSonicDistance2;
          useOtherUltrasonic = 1;
          status = Backward;
          break;
        case TwoBackwardsUltrasonicMovement:
          previousDistance1 = ultraSonicDistance1;
          previousDistance2 = ultraSonicDistance2;
          useOtherUltrasonic = 2;
          status = Backward;
          break;
        case ThreeBackwardsUltrasonicMovement:
          previousDistance1 = ultraSonicDistance1;
          previousDistance2 = ultraSonicDistance2;
          useOtherUltrasonic = 3;
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
  }

  float distance = 0;
  //Controls the distance depending on the instruction
  {
    if (status == Forward) {
      if (useOtherUltrasonic == 0) {
        if (counter == 0) {
          distance = 40;
        } else {
          distance = 50;
        }
      } else {
        distance = 15 + (30 * (useOtherUltrasonic - 1));
      }
    } else if (status == Backward) {
      if (useOtherUltrasonic == 0) {
        distance = 52;
      } else {
        distance = 12 + (50 * (useOtherUltrasonic - 1));
      }
    }
  }

  //Instructs the robot when to stop
  {
    if (useOtherUltrasonic != 0 && !delayBool) {
      if (status == Forward && (ultraSonicDistance1 < distance) && (abs(timer - currentTime) > 1000)) {
        status = stop_it;
        finished = true;
        delayBool = true;
        currentTime = millis(); 
        previousDistance1 = 0;
        useOtherUltrasonic = 0;
      } else if (status == Backward && (ultraSonicDistance2 < distance) && (abs(timer - currentTime) > 1000)) {
        status = stop_it;
        finished = true;
        delayBool = true;
        currentTime = millis(); 
        previousDistance1 = 0;
        useOtherUltrasonic = 0;
      }
    } else if (!delayBool) {
      if ((abs(timer - currentTime) > getTimeForDistance(distance))) {
        status = stop_it;
        finished = true;
        delayBool = true;
        currentTime = millis();
        previousDistance1 = 0;
        useOtherUltrasonic = 0;
      }
    }
  }
}
