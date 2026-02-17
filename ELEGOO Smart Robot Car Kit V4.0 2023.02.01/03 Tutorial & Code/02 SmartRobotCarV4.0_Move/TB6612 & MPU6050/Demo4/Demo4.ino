#include <avr/wdt.h>
#include <PID_v1.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.cpp"
#include "test.cpp"

//https://www.youtube.com/watch?v=LrsTBWf6Wsc Helpful youtube video about odometry (how to get position and orientation of a robot based on simple values)

//Input values (Grid, target time, etc...)
//.-.
//| | represents a single square of the grid, with up, left, right, and down boundaries
//.-.
//X is the target ending point
//G is a gate bonus zone
//S is where the robot will start
//L is the last gate bonus zone
//B are obstacles
const char string_0[] PROGMEM =  ".-.-.-.-."; // .-.-.-.-.
const char string_1[] PROGMEM =  "|G| | B |"; // | | | | |
const char string_2[] PROGMEM =  ".-.B.-.B."; // .-.-.-.-.
const char string_3[] PROGMEM =  "| | | |X|"; // | | | | |
const char string_4[] PROGMEM =  ".B.-.-.-."; // .-.-.-.-.
const char string_5[] PROGMEM =  "S B | BG|"; // | | | | |
const char string_6[] PROGMEM =  ".-.-.-.-."; // .-.-.-.-.
const char string_7[] PROGMEM =  "| | | | |"; // | | | | |
const char string_8[] PROGMEM =  ".-.B.-.B."; // .-.-.-.-.
const char string_9[] PROGMEM =  "| |LB |G|"; // | | | | |
const char string_10[] PROGMEM = ".-.-.-.-."; // .-.-.-.-.

const char *const grid[] PROGMEM = {string_0, string_1, string_2, string_3, string_4, string_5, string_6, string_7, string_8, string_9, string_10};

const byte pathArray[] = {8, 12, 16, 17, 16, 12, 13, 9, 5, 4, 0, 4, 5, 9, 10, 14, 18, 19, 18, 14, 15, 11, 7};
const int pathLength = 23;

float targetTime = 80;
//Experimental feature where the ultrasonic is used for distances over ~100 cm (doesn't work well)
bool useLongUltrasonic = false;

//Intialization of default values
DeviceDriverSet_Motor AppMotor;
Application_xxx Application_ConquerorCarxxx0;
MPU6050_getdata AppMPU6050getdata;

unsigned long timer = 0;
ConquerorCarMotionControl status = stop_it;
Directions* carDirections = (Directions*)malloc((V*6) * sizeof(byte));
int src = -1;
int target = -1;
int lastGate = -1;

byte* gates = (byte*)malloc((3) * sizeof(byte));
Directions startingDirection = East;
byte (*graph)[V] = malloc(sizeof(byte[V][V]));

//So many default values
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
unsigned long currentTime = 0;
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
int useOtherUltrasonic = 0;

unsigned long debounceTime = 0;

// Constant for steps in disk
float stepcount = 20.00;  // 20 Slots in disk, change if different

// Constant for wheel diameter
float wheeldiameter = 66.50; // Wheel diameter in millimeters, change if different


//Change the node-to-node array here
byte pathArray[] = {0, 1, 2, 3}; 
int pathLength = 4; //Change this to match the length of the path array

//Optical Interruptor Pins
byte MOTOR_FL = 18;
byte MOTOR_FR = 19;

// Integers for pulse counters
int counter_FL = 0;
int counter_FR = 0;

// Interrupt Service Routines

void ISR_countFL()  
{
  counter_FL++;
} 

void ISR_countFR()  
{
  counter_FR++;
}

//Extrapolation for how much a time a certain distance will take
//Doesnt work very well either (need to figure out encoders)
float getTimeForDistance(float distance) {

  return distance*21.80194 + 54.03797;
}

boolean firstTime = false;
void setup() {
  Serial.begin(9600);

  //Setup of directions
  {
    //Change this based on where the src is
    startingDirection = North;

    //Based on which direction you start on the src, set the correct relative angles
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
    AppMotor.DeviceDriverSet_Motor_Init();
    myUltrasonic.DeviceDriverSet_ULTRASONIC_Init();
    AppMPU6050getdata.MPU6050_dveInit();
    delay(2000);
    AppMPU6050getdata.MPU6050_calibration();
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    // Attach the Interrupts to their ISR's
    attachInterrupt(digitalPinToInterrupt (MOTOR_FL), ISR_countFL, RISING);
    attachInterrupt(digitalPinToInterrupt (MOTOR_FR), ISR_countFR, RISING);
  }

  carDirections[0] = Movement;
  int lastCounter = 1; //CHANGE THIS BACK TO 1 WHEN REVERTING BACK TO NORMAL CODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  int ultrasonicMovement = 0;
  int tempDirection = startingDirection;
  int totalRotations = 0;
  int ultrasonicCounter = 1;
  //Creation of the actual directions array that the robot can act on
  {
    for (int i = 0; i < pathLength - 1; i++) {
      int currentNode = pathArray[i];
      int nextNode = pathArray[i + 1];

      //If for some reason pathArray has invalid nodes
      if (nextNode > 19 || nextNode < 0) break;

      int difference = currentNode - nextNode;

      //Using difference, find if the robot needs to turn and if ultrasonic is usable
      Directions orientation;
      if (difference == -4) {
        orientation = South;
      } else if (difference == 4) {
        orientation = North;
      } else if (difference == -1) {
        orientation = East;
      } else if (difference == 1) {
        orientation = West;
      } else if (difference == 0) {
        continue;
      }

      //IDK really whats going on here but I think its decreasing the estimated amount of time a rotation takes from the timer
      if (orientation != tempDirection && abs(rotations[orientation] - rotations[tempDirection]) != 180) {
        if (abs(abs(rotations[orientation]) - abs(rotations[tempDirection])) == 90) {
          targetTime -= 1.0274;
        }

        carDirections[lastCounter] = orientation;

        lastCounter++;
        totalRotations++;
        tempDirection = orientation;
      }

      int tempNode = nextNode;
      switch (orientation) {
        case South:
          if (!useLongUltrasonic) {
            if (tempNode + 4 <= 19) {
              if (graph[tempNode][tempNode + 4] == 0) {
                ultrasonicMovement = ultrasonicCounter;
              }
            }
          } else {
            while (tempNode + 4 <= 19) {
              if (graph[tempNode][tempNode + 4] == 0) {
                ultrasonicMovement = ultrasonicCounter;
                break;
              }

              ultrasonicCounter++;
              tempNode += 4;
            } 
          }
          break;
        case North:
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
          break;
        case West:
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
          break;
        case East:
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
          break;
      }

      //Determines the type of movement forward that we should use (Ultrasonic/Long Ultrasonic/Normal)
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

      //Incrementing counter and resetting variables
      lastCounter++; 
      ultrasonicMovement = 0;
      ultrasonicCounter = 1;
    }

    currentDirection = startingDirection;

    carDirections[lastCounter] = Default;

    free(graph);
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

    delayTime = (targetTime - (((getTimeForDistance(50))*totalMovement)/1000))/(totalMovement - 1 + totalRotations) * 1000;
    if (delayTime < 0) {
      delayTime = 0;
    } else if (delayTime > 3000) { //If the delay time is over 3 seconds (will result in a penalty), bump it down to 2.5
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

    debounceTime = 0;

    stepcount = 20.00;  // 20 Slots in disk, change if different

    wheeldiameter = 66.50; // Wheel diameter in millimeters, change if different

    MOTOR_FL = 18;
    MOTOR_FR = 19;

    counter_FL = 0;
    counter_FR = 0;

    firstTime = false;
  }
}

// Function to convert from centimeters to steps
int CMtoSteps(float cm) {
  int result;  // Final calculation result
  float circumference = (wheeldiameter * 3.14) / 10; // Calculate wheel circumference in cm
  float cm_step = circumference / stepcount;  // CM per Step
  
  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)

  return result;  // End and return result
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

  double m_kP = 0.25;
  int lowerBound = 50;
  int upperBound = 100;

  //abs(Yaw - desiredYaw) > 0.3
  currentTime = millis();
  while (abs(Yaw - desiredYaw) > 0.3) {
    Serial.println(Yaw);

    int speed = lowerBound + abs((Yaw - desiredYaw) / m_kP);

    if (speed < lowerBound) {
      speed = lowerBound;
    } else if (speed > upperBound) {
      speed = upperBound;
    }

    turnDirection = Yaw < desiredYaw;

    if (turnDirection) { //Right
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    } else if (!turnDirection) { //Left
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                             /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
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

  //Recalculate distance shit with fully charged batteries because I fucking forgot to push

  //Handling of Ultrasonic values
  //Currently commented because it makes loop time super slow, which messes up encoder readings <- turns out this was b/c the encoders were unplugged, which makes it slow for some reason
  {
    myUltrasonic.DeviceDriverSet_ULTRASONIC_1_Get(&ultraSonicDistance1);
    myUltrasonic.DeviceDriverSet_ULTRASONIC_2_Get(&ultraSonicDistance2);

    // if (abs(ultraSonicDistance1 - previousDistance1) > 20 && previousDistance1 != 0) {
    //   ultraSonicDistance1 = previousDistance1;
    // } else {
    //   previousDistance1 = ultraSonicDistance1;
    // }

    // if (abs(ultraSonicDistance2 - previousDistance2) > 20 && previousDistance2 != 0) {
    //   ultraSonicDistance2 = previousDistance2;
    // } else {
    //   previousDistance2 = ultraSonicDistance2;
    // }
  }

  if (carDirections[counter] == Default) {
    Serial.println("finished");
    return;
  }

  timer = millis();

  //Handling of calibrating stops
  {
    if (delayBool) {

      AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
      if (delayTime == 0) {

        previousDistance1 = 0;
        delayBool = false;
        counter++;

        counter_FL = 0;
        counter_FR = 0;
      } else if (abs(currentTime - timer) >= delayTime) {

        previousDistance1 = 0;
        counter++;
        delayBool = false;

        counter_FL = 0;
        counter_FR = 0;
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
        case BackwardsMovement:
          status = Backward;
          break;
        case OneBackwardsUltrasonicMovement:
          useOtherUltrasonic = 1;
          status = Backward;
          break;
        case TwoBackwardsUltrasonicMovement:
          useOtherUltrasonic = 2;
          status = Backward;
          break;
        case ThreeBackwardsUltrasonicMovement:
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

      counter_FL = 0;
      counter_FR = 0;
      previousDistance1 = ultraSonicDistance1;
      previousDistance2 = ultraSonicDistance2;
      currentTime = millis();
    }
  }

  float distance = 0;
  //Controls the distance depending on the instruction
  {
    if (status == Forward) {
      if (useOtherUltrasonic == 0) {
        if (counter == 0) {
          //Change this to the correct value once actually running on the correct field
          // distance = 36.388;
          distance = 50;
        } else if (carDirections[counter + 1] == Default) {
          distance = 38.612;
        } else {
          distance = 50;
        }
      } else {
        distance = 20;
      }
    } else if (status == Backward) {
      if (useOtherUltrasonic == 0) {
        distance = 50;
      } else {
        distance = 20;
      }
    }
  }

  //Instructs the robot when to stop
  {
    if (useOtherUltrasonic != 0 && !delayBool) {
      if (status == Forward) {
        if (ultraSonicDistance1 < distance) {
          if (debounceTime == 0) {
            debounceTime = millis();
          } else if (abs(millis() - debounceTime) > 50) {
            status = stop_it;
            finished = true;
            delayBool = true;
            currentTime = millis(); 
            previousDistance1 = 0;
            useOtherUltrasonic = 0;
            debounceTime = 0;

            counter_FL = 0;
            counter_FR = 0;
          }
        } else debounceTime = 0;
      } else if (status == Backward && ultraSonicDistance2 < distance) {
        if (ultraSonicDistance2 < distance) {
          if (debounceTime == 0) {
            debounceTime = millis();
          } else if (abs(millis() - debounceTime) > 50) {
            status = stop_it;
            finished = true;
            delayBool = true;
            currentTime = millis(); 
            previousDistance1 = 0;
            useOtherUltrasonic = 0;
            debounceTime = 0;

            counter_FL = 0;
            counter_FR = 0;
          }
        } else debounceTime = 0;
      }
    } else if (!delayBool) {
      if (counter_FL > CMtoSteps(distance) && counter_FR > CMtoSteps(distance)) {
        status = stop_it;
        finished = true;
        delayBool = true;
        currentTime = millis();
        previousDistance1 = 0;
        useOtherUltrasonic = 0;

        counter_FL = 0;
        counter_FR = 0;
      }
    }
  }
}
