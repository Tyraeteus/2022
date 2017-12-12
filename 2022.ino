#include "Encoder.h"
#include "PID_v1.h"
#include "Wire.h"
#include "NewPing.h"
#include "BasicStepperDriver.h" 
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>

//Variables for heading calculation
//Written by Joe St. Germain for RBE 2002
float G_Dt=0.020;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpose timer
long timer1=0;  
long timer2=0;  

float G_gain=.00875; // gyros gain factor for 250deg/sec
float gyro_x; //gyro x val
float gyro_y; //gyro x val
float gyro_z; //gyro x val
float gyro_xold; //gyro cummulative x value
float gyro_yold; //gyro cummulative y value
float gyro_zold; //gyro cummulative z value
float gerrx; // Gyro x error
float gerry; // Gyro y error
float gerrz; // Gyro 7 error

float A_gain=.00875; // gyros gain factor for 250deg/sec
float accel_x; //gyro x val
float accel_y; //gyro x val
float accel_z; //gyro x val
float accel_xold; //gyro cummulative x value
float accel_yold; //gyro cummulative y value
float accel_zold; //gyro cummulative z value
float aerrx; // Accel x error
float aerry; // Accel y error
float aerrz; // Accel 7 error
 
//Front distance pid
//Input: Distance between front distance sensor and obstacle
//Output: Speed value (-1 to 1) for how quickly to approach the wall
double kpFront = 1;
double kiFront = 0;
double kdFront = .1;
double frontSetpoint = 5; //cm
double frontInput;
double frontOutput;
PID frontPID(&frontInput, &frontOutput, &frontSetpoint, kpFront, kiFront, kdFront, DIRECT);

//Side distance pid
//Input: distance between side distance sensor and wall
//Output: Turn value (-1 to 1) for how much to turn towards or away from the wall
double kpSide = 1;
double kiSide = 0;
double kdSide = .1;
double sideSetpoint = 5;  //cm
double sideInput;
double sideOutput;
PID sidePID(&sideInput, &sideOutput, &sideSetpoint, kpSide, kiSide, kdSide, DIRECT);

//Drive straight with encoders pid
//Input: desired difference between encoders (typically 0)
//Output: Turn value (-1 to 1) for how much to turn to drive straight
double kpEnc = 1;
double kiEnc = 0;
double kdEnc = .1;
double encSetpoint;
double encInput;
double encOutput;
PID encPID(&encInput, &encOutput, &encSetpoint, kpEnc, kiEnc, kdEnc, DIRECT);

//Drive distance pid
//Input: current distance logged by encoders
//Output: Speed value (-1 to 1) for the drive motors
double kpDist = 1;
double kiDist = 0;
double kdDist = .1;
double distSetpoint;
double distInput;
double distOutput;
PID distPID(&distInput, &distOutput, &distSetpoint, kpDist, kiDist, kdDist, DIRECT);

//Turn angle pid
//Input: current heading of robot
//Output: Turn value (-1 to 1) for drive motors
double kpAngle = 1;
double kiAngle = 0;
double kdAngle = .1;
double angleSetpoint;
double angleInput;
double angleOutput;
PID anglePID(&angleInput, &angleOutput, &angleSetpoint, kpAngle, kiAngle, kdAngle, DIRECT);

//Robot parameters
const int wheelCircumfrence = 8.639;  //Circumfrence of wheels, used to calulate distance travelled
const int maxDistanceReading = 35;  //Max distance reading of ultrasonic sensors, cm
const int flameThreshold = 50;  //Threshold to determine whether or not a flame has been spotted
const int stepperSteps = 1.8; //Degrees per step of stepper motor
const int stepperUpperLimit = 30; //Upper limit of stepper motor
const int stepperLowerLimit = -45;  //Lower limit of stepper motor
const int cliffThreshold = 0; //Threshold to determine whether the robot is on a cliff
const int countsPerRev = 4096;  //Encoder counts per revolution of wheel
const double stepInterval = 1;  //Interval to be used for each step when sweeping the stepper motor
const int stepperRPM = 300; //RPM to drive the stepper motor at
const int stepperMicrosteps = 1; //Number of microsteps for stepper motor

//Motor control ports
const int leftMotorPort = 0;
const int leftDriveFow = 0;
const int leftDriveRev = 0;
const int rightMotorPort = 0;
const int rightDriveFow = 0;
const int rightDriveRev = 0;
const int stepperStepPort = 0;
const int stepperDirPort = 0;
const int fanPort = 0;

//Analog input ports
const int flameSensorPort = 0;
const int wallFollowerSensorPort = 0;
const int frontDistanceSensorPort = 0;
const int rightCliffSensorPort = 0;
const int frontCliffSensorPort = 0;

//Digital I/O ports
const int leftEncPinA = 0;
const int leftEncPinB = 0;
const int rightEncPinA = 0;
const int rightEncPinB = 0;
const int frontTrig = 0;
const int frontEcho = 0;
const int sideTrig = 0;
const int sideEcho = 0;
const int startButton = 0;

//Control and state variables
long fanSteps = 0;  //Current position of the stepper in steps
double distanceToHome = 0;  //Distance from the virtual home (wall) to actual home
double robotXLoc = 0; //Current X-location of the robot
double robotYLoc = 0; //Current Y-location of the robot
int flamePasses = 0;  //Number of passes made by flame sensor when finding flame
int flameSensorMin = 1024;  //Variable to keep track of running minimum when finding flame
double flameAngleGuess = 0; //Varibale to keep track of current best guess of flame position
boolean isTiltingUp = false;  //Variable to keep track of the current direction of movement of the fan
double flameXLoc = 0; //X-location of the flame
double flameYLoc = 0; //Y-location of the flame
double flameZLoc = 0; //Z-location of the flame
boolean isAtWall = false; //Variable to keep track of if the robot has left true home yet
boolean hasSeenFlame = false; //Variable to keep track of whether or not the robot has seen the flame
boolean flameOut = false; //Variable to keep track of whether or not the flame is out
long prevDistanceReading = 0; //Variable to track the previous distance measurement used for odometry
boolean hasLeftHome = false;  //Flag for when the robot leaves true home
boolean isToHome = false; //Flag to keep track of whether or not the robot is headed home
boolean isFirstIteration = true; //Variable to run setup code in the first iteration of  state

//Enum to keep track of robot heading
typedef enum robotHeading {
      NORTH,
      EAST,
      SOUTH,
      WEST
    };

//Objects
robotHeading heading = EAST;

BasicStepperDriver stepper(stepperSteps, stepperDirPort, stepperStepPort);
    
Encoder leftEnc(leftEncPinA, leftEncPinB);
Encoder rightEnc(rightEncPinA, rightEncPinB);

NewPing frontSonar(frontTrig, frontEcho, maxDistanceReading);
NewPing sideSonar(sideTrig, sideEcho, maxDistanceReading);

L3G gyro;
LSM303 accel;

//States for robot state machine
enum robotState {
  FOLLOWING_WALL,
  TURNING_RIGHT_90,
  TURNING_LEFT_90,
  DRIVING_TO_WALL,
  FINDING_FLAME,
  EXTINGUISHING_FLAME,
  DRIVING_DISTANCE,
  STOPPED
}robotState;

void setup() {
  Serial.begin(9600);
  Wire.begin(); // i2c begin

  //Configure PID controllers
  frontPID.SetOutputLimits(-1,1);
  sidePID.SetOutputLimits(-1,1);
  encPID.SetOutputLimits(-1,1);
  distPID.SetOutputLimits(-1,1);
  anglePID.SetOutputLimits(-1,1);

  frontPID.SetMode(AUTOMATIC);
  sidePID.SetMode(AUTOMATIC);
  encPID.SetMode(AUTOMATIC);
  distPID.SetMode(AUTOMATIC);
  anglePID.SetMode(AUTOMATIC);

  //Initialize stepper
  stepper.begin(stepperRPM, stepperMicrosteps);

  //Initialize gyro
  //Written by Joe St. Germain for RBE 2002
  if (!gyro.init()){ // gyro init
    Serial.println("Failed to autodetect gyro type!");
    while (1); 
  }
  timer=millis(); // init timer for first reading
  gyro.enableDefault(); // gyro init. default 250/deg/s
  delay(1000);// allow time for gyro to settle
   Serial.println("starting calibration");
  gyroZero();
  Accel_Init();

  while(!startButton) {}  //Wait for start button to be pressed
}

void loop() {
  //Robot state machine
  switch(robotState) {
    //Following wall
    case FOLLOWING_WALL:
      //If we see the flame, set the flag and go to it
      if(seesFlame()) {
        robotState = TURNING_RIGHT_90;
        hasSeenFlame = true;
        isFirstIteration = true;
        //Otherwise, if we hit an obstacle (wall or cliff) turn right
      } else if(getFrontDistance() == frontSetpoint || isFrontCliff()) {
        robotState = TURNING_RIGHT_90;
        isFirstIteration = true;
        isAtWall = true;
        //Otherwise, if we're following a cliff, drive straight
      } else if(isRightCliff()) {
        driveToWall(1);
        isFirstIteration = true;
        //If we're "home," turn right to go to true home
      } else if(flameOut && robotXLoc == 0 && robotYLoc == 0) {
        robotState = TURNING_RIGHT_90;
        isToHome = true;
        isFirstIteration = true;
      } else {
        //Otherwise, continue to wall follow
        wallFollow(1);
      }
      //Sweep the fan up and down to look for the flame
      if(isTiltingUp) {
          if(fanSteps == stepperUpperLimit) {
            isTiltingUp = false;
          } else {
            moveStepperDegrees(3 * stepInterval); 
          }
        } else {
          if(fanSteps == stepperLowerLimit) {
            isTiltingUp = true;
          } else {
            moveStepperDegrees(-3 * stepInterval); 
          }
      break;
    //Turning 90 degrees right (CW)
    case TURNING_RIGHT_90:
      if(isFirstIteration) {
        //Set desired angle to the current angle plus 90 degrees
        angleSetpoint = getGyroAngle() + 90;
        isFirstIteration = false;
      }
      //If we haven't turned far enough, turn
      if(getGyroAngle() != angleSetpoint) {
        drive(turn90Speed(), turn90Speed()/abs(turn90Speed()));
      //Otherwise, update heading and advance to next state
      } else {
        if(heading == NORTH) {
          heading = EAST;
        } else if(heading == EAST){
          heading = SOUTH;
        } else if(heading == SOUTH) {
          heading = WEST;
        } else {
          heading = NORTH;
        }
        if(hasSeenFlame && !flameOut) {
          robotState = DRIVING_TO_WALL;
        } else if(isToHome) {
          robotState = DRIVING_DISTANCE;
          isFirstIteration = true;
        }
      }
      break;
    //Turning 90 degrees left (CCW)
    case TURNING_LEFT_90:
      if(isFirstIteration) {
        //Set the desired angle to the current angle minus 90 degrees
        angleSetpoint = getGyroAngle() - 90;
        isFirstIteration = false;
      }
      //If we haven't turned far enough, turn
      if(getGyroAngle() != angleSetpoint) {
        drive(turn90Speed(), turn90Speed()/abs(turn90Speed());
      //Otherwise, update heading and advance to next state
      } else {
        if(heading == NORTH) {
          heading = WEST;
        } else if(heading == WEST){
          heading = SOUTH;
        } else if(heading == SOUTH) {
          heading = EAST;
        } else {
          heading = NORTH;
        }
        if(flameOut) {
          robotState = DRIVING_TO_WALL;
        } else {
          robotState = FINDING_FLAME;
        }
      }
      break;
    //Driving to wall or candle
    case DRIVING_TO_WALL:
      if(isFirstIteration) {
        if(!hasLeftHome) {
          //If we are leaving home, log the distance before going
          distanceToHome = getFrontDistance();
        }
        hasLeftHome = true;
        isFirstIteration = false;
      }
      //If we aren't at an obstacle, drive forwards
      if(getFrontDistance() > frontSetpoint) {
        driveToWall(1);
      //Otherwise, advance to the next state
      } else {
        if(hasSeenFlame && !flameOut) {
          robotState = TURNING_LEFT_90;
          isFirstIteration = true;
        } else {
          robotState = TURNING_RIGHT_90;
          isFirstIteration = true;
        }
      }
      break;
    //Finding exact location of the flame
    case FINDING_FLAME:
      //Perform 6 passes
      if(flamePasses > 6) {
        if(isTiltingUp) {
          if(fanSteps == stepperUpperLimit) {
            isTiltingUp = false;
            flamePasses++;
          } else {
            moveStepperDegrees(stepInterval); 
          }
        } else {
          if(fanSteps == stepperLowerLimit) {
            isTiltingUp = true;
            flamePasses++;
          } else {
            moveStepperDegrees(-stepInterval); 
          }
        }
        //If we find a new lo (a more certain flame location), log the reading and the position
        if(readFlameSensor() < flameSensorMin) {
            flameSensorMin = readFlameSensor();
            flameAngleGuess = fanSteps;
        }
      } else {
        //Once done, calculate the flame location
        flameZLoc = getFlameZ();
        //Add x or Y offset based off of heading
        if(heading == EAST) {
          flameXLoc = getXLocation() + frontSetpoint;
          flameYLoc = getYLocation();
        } else if(heading == SOUTH) {
          flameXLoc = getXLocation();
          flameYLoc = getYLocation() - frontSetpoint;
        } else if(heading = WEST) {
          flameXLoc = getXLocation() - frontSetpoint;
          flameYLoc = getYLocation();
        } else {
          flameXLoc = getXLocation();
          flameYLoc = getYLocation() + frontSetpoint;
        }
        robotState = EXTINGUISHING_FLAME;
        isFirstIteration = true;
      }
      break;
  //Entinguishing flame
  case EXTINGUISHING_FLAME:
    if(isFirstIteration) {
      //Move the fan to the best guess angle found earlier
      double stepValue = getFanAngle() - flameAngleGuess;
      double stepDir = stepValue / abs(stepValue);
      moveStepperDegrees(stepValue);
      //Start the fan
      startFan();
      isFirstIteration = false;
    }
    ///If we don't see the flame, it must be out
    if(!seesFlame()) {
      flameOut = true;
      stopFan();
      robotState = TURNING_LEFT_90;
      isFirstIteration = true;
    } 
    break;
  //Driving a set distance
  case DRIVING_DISTANCE:
    if(isFirstIteration) {
      //Reset encoders to reduce error
      resetEncoders();
      isFirstIteration = false;
    }
    //If we have reached home, stop
    if(getEncDistance() == distanceToHome) {
      robotState = STOPPED;
    //Otherwise, keep driving
    } else {
      driveDistance(distanceToHome, 1);
    }
    break;
  //Stopped
  case STOPPED:
    //Stop the robot
    stopRobot();
    break;
  }
  //Log the distance travelled
  logDistance();

  //Code to read gyro
  //Written by Joe St. Germain for RBE 2002
  // reads imu every 20ms
  if((millis()-timer)>=20)  
  {
  complimentaryFilter();
  readGyro();
  }
}
}

void calibrateSensors() {
  
}

/*
 * Returns the counts logged by the left side encoder
 * Output: counts of left encoder
 */
long getLeftCounts() {
  return leftEnc.read();
}

/*
 * Returns the counts logged by the right side encoder
 * Output: counts of right encoder
 */
long getRightCounts() {
  return rightEnc.read();
}

/*
 * Function to reset encoders (write zero to both)
 */
void resetEncoders() {
  leftEnc.write(0);
  rightEnc.write(0);
}

/*
 * Calculates the distance travelled by both encoders, then averages the two
 * Output: average distance logged by left and right encoders
 */
double getEncDistance() {
  double avgCounts = (getLeftCounts() + getRightCounts()) / 2;
  return avgCounts * countsPerRev * wheelCircumfrence;
}

/*
 * Drives the robotat the given speed and turn magnitude
 * speed: double [-1,1]; -1 being full reverse while 1 is full forward
 * turn: double [-1,1]; -1 being CCW rotation and 1 being CW rotation
 */
void drive(double speed, double turn) {
  if(turn == 0) {
    driveMotors(speed, speed);
  } else if (turn < 0) {
    driveMotors((1 - turn) * speed, speed);
  } else {
    driveMotors(speed,  (1 - turn) * speed);
  }
}

/*
 * Drives the motors in a "tank drive" configuartion
 * left: double [-1,1]; -1 being full reverse while 1 is full forward
 * right: double [-1,1]; -1 being full reverse while 1 is full forward
 */
void driveMotors(double left, double right) {
  moveLeftDrive(left);
  moveRightDrive(right);
}

/*
 * Stops the drive motors (writes 0 speed to both)
 */
void stopRobot() {
  driveMotors(0, 0);
}

/*
 * Low level drive function for left wheel, abstraction of h-bridge operation
 * speed: double [-1,1]; -1 being full reverse while 1 is full forward
 */
void moveLeftDrive(double speed) {
  if(speed > 0) {
    analogWrite(leftDriveFow, abs(255 * speed));
    digitalWrite(leftDriveRev, LOW);
  } else {
    digitalWrite(leftDriveFow, LOW);
    analogWrite(leftDriveRev, abs(255 * speed));
  }
}

/*
 * Low level drive function for right wheel, abstraction of h-bridge operation
 * speed: double [-1,1]; -1 being full reverse while 1 is full forward
 */
void moveRightDrive(double speed) {
  if(speed > 0) {
    analogWrite(rightDriveFow, abs(255 * speed));
    digitalWrite(rightDriveRev, LOW);
  } else {
    digitalWrite(rightDriveFow, LOW);
    analogWrite(rightDriveRev, abs(255 * speed));
  }
}

/*
 * Turns on fan by PWMing signal pin
 */
void startFan() {
  analogWrite(fanPort, 255);
}

/*
 * Stops fan
 */
void stopFan() {
  digitalWrite(fanPort, 0);
}

/*
 * Function to calculate speed when travelling a specified distance
 * distance: double; distance to drive
 * Output: double [-1,1]; speed to be passed to drive function
 */
double driveDistanceSpeed(double distance) {
  distInput = getEncDistance();
  distSetpoint = distance;
  distPID.Compute();
  return distOutput;
}

/*
 * Function to calulate speed when turning a specified angle
 * angle: double; the angle to turn to (not bounded [0,360] due to operation of gyro)
 * Output: double [-1,1]; speed to be passed to drive function
 * Note: use turnAngleSpeed/abs(turnAngleSpeed) for turn magnitude to ensure proper direction of turn
 */
double turnAngleSpeed(double angle) {
  angleInput = getGyroAngle();
  angleSetpoint = angle;
  anglePID.Compute();
  return angleOutput;
}

/*
 * Function to calculate speed when turning 90 degrees (for use with constant desired angle)
 * Output: double [-1,1]; speed to be passed to drive function
 * Note: use turnAngleSpeed/abs(turnAngleSpeed) for turn magnitude to ensure proper direction of turn
 */
double turn90Speed() {
  angleInput = getGyroAngle();
  anglePID.Compute();
  return angleOutput;
}

/*
 * Function to calculate turn magnitude when following a wall
 * Output: double [-1,1]; magnitude of turn to be passed to drive function
 */
double wallFollowTurn() {
  sideInput = getSideDistance();
  sidePID.Compute();
  return sideOutput;
}

/*
 * Function to calculate speed when approaching a wall
 * Output: double [-1,1]; speed to be passed to drive function
 */
double frontWallSpeed() {
  frontInput = getFrontDistance();
  frontPID.Compute();
  return frontOutput;
}

/*
 * Function to calculate magnitude of turn when driving straight with encoders
 * Output: double [-1,1]; magnitude of turn to be passed to drive function
 */
double keepStraightTurn() {
  encInput = getRightCounts() - getLeftCounts();
  encPID.Compute();
  return encOutput;
}

/*
 * Function to read the distance from the front ultrasonic sensor
 * Output: int [5,30]; the distance read by the ultrasonic sensor
 * Note: the interval given describes maximum and minimum effective ranges
 * for the sensor. Values outside of this interval should be ignored.
 * Of special note is "0," which indicates a bad reading.
 */
int getFrontDistance() {
  return frontSonar.ping_cm();
}

/*
 * Function to read the distance from the right ultrasonic sensor
 * Output: int [5,30]; the distance read by the ultrasonic sensor
 * Note: the interval given describes maximum and minimum effective ranges
 * for the sensor. Values outside of this interval should be ignored.
 * Of special note is "0," which indicates a bad reading.
 */
int getSideDistance() {
  return sideSonar.ping_cm();
}

/*
 * Function to drive along a wall up to a wall in front of the robot
 * using front and side ultrasonic sensors for navigation
 * speed: double (0,1]; 1 being full speed and 0 being stopped
 */
void wallFollow(double speed) {
  drive(speed * frontWallSpeed(), speed * wallFollowTurn());
}

/*
 * Function to drive a speciified distance using encoders for both
 * odometry and keeping straight
 * distance: double; distance to drive
 * speed: double (0,1]; 1 being full speed and 0 being stopped
 */
void driveDistance(double distance, double speed) {
  drive(speed * driveDistanceSpeed(distance), speed * keepStraightTurn());
}

/*
 * Function to turn a specified number of degrees using the gyro
 * degrees: double; number of degrees to turn
 * speed: double (0,1]; 1 being full speed and 0 being stopped
 */
void turnDegrees(double degrees, double speed) {
  drive(speed * turnAngleSpeed(degrees), turnAngleSpeed(degrees)/abs(turnAngleSpeed(degrees));
}

/*
 * Function to approach obstacle in front of robot (candle or wall)
 * using ultrasonic sensor for ranging and encoders to keep straight
 * speed: double (0,1]; 1 being full speed and 0 being stopped
 */
void driveToWall(double speed) {
  drive(speed * frontWallSpeed(), speed * keepStraightTurn());
}

/*
 * Function to move stepper a specified number of degrees while keeping track
 * of the current position of the stepper
 * degrees: int; number of degrees to move the stepper
 */
void moveStepperDegrees(int degrees) {
  moveStepperDegrees(degrees);
  fanSteps += degrees / stepperSteps;
}

/*
 * Function to calculate the height of the flame
 * Output: double; the z-location of the flame in inches
 */
double getFlameZ() {
  double angleRad = flameAngleGuess  * 3.141 / 180;
  return cos(angleRad) * frontSetpoint;
}

/*
 * Function to get the current angle of the fan
 * Output: double; the current angle of the fan in degrees
 */
double getFanAngle() {
  return fanSteps * 3.141 / 180;
}

/*
 * Function to log the distance travelled by the encoders for odometry
 */
void logDistance() {
  if(prevDistanceReading > getEncDistance()) {
    prevDistanceReading = 0;
  }
  if(heading == EAST) {
    robotXLoc += getEncDistance() - prevDistanceReading;
  } else if(heading == SOUTH) {
    robotYLoc -= getEncDistance() - prevDistanceReading;
  } else if(heading == WEST) {
    robotXLoc -= getEncDistance() - prevDistanceReading;
  } else {
    robotYLoc += getEncDistance() - prevDistanceReading;
  }
  prevDistanceReading = getEncDistance();
}

/*
 * Function to read the value of the flame sensor
 * Output: int [0,1023]; the raw value from the flame sensor
 */
int readFlameSensor() {
  return analogRead(flameSensorPort);
}

/*
 * Function to read the value of the front cliff sensor
 * Output: int [0,1023]; the raw value from the front cliff sensor
 */
int readFrontCliff() {
  return analogRead(frontCliffSensorPort);
}

/*
 * Function to read the value of the right cliff sensor
 * Output: int [0,1023]; the raw value from the right cliff sensor
 */
int readRightCliff() {
  return analogRead(rightCliffSensorPort);
}

/*
 * Function to signify whether or not the robot is on aa cliff in front
 * Output: boolean; true if the robot is currently on a cliff in front, false otherwise
 */
boolean isFrontCliff() {
  return readFrontCliff() > cliffThreshold;
}

/*
 * Function to signify whether or not the robot is on a cliff on the right
 * Output: boolean; true if the robot is currently on a cliff on the right, false otherwise
 */
boolean isRightCliff() {
  return readRightCliff() > cliffThreshold;
}

/*
 * Function to signify if the robot currently sees the flame
 * Output: boolean; true if the robot sees a flame, false otherwise
 */
boolean seesFlame() {
  return readFlameSensor() < flameThreshold;
}

/*
 * Function to get the current X-location of the robot as determined by the encoders
 * Output: double; the current X-location of the robot
 */
double getXLocation() {
  return robotXLoc;
}


/*
 * Function to get the current X-location of the robot as determined by the encoders
 * Output: double; the current X-location of the robot
 */
double getYLocation() {
  return robotYLoc;
}

/*
 * Function to get the current heading of the robot from hte gyro
 * Output: double; the current heading of the robot
 */
double getGyroAngle() {
  return gyro_z;
}

//Code below is used to cetermine heading from gyro
//Written by Joe St. Germain for RBE 2002
void gyroZero() {
// takes 200 samples of the gyro
  for(int i =0;i<200;i++){  
  gyro.read();
  gerrx+=gyro.g.x;
  gerry+=gyro.g.y;
  gerrz+=gyro.g.z;
  delay(20);
  }
  gerrx = gerrx/200; // average reading to obtain an error/offset
  gerry = gerry/200;
  gerrz = gerrz/200;

  Serial.println(gerrx); // print error vals
  Serial.println(gerry);  
  Serial.println(gerrz);
}

void readGyro(){
  gyro.read(); // read gyro
  timer=millis(); //reset timer
  gyro_x=(float)(gyro.g.x-gerrx)*G_gain; // offset by error then multiply by gyro gain factor 
  gyro_y=(float)(gyro.g.y-gerry)*G_gain;
  gyro_z=(float)(gyro.g.z-gerrz)*G_gain;

  gyro_x = gyro_x*G_Dt; // Multiply the angular rate by the time interval
  gyro_y = gyro_y*G_Dt; 
  gyro_z = gyro_z*G_Dt;

  gyro_x +=gyro_xold; // add the displacment(rotation) to the cumulative displacment
  gyro_y += gyro_yold;
  gyro_z += gyro_zold;
        
  gyro_xold=gyro_x ; // Set the old gyro angle to the current gyro angle
  gyro_yold=gyro_y ;
  gyro_zold=gyro_z ;
}

void printGyro(){
  timer2=millis();
    
 // The gyro_axis variable keeps track of roll, pitch,yaw based on the complimentary filter
  Serial.print(" GX: ");
  Serial.print(gyro_x);
  Serial.print(" GY: ");
  Serial.print(gyro_y);
  Serial.print(" GZ: ");
  Serial.print(gyro_z);

  Serial.print("  Ax =  ");
  Serial.print(accel_x);
  Serial.print("  Ay =  ");
  Serial.print(accel_y);
  Serial.print("  Az =  ");
  Serial.println(accel_z);
}

void Accel_Init()
{
  accel.init();
  accel.enableDefault();
  Serial.print("Accel Device ID");
  Serial.println(accel.getDeviceType());
  switch (accel.getDeviceType())
  {
    case LSM303::device_D:
      accel.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::device_DLHC:
      accel.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      accel.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
}

void accelZero(){
  //I found this to be more problematic than it was worth.
  //not implemented 
  // takes 100 samples of the accel
  for(int i =0;i<100;i++){  
  gyro.read();
  aerrx+=accel.a.x >> 4;
  aerry+=accel.a.y >> 4;
  aerrz+=accel.a.z >> 4;
  delay(10);
  }
  aerrx = gerrx/100; // average reading to obtain an error/offset
  aerry = gerry/100;
  aerrz = gerrz/100;
  Serial.println("accel starting values");
  Serial.println(aerrx); // print error vals
  Serial.println(aerry);  
  Serial.println(aerrz);
}


// Reads x,y and z accelerometer registers
void readAccel()
{
  accel.readAcc();
  
  accel_x = accel.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  accel_y = accel.a.y >> 4; 
  accel_z = accel.a.z >> 4; 

  // accelerations in G
  accel_x = (accel_x/256);
  accel_y = (accel_y/256);
  accel_z = (accel_z/256);
  }

void complimentaryFilter(){
  readGyro();
  readAccel();
float x_Acc,y_Acc;
float magnitudeofAccel= (abs(accel_x)+abs(accel_y)+abs(accel_z));
if (magnitudeofAccel < 6 && magnitudeofAccel > 1.2)
{
  x_Acc = atan2(accel_y,accel_z)*180/ PI;
  gyro_x = gyro_x * 0.98 + x_Acc * 0.02;

  y_Acc = atan2(accel_x,accel_z)* 180/PI;
  gyro_y = gyro_y * 0.98 + y_Acc * 0.02;  
}

}
