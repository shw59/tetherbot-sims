/* end_agent.ino

  // Inputs:
  // Flex Sensor Readings - From pins A2 and A3 
  // ang_cant0 = Angle of cantilever 0 (bottom cantilever)  
  // ang_cant1 = Angle of cantilever 1 (top cantilever)
  // Encoder Angles - From qwiic and SDA (pin 6) and SCL (pin 7)
  // ang_T0 = Angle of encoder 0 (bottom encoder)
  // ang_T1 = Angle of encoder 1 (top encoder)
      // Convert raw angles into usable angles
      // theta0 = Calibrated angle of encoder 0
      // theta1 = Calibrated angle of encoder 1

  // Outputs:
  // Matrix combining the direction vectors of the cantilevers and encoders
  // Matrix VR
  // mag_VR = Magnitude of matrix VR
  // ang_VR = Angle of matrix VR
      // Find a usable angle from ang_VR
      // phiDes = Angle to orient encoder 0 during PID control

*/

#include <Wire.h>
#include "AS5600.h"
#include "math.h"
#include <BasicLinearAlgebra.h>
using namespace BLA;

#define RIGHT_MOTOR_FORWARD A16
#define RIGHT_MOTOR_BACKWARD A15
#define LEFT_MOTOR_FORWARD 13
#define LEFT_MOTOR_BACKWARD A14

#define BOTTOM_FLEX_SENSOR A2
#define TOP_FLEX_SENSOR A3

// I2C wires for the tether angle encoders
AS5600 as5600_0(&Wire);
AS5600 as5600_1(&Wire1);

// Flex Sensor Initialization
float bottomFlexSensorRaw; float topFlexSensorRaw; // analog outputs
float bottomFlexSensorAngle; float topFlexSensorAngle; // angle outputs
float bottomStrain; float topStrain; // not really strain, but a vector magnitude calculated based on the flex sensor's angles

//// ANALOG OUTPUTS FOR CALIBRATING FLEX SENSORS ////// 
  //// STRAIGHT = 40 degrees //// BEND = 90 degrees

//// ROBOT 3 //////////////////////////////////////////
const float STRAIGHT_0 = 405; const float BEND_0 = 295;
const float STRAIGHT_1 = 430; const float BEND_1 = 319;
///////////////////////////////////////////////////////

/////////////////////////////////////////////////////// 

// Encoder Angles Initalization
float bottomEncoderRaw; float topEncoderRaw; // raw encoder angles
float bottomTetherAngle; float topTetherAngle; // calibrated encoder angle
float deltaM;                   // Difference between encoder angles
float mag_VE;                   // Combined Magnitude
float VE_ang_OG; float VE_ang;  // Angle to input into direction vector

//// ENCODER CALIBRATION OUTPUTS /////////////////////////////////////
//// THESE WILL BE DIFFERENT FOR EACH ENCODER
//// ROBOT 3 ////////////////////////////////////////////////////////
  //// Encoder 0 
float T0_0 = 110; float T0_90first = 16; float T0_90second = 376; 
float T0_180 = 276; float T0_270 = 194; float T0_360 = 110;
  //// Encoder 1
float T1_0 = 116; float T1_90first = 35; float T1_90second = 395; 
float T1_180 = 302; float T1_270 = 210; float T1_360 = 116;

//////////////////////////////////////////////////////////////////////

// direction vectors
float goalHeadingMagnitude; float goalHeadingAngle; // next goal pose for agent

// Resultant Vector
float mag_VR; float ang_VR;

// PID Initialization 
float dt; float errorE; float derivative; float PID_control_O; float previousError; 

// Goal Angle and Upper Stop Angle
float goalA = 65;
float lowerA = 45;
float upperA = 80;

// Flex Sensor Range Angles 
float lower0 = 60; float upper0 = 70;
float lower1 = 60; float upper1 = 70;

// Array Initialization
const int maxnew = 5000; 
float timeR[maxnew];
float ang_cant0_Array[maxnew]; float ang_cant1_Array[maxnew];     
float theta0_Array[maxnew]; float theta1_Array[maxnew];     
float deltaM_Array[maxnew]; float deltaD_Array[maxnew];
float mag_VR_Array[maxnew]; float ang_VR_Array[maxnew]; 
float phiDes_Array[maxnew]; 

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire1.begin();

  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);

  pinMode(BOTTOM_FLEX_SENSOR, INPUT);
  pinMode(TOP_FLEX_SENSOR, INPUT);

  as5600_0.begin();  //  set direction pin.
  as5600_0.setDirection(AS5600_CLOCK_WISE);
  Serial.println(as5600_0.getAddress(),HEX);
  Serial.print("Connect device 0: ");
  Serial.println(as5600_0.isConnected() ? "true" : "false");
  int b = as5600_0.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  delay(1000);

  as5600_1.begin();  //  set direction pin.
  as5600_1.setDirection(AS5600_CLOCK_WISE);
  Serial.println(as5600_1.getAddress(),HEX);
  Serial.print("Connect device 1: ");
  Serial.println(as5600_1.isConnected() ? "true" : "false");
  int c = as5600_1.isConnected();
  Serial.print("Connect: ");
  Serial.println(c);
  delay(1000);
}


void loop() {
//// READING SENSORS
  // Flex Sensor Angles (degrees) 
  bottomFlexSensorRaw = analogRead(BOTTOM_FLEX_SENSOR);
  bottomFlexSensorAngle = map(bottomFlexSensorRaw, STRAIGHT_0, BEND_0, 40, 90);
  topFlexSensorRaw = analogRead(TOP_FLEX_SENSOR);
  topFlexSensorAngle = map(topFlexSensorRaw, STRAIGHT_1, BEND_1, 40, 90);

  topFlexSensorAngle = goalA;
  Serial.print("ang_cant0: "); Serial.print(bottomFlexSensorAngle); Serial.print("  "); Serial.print("ang_cant1: "); Serial.print(topFlexSensorAngle); Serial.println();

  // Encoder Angles 
  bottomEncoderRaw = as5600_0.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI; 
  topEncoderRaw = as5600_1.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;
  Serial.print("ang_T0: "); Serial.print(bottomEncoderRaw); Serial.print("  "); Serial.print("ang_T1: "); Serial.print(topEncoderRaw); Serial.println();

/////// CONVERTING RAW SENSOR VALUES TO SOMETHING USEABLE
  // Using cantilever angle to compute magnitude for each tether
  mag_T0 = sq((bottomFlexSensorAngle - goalA) * M_PI/180);
  mag_T1 = sq((topFlexSensorAngle - goalA) * M_PI/180);
  //Serial.print("mag_T0: "); Serial.print(mag_T0); Serial.print("  "); Serial.print("mag_T1: "); Serial.print(mag_T1); Serial.println();
  
  if (bottomFlexSensorAngle > upperA) {mag_T0 = 0;}
  if (topFlexSensorAngle > upperA) {mag_T1 = 0;}

//// ROBOT 3 
  // Calibrating the magnetic encoder for tether angles
  if (ang_T0 < T0_90first && ang_T0 >= 0) {ang_T0 = ang_T0 + 360;}
  if(bottomEncoderRaw > T0_90first && bottomEncoderRaw <= T0_0) {theta0 = map(bottomEncoderRaw, T0_0, T0_90first, 0, 90);}
  else if (bottomEncoderRaw > T0_180 && bottomEncoderRaw <= T0_90second) {theta0 = map(bottomEncoderRaw, T0_90second, T0_180, 90, 180);}
  else if (bottomEncoderRaw > T0_270 && bottomEncoderRaw <= T0_180) {theta0 = map(bottomEncoderRaw, T0_180, T0_270, 180, 270);}
  else if (bottomEncoderRaw > T0_360 && bottomEncoderRaw <= T0_270) {theta0 = map(bottomEncoderRaw, T0_270, T0_360, 270, 360);}

  //Serial.println(ang_T1);
  if (topEncoderRaw < T1_90first && topEncoderRaw >= 0) {topEncoderRaw = topEncoderRaw + 360;}
  if (topEncoderRaw > T1_90first && topEncoderRaw <= T1_0) {theta1 = map(topEncoderRaw, T1_0, T1_90first, 0, 90) + theta0;}
  else if (topEncoderRaw > T1_180 && topEncoderRaw <= T1_90second) {theta1 = map(topEncoderRaw, T1_90second, T1_180, 90, 180) + theta0;}
  else if (topEncoderRaw > T1_270 && topEncoderRaw <= T1_180) {theta1 = map(topEncoderRaw, T1_180, T1_270, 180, 270) + theta0;}
  else if (topEncoderRaw > T1_360 && topEncoderRaw <= T1_270) {theta1 = map(topEncoderRaw, T1_270, T1_360, 270, 360) + theta0;}
  if (theta1 > 360) {theta1 = theta1 - 360;}


  theta0 = theta0 * M_PI/180; // Convert angles to radians
  theta1 = theta1 * M_PI/180;

  Serial.print("theta0: "); Serial.print(theta0 * 180/M_PI); Serial.print("  "); Serial.print("theta1: "); Serial.print(theta1 * 180/M_PI); Serial.println();
  Serial.println();

/////// STRAIN CORRECTION VECTORS
  // Find angles to input into direction vectors
  // If tether angle is less than 80 use theta0 or theta1
  // If tether angle is greater than 80 add 180 to theta0 or theta1 (flip angle)
  if (bottomFlexSensorAngle <= goalA) {ang_T0 = theta0;}
  else if (bottomFlexSensorAngle > goalA) {ang_T0 = theta0 + (180 * M_PI/180);}
  if (topFlexSensorAngle <= goalA) {ang_T1 = theta1;}
  else if (topFlexSensorAngle > goalA) {ang_T1 = theta1 + (180 * M_PI/180);}

  // Keep angles under 360 degrees 
  if (ang_T0 > (360 * M_PI/180)) {ang_T0 = ang_T0 - (360 * M_PI/180);}
  if (ang_T1 > (360 * M_PI/180)) {ang_T1 = ang_T1 - (360 * M_PI/180);}
  Serial.print("ang_T0: "); Serial.print(ang_T0 * 180/M_PI); Serial.print("  "); Serial.print("ang_T1: "); Serial.print(ang_T1 * 180/M_PI); Serial.println();

/////// ASSEMBLE VECTORS
  Matrix<1,2> VT0 = {mag_T0*cos(ang_T0), mag_T0*sin(ang_T0)}; // Tether 0 (Lower)
  Matrix<1,2> VT1 = 0; //{mag_T1*cos(ang_T1), mag_T1*sin(ang_T1)}; // Tether 1 (Upper)
  Matrix<1,2> VE = {mag_VE*cos(VE_ang), mag_VE*sin(VE_ang)}; // Magnetic Encoders 

  // Constants
  float cE = 1; float cT = 1;   
  // Cantilevers
  Matrix<1,2> VT_combined = VT0; //VT0 + VT1;
  float VT_p1 = cT*VT_combined(0,0); float VT_p2 = cT*VT_combined(0,1);
  Matrix<1,2> VT_new = {VT_p1, VT_p2};
  float mag_VT = sqrt(sq(VT_new(0,0)) + sq(VT_new(0,1)));
  float ang_VT = atan2(VT_new(0,1),VT_new(0,0));
  // Encoders
  float VE_p1 = cE*VE(0,0); float VE_p2 = cE*VE(0,1);
  // If there is no desired angle only keep tension 
  // Set Encoder portion of input to zero
  if (angleInput == 0) {
    VE_p1 = 0; VE_p2 = 0; deltaD = 0;
    Serial.println("NO ENCODER");
  } 
  Matrix<1,2> VE_new = {VE_p1, VE_p2};
  float mag_VE = sqrt(sq(VE_new(0,0)) + sq(VE_new(0,1)));
  float ang_VE = atan2(VE_new(0,1),VE_new(0,0));

  // Resultant Vector - Magnitude and Direction
  Matrix<1,2> VR = {VT_new}; //+ VE_new};
  mag_VR = sqrt(sq(VR(0,0)) + sq(VR(0,1)));
  ang_VR = atan2(VR(0,1),VR(0,0));
  
  // Find positive angle if negative
  if (ang_VR < 0) {ang_VR = (360 * M_PI/180) + ang_VR;}

  // Find desired theta0 to use in PID control
  goalHeadingAngle = (90 * M_PI/180) + theta0 - ang_VR;
  if (goalHeadingAngle > (360 * M_PI/180)) {goalHeadingAngle = goalHeadingAngle - (360 * M_PI/180);} 
  if (goalHeadingAngle < 0) {goalHeadingAngle = goalHeadingAngle + (360 * M_PI/180);}
  if (bottomFlexSensorAngle == goalA && topFlexSensorAngle > upperA) {goalHeadingAngle = theta0;}

  Serial.print("phiDes: "); Serial.print(goalHeadingAngle * 180/M_PI); Serial.println();

  //// Step 3: Spin to phiDes AND Step 4: Drive for ts //////////////////////////////
  // Only enter Spin and Drive functions if flex sensors are in tension (range 60 - 85)
  //if (ang_cant0 <= lower0 || ang_cant0 >= upper0 && ang_cant0 < lowerA && ang_cant0 > upperA)// && ang_cant1 <= lower1 && ang_cant1 >= upper1) //(ang_cant0 >= 50 && ang_cant0 <= 80)
  if (bottomFlexSensorAngle <= lower0 || bottomFlexSensorAngle >= upper0) // && ang_cant1 < lowerA && ang_cant1 > upperA)
  {
    // Spin to phiDes
    SPIN(theta0, goalHeadingAngle, deltaD); Serial.println("SPIN DONE:");
    
    // Drive 
    DRIVE(); Serial.println("DRIVE DONE:");
    //DRIVE(angleInput, deltaD); Serial.println("DRIVE DONE:");
  }

  delay(250);
}


void SPIN(float theta0, float phiDes, float deltaD) 
{
  Serial.print("theta0: "); Serial.println(theta0 * 180/M_PI);
  Serial.print("phiDes: "); Serial.println(phiDes * 180/M_PI);
  float Kp = 1;//16; float Kd = 1; 
  float endTime = millis(); float previousTime = millis(); 
  
  while (theta0*180/M_PI > (phiDes*180/M_PI)+5 || theta0*180/M_PI < (phiDes*180/M_PI)-5)
  {
    Serial.println("START:");
    // Angle Formation //////////////////////////////////////////////////////////////////
    ang_T0 = as5600_0.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;
    ang_T1 = as5600_1.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;

  //// ROBOT 3 
    // Calibrating the magnetic encoder for tether angles
    if (ang_T0 < T0_90first && ang_T0 >= 0) {ang_T0 = ang_T0 + 360;}
    if(ang_T0 > T0_90first && ang_T0 <= T0_0) {theta0 = map(ang_T0, T0_0, T0_90first, 0, 90);}
    else if (ang_T0 > T0_180 && ang_T0 <= T0_90second) {theta0 = map(ang_T0, T0_90second, T0_180, 90, 180);}
    else if (ang_T0 > T0_270 && ang_T0 <= T0_180) {theta0 = map(ang_T0, T0_180, T0_270, 180, 270);}
    else if (ang_T0 > T0_360 && ang_T0 <= T0_270) {theta0 = map(ang_T0, T0_270, T0_360, 270, 360);}

    //Serial.println(ang_T1);
    if (ang_T1 < T1_90first && ang_T1 >= 0) {ang_T1 = ang_T1 + 360;}
    if (ang_T1 > T1_90first && ang_T1 <= T1_0) {theta1 = map(ang_T1, T1_0, T1_90first, 0, 90) + theta0;}
    else if (ang_T1 > T1_180 && ang_T1 <= T1_90second) {theta1 = map(ang_T1, T1_90second, T1_180, 90, 180) + theta0;}
    else if (ang_T1 > T1_270 && ang_T1 <= T1_180) {theta1 = map(ang_T1, T1_180, T1_270, 180, 270) + theta0;}
    else if (ang_T1 > T1_360 && ang_T1 <= T1_270) {theta1 = map(ang_T1, T1_270, T1_360, 270, 360) + theta0;}
    if (theta1 > 360) {theta1 = theta1 - 360;}


    theta0 = theta0 * M_PI/180;
    theta1 = theta1 * M_PI/180;

    if (theta0 < theta1) {deltaM = theta1 - theta0;}
    else if (theta0 > theta1 && theta0 >= (270 * M_PI/180) && theta0 <= (360 * M_PI/180) && theta1 >= 0 && theta1 <= (270 * M_PI/180))
    {deltaM = (360*M_PI/180) - theta0 + theta1;}
    else {deltaM = (360*M_PI/180) - abs(theta1 - theta0);}

    // PID Control
    dt = (millis() - previousTime) / 1000;

    errorE = -1*(phiDes - theta0);
    if (errorE > M_PI) {errorE = -1*(2*M_PI - errorE);}
    if (errorE < -M_PI) {errorE = 1*(2*M_PI + errorE);}

    derivative = (errorE - previousError) / dt;
    if (derivative > 10) {derivative = 10;}
    else if (derivative < -10) {derivative = -10;}
    
    PID_control_O = Kp*errorE;// + Kd*derivative;  
    if (PID_control_O > 50) {PID_control_O = 50;}
    else if (PID_control_O < -50) {PID_control_O = -50;}
    previousError = errorE;
    previousTime = millis();
    
    // Motor Control
    int lowerR = 145;
    int lowerL = 145;
    RIGHT_MOTOR(lowerR);                   
    LEFT_MOTOR(lowerL); 

    int flexADC_0 = analogRead(A2);
    float ang_cant0 = map(flexADC_0, STRAIGHT_0, BEND_0, 40, 90);
    int flexADC_1 = analogRead(A3);
    float ang_cant1 = map(flexADC_1, STRAIGHT_1, BEND_1, 40, 90);

    Serial.print("PID_control_O: "); Serial.print(PID_control_O); Serial.println(); 
    Serial.print("ang_cant0: "); Serial.print(ang_cant0); Serial.print("  "); 
    Serial.print("ang_cant1: "); Serial.print(ang_cant1); Serial.println(); Serial.println();
    delay(250); 
  }  

  Serial.println("DONE:");     
  analogWrite(RIGHT_MOTOR_FORWARD, 0);
  analogWrite(A15, 0);   
  analogWrite(13, 0);
  analogWrite(A14, 0);   
  delay(1000);
}

void DRIVE()
{ 
  analogWrite(RIGHT_MOTOR_FORWARD, Right);
  analogWrite(A15, 0);   
  analogWrite(13, Left);
  analogWrite(A14, 0);  
  delay(100); 

  analogWrite(RIGHT_MOTOR_FORWARD, 0);
  analogWrite(A15, 0);   
  analogWrite(13, 0);
  analogWrite(A14, 0);  
  delay(1000); 
}

//Function to momag_VE Right Motor 
void RIGHT_MOTOR(int lower) {
  // Pin A16 - Right, Forward
  if (PID_control_O > 0) {
    if (PID_control_O >= lower && PID_control_O <= 255) {
      analogWrite(RIGHT_MOTOR_FORWARD, PID_control_O);
      analogWrite(A15, 0);
    }
    else if (PID_control_O >= 0 && PID_control_O < lower) {
      analogWrite(RIGHT_MOTOR_FORWARD, lower);
      analogWrite(A15, 0);
    }
    else if (PID_control_O > 255) {
      analogWrite(RIGHT_MOTOR_FORWARD, 255);
      analogWrite(A15, 0);
    }
  }
  // Pin A15 - Right, Backward
  if (PID_control_O < 0) {
    if (abs(PID_control_O) >= lower && abs(PID_control_O) <= 255) {
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
      analogWrite(A15, PID_control_O);
    }
    else if (abs(PID_control_O) >= 0 && abs(PID_control_O) < lower) {
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
      analogWrite(A15, lower);
    }
    else if (abs(PID_control_O) > 255) {
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
      analogWrite(A15, 255);
    }
  }
}
//Function to momag_VE Left Motor
void LEFT_MOTOR(int lower) {
  // Pin 13 - Left, Forward
  if (PID_control_O < 0) {
    if (abs(PID_control_O) >= lower && abs(PID_control_O) <= 255) {
      analogWrite(13, abs(PID_control_O));
      analogWrite(A14, 0);
    }
    else if (abs(PID_control_O) >= 0 && abs(PID_control_O) < lower) {
      analogWrite(13, lower);
      analogWrite(A14, 0);
    }
    else if (abs(PID_control_O) > 255) {
      analogWrite(13, 255);
      analogWrite(A14, 0);
    }
  }
  // Pin A14 - Left, Backward
  if (PID_control_O > 0) {
    if (abs(PID_control_O) >= lower && abs(PID_control_O) <= 255) {
      analogWrite(13, 0);
      analogWrite(A14, PID_control_O);
    }
    else if (abs(PID_control_O) >= 0 && abs(PID_control_O) < lower) {
      analogWrite(13, 0);
      analogWrite(A14, lower);
    }
    else if (abs(PID_control_O) > 255) {
      analogWrite(13, 0);
      analogWrite(A14, 255);
    }
  }
}
