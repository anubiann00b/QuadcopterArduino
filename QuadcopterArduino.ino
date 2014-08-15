#include <Servo.h>
#include <PID_v1.h>

#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#define DEBUG
#ifdef DEBUG
#include "DebugUtils.h"
#endif

#include "CommunicationUtils.h"
#include "FreeSixIMU.h"
#include <Wire.h>

float q[4]; //hold q values

// Set the FreeIMU object
FreeSixIMU my3IMU = FreeSixIMU();

//Define Variables we'll be connecting to
double rollSet, rollIn, rollOut;
double pitchSet, pitchIn, pitchOut;

//Specify the links and initial tuning parameters
PID roll(&rollIn, &rollOut, &rollSet, 2, 5, 1, DIRECT);
PID pitch(&pitchIn, &pitchOut, &pitchSet, 2, 5, 1, DIRECT);

Servo sfr;
Servo sfl;
Servo sbr;
Servo sbl;

void setup() {
  sfr.attach(2);
  sfl.attach(3);
  sbr.attach(4);
  sbl.attach(5);

  Serial.begin(115200);
  Wire.begin();

  delay(5);
  my3IMU.init();
  delay(5);

  //initialize the variables we're linked to
  rollIn = 0;
  rollSet = 0;  
  pitchIn = 0;
  pitchOut = 0;

  //turn the PID on
  roll.SetMode(AUTOMATIC);
  pitch.SetMode(AUTOMATIC);

  sfr.writeMicroseconds(1000);
  sfl.writeMicroseconds(1000);
  sbr.writeMicroseconds(1000);
  sbl.writeMicroseconds(1000);

  //delay(15000);

  sfr.writeMicroseconds(1100);
  sfl.writeMicroseconds(1100);
  sbr.writeMicroseconds(1100);
  sbl.writeMicroseconds(1100);

  //delay(5000);

  sfr.writeMicroseconds(1000);
  sfl.writeMicroseconds(1000);
  sbr.writeMicroseconds(1000);
  sbl.writeMicroseconds(1000);
}

void loop() { 
  my3IMU.getQ(q);

  rollIn = q[1];
  pitchIn = q[2];
  roll.Compute();
  pitch.Compute();
  Serial.print(rollIn);
  Serial.print(" ");
  Serial.println(rollOut);



  delay(60);
}
