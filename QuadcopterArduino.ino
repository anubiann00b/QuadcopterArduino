#include <Servo.h>
#include <PID_v1.h>

#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include "CommunicationUtils.h"
#include <Wire.h>
#include "CommunicationUtils.h"
#include "DebugUtils.h"
#include "PID_v1.h"

typedef struct {
  double 
} CustomPid;

float q[4];

FreeSixIMU my3IMU = FreeSixIMU();

double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;

PID rollPid(&rollInput, &rollOutput, &rollSetpoint, 80, 0, 0, DIRECT);
PID pitchPid(&pitchInput, &pitchOutput, &pitchSetpoint, 80, 0, 0, DIRECT);

//Servo frontRight;
//Servo frontLeft;
//Servo backRight;
//Servo backLeft;

void setup_motor() {
  //frontRight.writeMicroseconds(1000);
  //frontLeft.writeMicroseconds(1000);
  //backRight.writeMicroseconds(1000);
  //backLeft.writeMicroseconds(1000);
  delay(15000);
}

void setup() {
  //frontRight.attach(6);
  //frontLeft.attach(9);
  //backRight.attach(5);
  //backLeft.attach(3);

  Serial.begin(19200);
  Wire.begin();

  delay(5);
  my3IMU.init();
  delay(5);

  rollPid.SetMode(AUTOMATIC);
  pitchPid.SetMode(AUTOMATIC);

  rollPid.SetOutputLimits(-50, 50);
  pitchPid.SetOutputLimits(-50, 50);

  //setup_motor();
  Serial.println("Initialized");
}

void loop() { 
  pid();

//  print(1200+rollOutput+pitchOutput, 1200-rollOutput+pitchOutput, 1200+rollOutput-pitchOutput, 1200-rollOutput-pitchOutput);
  print(rollInput, pitchInput);

  //frontRight.writeMicroseconds(1200 + rollOutput + pitchOutput);
  //frontLeft.writeMicroseconds(1200 - rollOutput + pitchOutput);
  //backRight.writeMicroseconds(1200 + rollOutput - pitchOutput);
  //backLeft.writeMicroseconds(1200 - rollOutput - pitchOutput);

  delay(60);
}

void pid() {
  my3IMU.getQ(q);

  rollInput = q[1];
  pitchInput = q[2];

  rollPid.Compute();
  pitchPid.Compute();
}

void print(double d1, double d2) {
  Serial.print(d1);
  Serial.print(" ");
  Serial.println(d2);
}

void print(double d1, double d2, double d3, double d4) {
  Serial.print(d1); Serial.print(" ");
  Serial.print(d2); Serial.print(" ");
  Serial.print(d3); Serial.print(" ");
  Serial.println(d4);
}

void pid 