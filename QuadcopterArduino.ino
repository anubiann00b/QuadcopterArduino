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
double rsp, rip, rop;
double rsn, rin, ron;
double psp, pip, pop;
double psn, pin, pon;

//Specify the links and initial tuning parameters
PID rp(&rip, &rop, &rsp, 2, 5, 1, DIRECT);
PID rn(&rin, &ron, &rsn, 2, 5, 1, DIRECT);
PID pp(&pip, &pop, &psp, 2, 5, 1, DIRECT);
PID pn(&pin, &pon, &psn, 2, 5, 1, DIRECT);

void setup() {

  Serial.begin(115200);
  Wire.begin();

  delay(5);
  my3IMU.init();
  delay(5);

  //turn the PID on
  rp.SetMode(AUTOMATIC);
  rn.SetMode(AUTOMATIC);
  pp.SetMode(AUTOMATIC);
  pn.SetMode(AUTOMATIC);
}

void loop() { 
  my3IMU.getQ(q);

  rip = q[1];
  rin = -q[1];
  pip = q[2];
  pin = -q[2];

  rp.Compute();
  rn.Compute();
  pp.Compute();
  pn.Compute();
  
  delay(60);
}
