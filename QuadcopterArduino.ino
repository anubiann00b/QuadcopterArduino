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

Servo sfr;
Servo sfl;
Servo sbr;
Servo sbl;

int pfr = 1000;
int pfl = 1000;
int pbr = 1000;
int pbl = 1000;

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

  rp.SetMode(AUTOMATIC);
  rn.SetMode(AUTOMATIC);
  pp.SetMode(AUTOMATIC);
  pn.SetMode(AUTOMATIC);

  setup_motor();
}

void loop() { 
  pid();

  sfr.writeMicroseconds(1225 -  (pop-pon + rop-ron)*25);
  sfl.writeMicroseconds(1225 -  (pop-pon - rop-ron)*25);
  sbr.writeMicroseconds(1225 - (-pop+pon + rop-ron)*25);
  sbl.writeMicroseconds(1225 - (-pop+pon - rop-ron)*25);

  delay(60);
}

void setup_motor() {
  sfr.writeMicroseconds(1000);
  sfl.writeMicroseconds(1000);
  sbr.writeMicroseconds(1000);
  sbl.writeMicroseconds(1000);
  delay(15000);
}

void pid() {
  my3IMU.getQ(q);

  rip = q[1];
  rin = -q[1];
  pip = q[2];
  pin = -q[2];

  rp.Compute();
  rn.Compute();
  pp.Compute();
  pn.Compute();
}