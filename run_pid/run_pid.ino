#include <Balboa32U4.h>
#include "PID.h"
#include <Wire.h>
#include <LSM6.h>
#include "util.h"

LSM6 imu;

extern Balboa32U4Motors motors;

int32_t new_motor;
pid_controller pid_theta;

char report[80];

void setup() {
  // put your setup code here, to run once:
  //init imu

  Serial.begin(9600);
  Wire.begin();
  init_imu();

  delay(1000);

  imu.readAcc();
  //pid_init(&pid_theta, 40, 25, 0);
  // 40, 0, 0
  pid_init(&pid_theta, 80, 0, 1);
  pid_reset(&pid_theta, imu.a.x);

  Serial.println("initialized");
}

void loop() {
  // put your main code here, to run repeatedly:
  /*motors.setLeftSpeed(100);
  motors.setRightSpeed(0);
  delay(1000);
  motors.setLeftSpeed(0);
  motors.setRightSpeed(100);
  delay(1000);
  motors.setSpeeds(0, 0);
  delay(1000);
  motors.setSpeeds(50, 50);
  delay(1000);
  motors.setSpeeds(0, 0);*/
  //NOTE: positive motor speed makes robot go towards arms with board side up

  //Read velocity

  //Run PID velocity update -> new theta

  //Set theta target

  //Read theta
  imu.readAcc();

  //Run PID theta update -> new motor speed
  new_motor = pid_update(&pid_theta, 0, imu.a.z); //16276
  new_motor = -1 * new_motor;
  //new_motor = 0;

  snprintf(report, sizeof(report), "acc: %6d, pidout: %6d",
    imu.a.x, new_motor);
  Serial.println(report);

  //Set motor speed (in [-300, 300])
  motors.setSpeeds(new_motor, new_motor);
  delay(8);
}
