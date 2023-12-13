#include <Balboa32U4.h>
#include "PID.h"
#include <Wire.h>
#include <LSM6.h>
#include "util.h"

LSM6 imu;

extern Balboa32U4Motors motors;
extern Balboa32U4Encoders encoders;

int32_t new_motor;
pid_controller pid_theta;

char report[80];

// 100Hz
const uint8_t UPDATE_FREQ_MS = 10;
const uint8_t CALIBRATION_ITERATIONS = 100;
const int16_t DISTANCE_RESPONSE = 73;
const int16_t SPEED_RESPONSE = 3300;
const int16_t DISTANCE_DIFF_RESPONSE = -50;

static int32_t gYZero;
static int32_t motorSpeed;
static int32_t angle, angleRate;//, angleSum;
static int32_t distanceLeft, speedLeft, distanceRight, speedRight;

// Parameters & methods taken directly from Balboa32U4 library example
void standup() {
  motors.setSpeeds(-300, -300);
  delay(400);
  motors.setSpeeds(150, 150);
  motorSpeed = 150;
}

/* Assuming lying still!
 * 1. Measure angleRate bias in gyro
 * 2. Measure angle using accelerometer
 */
void calibrate() {
  motors.setSpeeds(0, 0);
  motorSpeed = 0;

  // Calibrate the gyro.
  int32_t total = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++) {
    imu.read();
    total += imu.g.y;
    delay(1);
  }

  gYZero = total / CALIBRATION_ITERATIONS;

  // multiply by 180000/pi to convert to milidegrees
  // Measure angle using accelerometer
  unsigned long before = millis();
  angle = atan2(imu.a.z, imu.a.x) * 57296;
  Serial.println("Time taken");
  Serial.println(millis() - before);
  //angleSum = 0;

  snprintf(report, sizeof(report), "gyro bias:%6d angle:%6d", gYZero, angle);
  Serial.println(report);
  delay(1000);
}

void balance() {
  static uint16_t lastMillis;
  uint16_t ms = millis();

  // Perform the balance updates at 100 Hz.
  if ((uint16_t)(ms - lastMillis) < UPDATE_FREQ_MS) { return; }
  lastMillis = ms;

  // Read angleRate
  imu.read();

  // convert & integrate
  // Convert from millideg/s to deg/s.
  // 1/29*2^8 ~= 9
  //angleRate = (imu.g.y - gYZero) / 29;
  angleRate = (imu.g.y - gYZero) * 9 >> 8; // 9 * 2^(-8)
  angle += angleRate * UPDATE_FREQ_MS;
  //angleSum += angle;

  static int16_t lastCountsLeft;
  int16_t countsLeft = encoders.getCountsLeft();
  speedLeft = (countsLeft - lastCountsLeft);
  distanceLeft += countsLeft - lastCountsLeft;
  lastCountsLeft = countsLeft;

  static int16_t lastCountsRight;
  int16_t countsRight = encoders.getCountsRight();
  speedRight = (countsRight - lastCountsRight);
  distanceRight += countsRight - lastCountsRight;
  lastCountsRight = countsRight;

  //if (angleSum > 1024) angleSum = 1024;
  //if (angleSum < -1024) angleSum = -1024;
  //angleSum = angleSum > 300 ? 300 : angleSum < -300 ? -300 : angleSum;

  angle = angle * 999 / 1000;
  //angleSum = angleSum * 999 / 1000;

  // Run PID velocity update -> new theta
  const int16_t ANGLE_RATE_RATIO = 140;
  const int16_t ANGLE_RESPONSE = 11;
  const int16_t GEAR_RATIO = 111;

  // two targets are 0, so no minus error
  // 1/100/111*2^16 = 5.9
  // (* 6) >>16
  int32_t risingAngleOffset = angleRate * ANGLE_RATE_RATIO + angle;// + (angleSum>>1) - 1024;
  motorSpeed += ((
    ANGLE_RESPONSE * risingAngleOffset
    + DISTANCE_RESPONSE * (distanceLeft + distanceRight)
    + SPEED_RESPONSE * (speedLeft + speedRight)
    ) * 6) >> 16;  // / 100 / GEAR_RATIO;

  // Set theta target
  if (motorSpeed > 300) motorSpeed = 300;
  if (motorSpeed < -300) motorSpeed = -300;

  //motors.setSpeeds(motorSpeed, motorSpeed);
  int16_t distanceDiff = distanceLeft - distanceRight;

  int16_t lSpeed, rSpeed;
  //lSpeed = motorSpeed + distanceDiff * DISTANCE_DIFF_RESPONSE / 100;
  //rSpeed = motorSpeed - distanceDiff * DISTANCE_DIFF_RESPONSE / 100;
  lSpeed = motorSpeed - (distanceDiff >> 2);
  rSpeed = motorSpeed + (distanceDiff >> 2);

  motors.setSpeeds(lSpeed, rSpeed);

  //snprintf(report, sizeof(report), "ang:%6ld del:%6ld spd:%6ld", angle, angleRate, motorSpeed);
  //Serial.println(report);
  //Serial.println(risingAngleOffset);
  //snprintf(report, sizeof(report), "g.x:%6d g.y:%6d g.z:%6d", imu.g.x, imu.g.y, imu.g.z);
  //Serial.println(report);
  //snprintf(report, sizeof(report), "a.x:%6d a.y:%6d a.z:%6d", imu.a.x, imu.a.y, imu.a.z);
  //Serial.println(report);

  snprintf(report, sizeof(report), "%6ld, %6ld, %6ld, %6ld, %6ld, %6ld, %6d, %6d", angle, angleRate, distanceLeft, distanceRight, speedLeft, speedRight, lSpeed, rSpeed);
  Serial.println(report);
}

void setup() {
  // put your setup code here, to run once:
  
  // Init imu
  Serial.begin(9600);
  Wire.begin();
  if (!imu.init()) {
    while (true) {
      Serial.println("IMU failed to initialize!");
      delay(1000);
    }
  }

  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s
  delay(1000); // Wait 1s to stablize IMU readings

  //imu.readAcc();
  //pid_init(&pid_theta, 40, 25, 0);
  // 40, 0, 0
  // 80, 0, 1
  //pid_init(&pid_theta, 80, 0, 1);
  //pid_reset(&pid_theta, imu.a.x);

  Serial.println("initialized");

  calibrate();
  //standup();
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

  //Read theta

  /*
  imu.readAcc();

  //Run PID theta update -> new motor speed
  //new_motor = pid_update(&pid_theta, 0, imu.a.z); //16276
  //new_motor = -1 * new_motor;
  int32_t angle, angleRate;
  angle = imu.a.z;

  int32_t pd = ;
  motorSpeed = new_motor;
  if (motorSpeed > 300) motorSpeed = 300;
  if (motorSpeed < -300) motorSpeed = -300;
  //new_motor = 0;

  snprintf(report, sizeof(report), "imu:%6d mot:%6d", imu.a.x, new_motor);
  Serial.println(report);
  Serial.println(motorSpeed);

  //Set motor speed (in [-300, 300])
  motors.setSpeeds(motorSpeed, motorSpeed);
  delay(8);
  */

  balance();
}
