#include <Wire.h>
#include <LSM6.h>
#include <Balboa32U4.h>

extern LSM6 imu;

void init_imu() {
  if(!imu.init()) {
    ledRed(1);
    while(1);
  }
  imu.enableDefault();
}