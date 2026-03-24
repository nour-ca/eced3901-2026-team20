#include <Wire.h>
#include <VL53L1X.h>

VL53L1X leftSensor;
VL53L1X rightSensor;

#define XSHUT_LEFT 2
#define XSHUT_RIGHT 3

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // Hold both in reset
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  // Bring up LEFT, set new address
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  if (!leftSensor.init()) {
    Serial.println("LEFT init failed");
    while (1) {}
  }
  leftSensor.setAddress(0x30);

  // Bring up RIGHT, set new address
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if (!rightSensor.init()) {
    Serial.println("RIGHT init failed");
    while (1) {}
  }
  rightSensor.setAddress(0x31);

  // Optional tuning
  leftSensor.setDistanceMode(VL53L1X::Long);
  rightSensor.setDistanceMode(VL53L1X::Long);

  leftSensor.startContinuous(50);
  rightSensor.startContinuous(50);
}

void loop() {
  int left_mm = leftSensor.read();   // millimeters
  int right_mm = rightSensor.read(); // millimeters

  // CSV for ROS: left_mm,right_mm
  Serial.print(left_mm);
  Serial.print(",");
  Serial.println(right_mm);

  delay(50);
}
