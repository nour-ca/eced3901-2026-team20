#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

#define TCAADDR 0x70

// Safety LEDs
#define SAFE_FRONT 2
#define SAFE_FRONT_LEFT 3
#define SAFE_FRONT_RIGHT 4
#define SAFE_BACK 5

// RGB LED pins
#define RGB_RED 6
#define RGB_GREEN 7
#define RGB_BLUE 8

// Multiplexer select
void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// Read sensor
int readSensor(uint8_t channel) {
  tcaSelect(channel);
  sensor.init();
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.startContinuous(20);
  delay(50);
  int distance = sensor.read();
  sensor.stopContinuous();
  return distance;
}

// Set RGB color
void setColor(int r, int g, int b) {
  analogWrite(RGB_RED, r);
  analogWrite(RGB_GREEN, g);
  analogWrite(RGB_BLUE, b);
}

void setup() {

  Wire.begin();
  Serial.begin(9600);

  pinMode(SAFE_FRONT, OUTPUT);
  pinMode(SAFE_FRONT_LEFT, OUTPUT);
  pinMode(SAFE_FRONT_RIGHT, OUTPUT);
  pinMode(SAFE_BACK, OUTPUT);

  pinMode(RGB_RED, OUTPUT);
  pinMode(RGB_GREEN, OUTPUT);
  pinMode(RGB_BLUE, OUTPUT);

  // Safety LEDs always on
  digitalWrite(SAFE_FRONT, HIGH);
  digitalWrite(SAFE_FRONT_LEFT, HIGH);
  digitalWrite(SAFE_FRONT_RIGHT, HIGH);
  digitalWrite(SAFE_BACK, HIGH);
}

void loop() {

  int d1 = readSensor(1);
  int d2 = readSensor(2);
  int d3 = readSensor(3);

  int closest = min(min(d1, d2), d3);

  Serial.print("Left: ");
  Serial.print(d1);
  Serial.print(" mm, Right: ");
  Serial.print(d2);
  Serial.print(" mm, Back: ");
  Serial.print(d3);
  Serial.print(" mm, Closest: ");
  Serial.println(closest);

  // Distance thresholds
  if (closest >= 1219) {           // >4 ft → BLUE
    setColor(0, 0, 255);
  }
  else if (closest >= 610 && closest < 1219) {       // 2–4 ft → YELLOW
    setColor(255, 0, 255);
  }
  else {                          // <2 ft → PINK
    setColor(50, 150, 255);
  }

  delay(200);
}
