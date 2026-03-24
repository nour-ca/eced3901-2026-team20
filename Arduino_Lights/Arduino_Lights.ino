#include <Wire.h>
#include <VL53L1X.h>

#define TCAADDR 0x70

VL53L1X sensor;

// Safety LEDs
#define SAFE_FRONT 2
#define SAFE_FRONT_LEFT 3
#define SAFE_FRONT_RIGHT 4
#define SAFE_BACK 5

// RGB LED pins (UNO PWM pins ONLY)
#define RGB_RED 9
#define RGB_GREEN 10
#define RGB_BLUE 11

// Track sensor status
bool sensor_ok[8] = {false, false, false, false, false, false, false, false};

// Select multiplexer channel
void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// Initialize a sensor on a given channel
bool initSensor(uint8_t channel) {
  tcaSelect(channel);
  delay(20);

  sensor.setTimeout(100);

  if (!sensor.init()) {
    Serial.print("Sensor init failed on channel ");
    Serial.println(channel);
    return false;
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);

  Serial.print("Sensor initialized on channel ");
  Serial.println(channel);
  return true;
}

// Initialize all sensors
void initAllSensors() {
  delay(500);  // allow sensors to boot after power
  sensor_ok[1] = initSensor(1);
  sensor_ok[2] = initSensor(2);
  sensor_ok[3] = initSensor(3);
}

// Safe sensor read with retry
int readSensor(uint8_t channel) {
  if (!sensor_ok[channel]) {
    sensor_ok[channel] = initSensor(channel);
    if (!sensor_ok[channel]) return -1;
  }

  tcaSelect(channel);
  delay(5);

  int distance = sensor.read();

  if (sensor.timeoutOccurred() || distance <= 0) {
    Serial.print("Read failed on channel ");
    Serial.print(channel);
    Serial.println(", retrying...");

    sensor_ok[channel] = initSensor(channel);
    if (!sensor_ok[channel]) return -1;

    tcaSelect(channel);
    delay(5);
    distance = sensor.read();

    if (sensor.timeoutOccurred() || distance <= 0) {
      Serial.print("Still failed on channel ");
      Serial.println(channel);
      return -1;
    }
  }

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

  // Turn on safety LEDs
  digitalWrite(SAFE_FRONT, HIGH);
  digitalWrite(SAFE_FRONT_LEFT, HIGH);
  digitalWrite(SAFE_FRONT_RIGHT, HIGH);
  digitalWrite(SAFE_BACK, HIGH);

  // RGB off initially
  setColor(0, 0, 0);

  Serial.println("Initializing sensors...");
  initAllSensors();
  Serial.println("Setup complete.");
}

void loop() {
  int d1 = readSensor(1);
  int d2 = readSensor(2);
  int d3 = readSensor(3);

  Serial.print("Left: ");
  Serial.print(d1);
  Serial.print(" mm, Right: ");
  Serial.print(d2);
  Serial.print(" mm, Back: ");
  Serial.print(d3);

  // Find closest safely (FIXED VERSION)
  int closest = -1;

  if (d1 > 0) closest = d1;
  if (d2 > 0 && (closest < 0 || d2 < closest)) closest = d2;
  if (d3 > 0 && (closest < 0 || d3 < closest)) closest = d3;

  if (closest < 0) {
    Serial.println(" -> no valid readings");
    setColor(255, 0, 0);  // RED = error
    delay(500);
    return;
  }

  Serial.print(", Closest: ");
  Serial.println(closest);

  // Distance thresholds
  if (closest >= 1219) {           // >4 ft
    setColor(0, 255, 255);           // BLUE
  }
  else if (closest >= 610) {       // 2–4 ft
    setColor(255, 70, 0);         // YELLOW
  }
  else {                           // <2 ft
    setColor(255, 0, 255);       // PINK
  }

  delay(200);
}
