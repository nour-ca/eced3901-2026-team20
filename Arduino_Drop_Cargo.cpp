#include <Servo.h>

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(115200);
  servo1.attach(9);
  servo2.attach(10);

  servo1.write(90);
  servo2.write(90);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "MOVE_BOTH") {
      servo1.write(30);
      servo2.write(150);
      delay(1000);

      servo1.write(90);
      servo2.write(90);
    }
  }
}
