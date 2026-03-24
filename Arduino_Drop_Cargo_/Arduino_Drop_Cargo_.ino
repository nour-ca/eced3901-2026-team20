#include <Servo.h>

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(115200);
  servo1.attach(5);
  servo2.attach(6);

  servo1.write(80);
  servo2.write(20);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "MOVE_BOTH") {
      servo1.write(220);
      servo2.write(160);
      delay(2000);

      servo1.write(80);
      servo2.write(20);
    }
  }
}
