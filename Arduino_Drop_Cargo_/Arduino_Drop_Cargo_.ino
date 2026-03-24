#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;

void setup() {
  Serial.begin(115200);
  servo1.attach(5);
  servo2.attach(6);
  servo3.attach(7);
  servo4.attach(8);
  servo5.attach(9);
  servo6.attach(10);
  servo7.attach(11);
  servo8.attach(12);

  servo1.write(25); //Left
  servo2.write(90); //Right
  servo3.write(40);
  servo4.write(110);
  servo5.write(-10); 
  servo6.write(70); 
  servo7.write(90); 
  servo8.write(-10); 

}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "MOVE_BOTH") {
      servo1.write(100);
      servo2.write(150);
      delay(600);
      servo1.write(20); //Left
      servo2.write(75); //Right
      
    }
  }
}
