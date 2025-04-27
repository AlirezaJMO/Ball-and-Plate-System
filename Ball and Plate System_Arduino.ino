#include <Servo.h>

Servo servo1;
Servo servo2;

void setup() {
  servo1.attach(9);  // attaches the servo on pin 9 y axis
  servo2.attach(10); // attaches the servo on pin 10 x axis
  Serial.begin(9600); // starts the serial communication
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n'); // read the incoming data until newline
    int separatorIndex = data.indexOf(','); // find the index of the comma separator

    int angle1 = data.substring(0, separatorIndex).toInt(); // extract the first angle
    int angle2 = data.substring(separatorIndex + 1).toInt(); // extract the second angle

      // Map the received angles from -90 to 90 to 0 to 180 degrees
      int mappedAngle1 = map(angle1, -90, 90, 0, 180);
      int mappedAngle2 = map(angle2, -90, 90, 0, 180);
      
            servo1.write(mappedAngle1);
      servo2.write(mappedAngle2);
  }
}
