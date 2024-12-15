#include <Servo.h>

Servo myservo;  // Create servo object
int pos = 25;   // Initial position
int increment = 1;  // How much to move each step

void setup() {
  myservo.attach(7);  // Attaches the servo on pin 9
  myservo.write(105); // 20-105
}

void loop() {
  // Move the servo
  // myservo.write(pos);
  
  // Update position
  // pos += increment;
  
  // Change direction when limits are reached
  // if (pos >= 65) {
  //   increment = -5;  // Start moving backwards
  // }
  // if (pos <= 25) {
  //   increment = 5;   // Start moving forwards
  // }
  
  delay(1000);  // Small delay for smooth movement
}