 #include "CytronMotorDriver.h"


// Configure the motor driver.
CytronMD motor1(PWM_DIR, 5, 4);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 6, 7); // PWM 2 = Pin 9, DIR 2 = Pin 10.


// The setup routine runs once when you press reset.
void setup() {
  
}


// The loop routine runs over and over again forever.
void loop() {
  motor1.setSpeed(-255);   // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(-255);  // Motor 2 runs backward at 50% speed.
  delay(10000);
  
  motor1.setSpeed(125);   // Motor 1 runs forward at full speed.
  motor2.setSpeed(-125);  // Motor 2 runs backward at full speed.
  delay(1100);

  motor1.setSpeed(-255);     // Motor 1 stops.
  motor2.setSpeed(-255);     // Motor 2 stops.
  delay(10000);

 
}
