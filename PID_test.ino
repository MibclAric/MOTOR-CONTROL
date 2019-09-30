#include <DueTimer.h>

#include <Encoder.h>
#include <CytronMotorDriver.h>
#include <PID_v1.h>

CytronMD motor1(PWM_DIR, 5, 4);  // PWM 1 = Pin 5, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 6, 7); // PWM 2 = Pin 6, DIR 2 = Pin 7.

Encoder WRight(22,23);
Encoder WLeft(24,25);

float    LOPosition = 0;
float    ROPosition = 0;

double Lspeed, Lturn, Rspeed, Rturn;
double Kp=1 ;double Ki=0.001 ;double Kd=0.5;
double LOutput, ROutput, setspeed;
double maxspeed = 25;

  PID leftPID(&Lspeed, &LOutput, &setspeed,Kp,Ki,Kd,DIRECT);
  PID rightPID(&Rspeed, &ROutput, &setspeed,Kp,Ki,Kd,DIRECT);
  

    
void setup() {
  Serial.begin(9600);
  setspeed = 20;
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);

  Timer1.attachInterrupt(measureSpeed).start(500000);
  
}

void loop() {

//  motor1.setSpeed(255);
//   motor2.setSpeed(255);

     leftPID.Compute();
     rightPID.Compute();
   
   motor1.setSpeed((LOutput/maxspeed)*255);
   motor2.setSpeed((ROutput/maxspeed)*255);


   Serial.print("Before PID   ");
   Serial.print(Lspeed); Serial.print("  ");
   Serial.println(Rspeed);
   Serial.print("after PID   ");
   Serial.print(LOutput);Serial.print("  ");
   Serial.println(ROutput);

}

void measureSpeed(){

       float    LNPosition = WLeft.read();
       float    RNPosition = - WRight.read();

     Lspeed = 3*((LNPosition - LOPosition)*60/(5880*0.5))*0.10472; //in cm/s
     Lturn = LNPosition/5880;
     Rspeed = 3*((RNPosition - ROPosition)*60/(5880*0.5))*0.10472;
     Rturn = RNPosition/5880;
//     Serial.print(LOPosition);
//     Serial.println(LNPosition);
//     Serial.print(ROPosition);
//     Serial.println(RNPosition);
      


        LOPosition = LNPosition;
        ROPosition = RNPosition;
}
