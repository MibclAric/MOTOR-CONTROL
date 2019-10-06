#include <DueTimer.h>
#include <Encoder.h>
#include <CytronMotorDriver.h>

double sensorDist;

CytronMD motor1(PWM_DIR, 5, 4);  // PWM 1 = Pin 5, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 6, 7); // PWM 2 = Pin 6, DIR 2 = Pin 7.

Encoder WRight(22,23);
Encoder WLeft(24,25);

double Ldist = 0;
double Rdist = 0;

double LSetSpeed ; double LOutput;
double RSetSpeed ; double ROutput;

double OldLeftSpeed = 0;
double LeftDerivative  = 0;
double LeftErrorSum = 0;

double OldRightSpeed = 0;
double RightDerivative = 0;
double RightErrorSum = 0;

double LSpeed = 0; double RSpeed = 0;
double Kp=15 ;double Ki=2 ;double Kd=1;
double Kp2=20;double Ki2=1.5;double Kd2=1;

   
  double Lerror = 0; 
  double Lcontrol = 0;
  double LFeedForward = 0;

  double Rerror = 0;
  double Rcontrol = 0;
  double RFeedForward = 0;

    
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); 

  pinMode(A0, INPUT);

  Timer1.attachInterrupt(TimerInterrupt);
  Timer1.start(10000); //every 0.05 seconds

  //Task Code

  

RSetSpeed = 15;
LSetSpeed = 15;





  //Task Code
 
}

void loop() 
{

}

void TimerInterrupt()
{
  measureSpeeds();
  controlSpeeds();
  
  Serial.print("speedL: ");
  Serial.println(LSpeed);
  Serial.print("speedR: ");
  Serial.println(RSpeed);

}

float ticksToCM(long encoder_val)
{
  return encoder_val/297.2545932;
}

void measureSpeeds()
{
  double Ldist_now = ticksToCM(WLeft.read());
  double Rdist_now = ticksToCM(-WRight.read());
  LSpeed = (Ldist_now-Ldist)/0.1;  //cm/s
  RSpeed = (Rdist_now-Rdist)/0.1;
  Ldist = Ldist_now;
  Rdist = Rdist_now;
}   

void controlSpeeds()
{
  Lerror = LSetSpeed - LSpeed;
  LeftDerivative = (OldLeftSpeed - LSpeed);
  LeftErrorSum += Lerror;

  Rerror = RSetSpeed - RSpeed;
  RightDerivative = (OldRightSpeed - RSpeed);
  RightErrorSum += Rerror;
  
  Lcontrol = Kp*Lerror + Kd*LeftDerivative + Ki*LeftErrorSum;
  Rcontrol = Kp2*Rerror + Kd2*RightDerivative + Ki2*RightErrorSum;

  
  LFeedForward = SpeedToPWM(LSetSpeed);
  RFeedForward = SpeedToPWM(RSetSpeed);
  
  if(LSetSpeed < 0)
  {
    LFeedForward = -SpeedToPWM(-LSetSpeed);
  }
  if(RSetSpeed <0)
  {
    RFeedForward = -SpeedToPWM(-RSetSpeed);
  }
  
  LOutput = LFeedForward + Lcontrol;
  ROutput = RFeedForward + Rcontrol;

  motor2.setSpeed(filterMotorSpeed(LOutput));
  motor1.setSpeed(filterMotorSpeed(ROutput));

}

float SpeedToPWM(float des_speed)
{
  return des_speed*des_speed*des_speed*des_speed*.0025 - 0.1099*des_speed*des_speed*des_speed +1.676*des_speed*des_speed -6.728*des_speed +38.228;
}

int filterMotorSpeed(float input_speed)
{
  if(input_speed > 255)
   return 255;
  if(input_speed < -255)
   return -255;

  return (int)input_speed;
}

void sensorRead(){
   if(analogRead(A0)>200){
  sensorDist = -0.088*(analogRead(A0))+64.393;
 }
 else sensorDist = 2;
}

void speedCorrection(){
  sensorRead();
  if(sensorDist<5){
    LSetSpeed = LSetSpeed - 0.5;
        Serial1.println("Left - - ");
 
  }
  else if(sensorDist>7){
    RSetSpeed = RSetSpeed - 0.5;
       Serial1.println("Right - - ");
  }
   
}
