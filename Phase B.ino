#include <DueTimer.h>
#include <Encoder.h>
#include <CytronMotorDriver.h>

double sensorDist;
CytronMD motor1(PWM_DIR, 5, 4);  // PWM 1 = Pin 5, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 6, 7); // PWM 2 = Pin 6, DIR 2 = Pin 7.

Encoder WRight(24,25); Encoder WLeft(23,22);
double Ldist = 0;double Rdist = 0;

double LSetSpeed ; double LOutput; double RSetSpeed ; double ROutput;

double OldLeftSpeed = 0;double LeftDerivative  = 0;double LeftErrorSum = 0;
double OldRightSpeed = 0;double RightDerivative = 0;double RightErrorSum = 0;

double LSpeed = 0; double RSpeed = 0;
double Kp=10;double Ki=5 ;double Kd=0;double Kp2=10;double Ki2=5;double Kd2=0;

double Lerror = 0;  double Lcontrol = 0;double LFeedForward = 0;
double Rerror = 0;double Rcontrol = 0;double RFeedForward = 0;


//*****************************************************************************//
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  char pose1='a'; char pose2='b'; char pose3 = 'c';
  Timer1.attachInterrupt(TimerInterrupt);
  Timer1.start(100000); //every 0.1 seconds
  //********************START*********************??

  delay(2000);

task(42,13);
setStop();
delay(200);
rightTurn();

task(42,13);
setStop();
delay(200);
leftTurn();


task(62,13);
setStop();
delay(200);
leftTurn();


task(36,13);
setStop();
rightTurn();
delay(500);
frontCheck(730);
rightTurn();

  Serial1.write(pose1);
  delay(100);
//*******************Lower Level************************//
task(50,6);
frontCheck(700);
rightTurn();
 Serial1.write(pose2);
 delay(200);
task(130,10);
frontCheck(700);
rightTurn();


task(63,13);
setStop();
rightTurn();

task(5,13);
frontCheck(730);
rightTurn();

task(30,13);
frontCheck(700);
leftTurn();

task(82,13);
setStop();
leftTurn();

task(40,13);
setStop();
leftTurn();
Serial1.write(pose3);
delay(500);

}
//*****************************************************************************//
void loop() {
 
  }
void TimerInterrupt()
{
  measureSpeeds();
  controlSpeeds();
}

//*****************************************************************************//

float ticksToCM(long encoder_val)
{
  return encoder_val/297.2545932;
}

void measureSpeeds()
{
  double Ldist_now = ticksToCM(WLeft.read());
  double Rdist_now = ticksToCM(WRight.read());
  LSpeed = (Ldist_now-Ldist)/0.1;  //cm/s
  RSpeed = (Rdist_now-Rdist)/0.1;
  Ldist = Ldist_now;
  Rdist = Rdist_now;
}   

void controlSpeeds()
{
  Lerror = LSetSpeed - LSpeed;
  LeftDerivative = (LSpeed-OldLeftSpeed);
  OldLeftSpeed = LSpeed;
  LeftErrorSum += Lerror;

  Rerror = RSetSpeed - RSpeed;
  RightDerivative = (RSpeed-OldRightSpeed);
  OldRightSpeed = RSpeed;
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
 else sensorDist = 13;
}

void speedCorrection(){
  sensorRead();
  if(sensorDist<13){
    LSetSpeed = LSetSpeed - 1.2;
    Serial.println(sensorDist);
  }
  else if(sensorDist>13&&sensorDist<18){
    RSetSpeed = RSetSpeed - 1.2;
    Serial.println(sensorDist);
  } 
}

//*****************************************************************************//
void leftTurn(){
    float currentRRead =ticksToCM(WRight.read());
    float currentLRead =ticksToCM( WLeft.read());
    setStop();
  while(((ticksToCM(WRight.read())-currentRRead)+(currentLRead-ticksToCM(WLeft.read())))/2 < 9){
    RSetSpeed = 8;
    LSetSpeed = -8;
  }
  setStop();
  delay(300);
}

void rightTurn(){
    float currentLRead =ticksToCM( WLeft.read());
    float currentRRead =ticksToCM(WRight.read());
    setStop();
  while(((ticksToCM(WLeft.read())-currentLRead)+(currentRRead-ticksToCM(WRight.read())))/2 <9){
    RSetSpeed = -8;
    LSetSpeed = 8;
  }
  setStop();
  delay(300);
}

void setStop(){
  RSetSpeed = 0;
  LSetSpeed = 0;
//  Serial1.println("Stopped");
}

void startSpeed(int speedset){
  for(int d=0;d<=speedset;d++){
      RSetSpeed = d;
      LSetSpeed = d;
      delay(50);
  }
}
void frontCheck(int frontset){
  while(analogRead(A1)<=frontset){
    RSetSpeed = LSetSpeed = 6+5*((frontset-analogRead(A1))/(frontset));
  // Serial1.println(analogRead(A1));
  }
  setStop();
  delay(300);
}
void task(int taskdist,int taskspeed){
  long currentDist = ticksToCM(WLeft.read());
  int Task = 0;
  startSpeed(taskspeed);
while(Task-currentDist<=taskdist){
    LSetSpeed = taskspeed;
    RSetSpeed = taskspeed;
    speedCorrection();
    Task = ticksToCM(WLeft.read());
    delay(80);
//    Serial1.print("Distance is : ");
//    Serial1.println(Task);
  }
  delay(300);
}
