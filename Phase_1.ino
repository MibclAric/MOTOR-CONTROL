#include <DueTimer.h>
#include <Encoder.h>
#include <CytronMotorDriver.h>

double sensorDist;

CytronMD motor1(PWM_DIR, 5, 4);  // PWM 1 = Pin 5, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 6, 7); // PWM 2 = Pin 6, DIR 2 = Pin 7.

Encoder WRight(22,23);   //encoder pins
Encoder WLeft(24,25);   //encoder pins

double Ldist = 0;  //left wheel distance traveled
double Rdist = 0; //right wheel distance traveled

double LSetSpeed ; double LOutput; //Input Speed to PID and Outpus Speed from PID
double RSetSpeed ; double ROutput;

double OldLeftSpeed = 0; //PID variables
double LeftDerivative  = 0;
double LeftErrorSum = 0;

double OldRightSpeed = 0; //PID variables
double RightDerivative = 0;
double RightErrorSum = 0;

double LSpeed = 0; double RSpeed = 0; //PID variables
double Kp=15 ;double Ki=3 ;double Kd=1;
double Kp2=15;double Ki2=3;double Kd2=1;

   
  double Lerror = 0;  //PID variables
  double Lcontrol = 0;
  double LFeedForward = 0;

  double Rerror = 0; //PID variables
  double Rcontrol = 0;
  double RFeedForward = 0;


//*****************************************************************************//
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); 

  pinMode(A0, INPUT); //IR Sensor Pin A0

  Timer1.attachInterrupt(TimerInterrupt); //Timer Interrupt for PID
  Timer1.start(100000); //every 0.1 seconds
  
//  Timer2.attachInterrupt(speedCorrection);
//  Timer2.start(500000); //every 0.5 seconds
  //Task Code
  delay(1000);
    long currentDist = 0;
    int Task1 = 0;
    startSpeed(15);
while(Task1<41){
  LSetSpeed = 15;
  RSetSpeed = 15;
  speedCorrection();
  Task1 = ticksToCM(WLeft.read());
  Serial1.print("distance");
  Serial1.println(ticksToCM(WLeft.read()));
 
  delay(50);
}
setStop();
delay(500);
rightTurn();
delay(500);

int Task2 = 0;
    startSpeed(15);
 currentDist = ticksToCM(WLeft.read());
while(Task2-currentDist<=120){
  LSetSpeed = 15;
  RSetSpeed = 15;
  speedCorrection();
  Task2 = ticksToCM(WLeft.read());
 
  delay(50);
}
setStop();
delay(500);
rightTurn();
delay(500);

int Task3 = 0;
    startSpeed(15);
 currentDist = ticksToCM(WLeft.read());
while(Task3-currentDist<=55){
  LSetSpeed = 15;
  RSetSpeed = 15;
  speedCorrection();
  Task3 = ticksToCM(WLeft.read());
 
  delay(50);
}
setStop();

delay(500);
rightTurn();
delay(500);

int Task4 = 0;
    startSpeed(15);
 currentDist = ticksToCM(WLeft.read());
while(Task4-currentDist<=143){
  LSetSpeed = 15;
  RSetSpeed = 15;
  speedCorrection();
  Task4 = ticksToCM(WLeft.read());
 
  delay(50);
}
setStop();
delay(500);
rightTurn();
delay(500);
//*******************Lower Level************************//
int Task5 = 0;
    startSpeed(15);
 currentDist = ticksToCM(WLeft.read());
while(Task5-currentDist<=57){
  LSetSpeed = 15;
  RSetSpeed = 15;
  speedCorrection();
  Task5 = ticksToCM(WLeft.read());
 
  delay(50);
}
setStop();
delay(500);
rightTurn();
delay(500);

int Task6 = 0;
    startSpeed(15);
 currentDist = ticksToCM(WLeft.read());
while(Task6-currentDist<=13){
  LSetSpeed = 15;
  RSetSpeed = 15;
  speedCorrection();
  Task6 = ticksToCM(WLeft.read());
 
  delay(50);
}
setStop();
delay(500);
rightTurn();
delay(500);

int Task7 = 0;
    startSpeed(15);
 currentDist = ticksToCM(WLeft.read());
while(Task7-currentDist<=12){
  LSetSpeed = 15;
  RSetSpeed = 15;
  speedCorrection();
  Task7 = ticksToCM(WLeft.read());
 
  delay(50);
}
setStop();
delay(500);
leftTurn();
delay(500);

int Task8 = 0;
    startSpeed(15);
 currentDist = ticksToCM(WLeft.read());
while(Task8-currentDist<=31){
  LSetSpeed = 15;
  RSetSpeed = 15;
  speedCorrection();
  Task8 = ticksToCM(WLeft.read());
 
  delay(50);
}
setStop();
delay(500);
leftTurn();
delay(500);

int Task9 = 0;
    startSpeed(15);
 currentDist = ticksToCM(WLeft.read());
while(Task9-currentDist<=13){
  LSetSpeed = 15;
  RSetSpeed = 15;
  speedCorrection();
  Task9 = ticksToCM(WLeft.read());
 
  delay(50);
}
setStop();
delay(500);
rightTurn();
delay(500);

int Task10 = 0;
    startSpeed(15);
 currentDist = ticksToCM(WLeft.read());
while(Task10-currentDist<=50){
  LSetSpeed = 15;
  RSetSpeed = 15;
  speedCorrection();
  Task10 = ticksToCM(WLeft.read());
 
  delay(50);
}
setStop();
  //Task Code
 
}

//*****************************************************************************//
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

//*****************************************************************************//

float ticksToCM(long encoder_val) //return from encoder value to CM
{
  return encoder_val/297.2545932;
}

void measureSpeeds() //measure speed by encoder
{
  double Ldist_now = ticksToCM(WLeft.read());
  double Rdist_now = ticksToCM(-WRight.read());
  LSpeed = (Ldist_now-Ldist)/0.1;  //cm/s
  RSpeed = (Rdist_now-Rdist)/0.1;
  Ldist = Ldist_now;
  Rdist = Rdist_now;
}   

void controlSpeeds() //PID
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

float SpeedToPWM(float des_speed) //return from desired speed to PWM
{
  return des_speed*des_speed*des_speed*des_speed*.0025 - 0.1099*des_speed*des_speed*des_speed +1.676*des_speed*des_speed -6.728*des_speed +38.228;
}

int filterMotorSpeed(float input_speed) //sign correction for input speed
{
  if(input_speed > 255)
   return 255;
  if(input_speed < -255)
   return -255;

  return (int)input_speed;
}

void sensorRead(){ //read distance from IR sensor and convert to cm
   if(analogRead(A0)>200){
  sensorDist = -0.088*(analogRead(A0))+64.393;
 }
 else sensorDist = 5;
}

void speedCorrection(){ //wall follower
  sensorRead();
  if(sensorDist<5){
    LSetSpeed = LSetSpeed - 1.25;
        Serial1.println("Left - - ");
 
  }
  else if(sensorDist>5&&sensorDist<10){
    RSetSpeed = RSetSpeed - 1.25;
       Serial1.println("Right - - ");
  }
  Serial1.println(sensorDist);
   
}

//*****************************************************************************//
void leftTurn(){
    float currentLRead =ticksToCM( WLeft.read());
    setStop();
  while(ticksToCM(WLeft.read())-currentLRead >= - 8.95){
    RSetSpeed = 5;
    LSetSpeed = -5;
  }
  setStop();
}
void rightTurn(){
    float currentRRead =ticksToCM( - WRight.read());
    setStop();
  while(ticksToCM(-WRight.read())-currentRRead >= - 8.95){
    RSetSpeed = -10;
    LSetSpeed = 10;
  setStop();
}
void setStop(){
  for(int c=0; c<20;c++){
  RSetSpeed = 0;
  LSetSpeed = 0;
  }
}
void startSpeed(int speedset){
  for(int d=0;d<=speedset;d++){
      RSetSpeed = d;
      LSetSpeed = d;
      delay(50);
  }
}
