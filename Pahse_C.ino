#include <DueTimer.h>
#include <Encoder.h>
#include <CytronMotorDriver.h>
#include <Servo.h>

Servo servoLeft;
int pos;
double sensorDist;
CytronMD motor1(PWM_DIR, 5, 4);  // PWM 1 = Pin 5, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 6, 7); // PWM 2 = Pin 6, DIR 2 = Pin 7.

Encoder WRight(24,25); Encoder WLeft(23,22);
double Ldist = 0;double Rdist = 0;
double olddist = 0;

double LSetSpeed ; double LOutput; double RSetSpeed ; double ROutput;

double OldLeftSpeed = 0;double LeftDerivative  = 0;double LeftErrorSum = 0;
double OldRightSpeed = 0;double RightDerivative = 0;double RightErrorSum = 0;

double LSpeed = 0; double RSpeed = 0;
double Kp=10;double Ki=5 ;double Kd=0;double Kp2=10;double Ki2=5;double Kd2=0;

double Lerror = 0;  double Lcontrol = 0;double LFeedForward = 0;
double Rerror = 0;double Rcontrol = 0;double RFeedForward = 0;

const int trigPin = 12;
const int echoPin = 13;
long duration;
int ultdistance;

//*****************************************************************************//
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  servoLeft.attach(A2);
  servoLeft.write(5);  
  char pose1='a'; char pose2='b'; char pose3 = 'c';
  Timer1.attachInterrupt(TimerInterrupt);
 Timer1.start(100000); //every 0.1 seconds
 
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  //********************START*********************//

  //delay(20000000);


task(33,10);
setStop();
delay(300);
leftTurn();

delay(300);
task(43,10);
setStop();
delay(300);
rightTurn();
setStop();
delay(300);

task(20,5);
setStop();

//************************BEFORE SCOOP******************************//
 
for(pos =0;pos<150;pos++){
servoLeft.write(5+pos);
delay(10);
}
task(13,5);
tasknc(15,5);
setStop();


for(pos =0;pos<50;pos++){
servoLeft.write(55-pos);
delay(60);
}

delay(1000);

backtask(40,-5);
setStop();
delay(300);

leftTurn();
setStop();
  //********************AFTER PICK UP*********************//

delay(300);
taskft(85,13,7);
setStop();
delay(300);
rightTurn();
delay(300);


taskft(50,5,5);
setStop();
delay(300);
rightTurn();
delay(300);

taskft(125,10,10);
setStop();
delay(300);
rightTurn();
delay(300);

//*******************Lower Level************************//

task(67,10);
setStop();
delay(300);
rightTurn();

task(43,10);
setStop();
delay(300);
rightTurn();

task(43,10);
setStop();
delay(300);
leftTurn();

task(44,10);
setStop();
delay(300);
leftTurn();

task(40,10);
setStop();

for(pos =0;pos<180;pos++){
servoLeft.write(5+pos);
delay(10);
}
}
//*****************************************************************************//
void loop() {

  }
void TimerInterrupt()
{
  measureSpeeds();
  controlSpeeds();
}

//********************************DO NOT CHANGE*********************************************//

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
//*************************************SPEED FUNCTION****************************************//
void sensorRead(){
   if(analogRead(A0)>200){
    float sensorDist_old;
  sensorDist = -0.088*(analogRead(A0))+64.393;
 // sensorDist=sensorDist_old;
  Serial2.println(sensorDist);
//  for (int x =0;x<200;x++){
//    sensorDist = sensorDist + sensorDist_old;
 }
//  sensorDist = sensorDist/200;
//  Serial2.println(sensorDist);
// }
 else if (analogRead(A0)<30){ sensorDist = 7.5;
 Serial2.println("Error");
 }
}

float backsensorRead(){
     float backsensorDist;
   if(analogRead(A5)>200){
     backsensorDist = -0.088*(analogRead(A5))+64.393;
   }
 else if (analogRead(A5)<30){ 
  backsensorDist = 4.5;
 }
return(backsensorDist);
}

void speedCorrection(){
  sensorRead();
  if(sensorDist<4){
        LSetSpeed = LSetSpeed - 1.2;

    Serial2.println("correct Left");
  }
   if(sensorDist>5&&sensorDist<9){
    
    RSetSpeed = RSetSpeed - 1.2;
    Serial2.println("near right");
  } 
}
void backspeedCorrection(){

  if(backsensorRead()<4){
      
    LSetSpeed = LSetSpeed + 1.2;
    //Serial.println("LSetSpeed");
  }
   if(backsensorRead()>5 && backsensorRead()<9){
    
    

     RSetSpeed = RSetSpeed + 1.2;
    Serial2.println("near right");
  } 
}


void leftTurn(){
    float currentRRead =ticksToCM(WRight.read());
    float currentLRead =ticksToCM( WLeft.read());
    setStop();
  while(((ticksToCM(WRight.read())-currentRRead)+(currentLRead-ticksToCM(WLeft.read())))/2 < 9){
    RSetSpeed = 6;
    LSetSpeed = -6;
  }
  setStop();
  delay(300);
}

void rightTurn(){
    float currentLRead =ticksToCM( WLeft.read());
    float currentRRead =ticksToCM(WRight.read());
    setStop();
  while(((ticksToCM(WLeft.read())-currentLRead)+(currentRRead-ticksToCM(WRight.read())))/2 <8.6){
    RSetSpeed = -6;
    LSetSpeed = 6;
  }
  setStop();
  delay(50);
}

void setStop(){
  RSetSpeed = -3;
  LSetSpeed = -3;
  delay(3);
  
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
  int frontDist;
  
  Serial2.println("Checking");
  while(ultSensor()>frontset){
    Serial2.print("Sensor Distance: ");
    Serial2.println(ultSensor());
  
}
setStop();
}

//***********************************TASK FUNCTION***********************************//
void task(int taskdist,int taskspeed){
  long currentDist = (ticksToCM(WLeft.read())+ticksToCM(WRight.read()))/2;
  int Task = 0;
  startSpeed(taskspeed);
while((Task-currentDist)<taskdist){
    LSetSpeed = taskspeed;
    RSetSpeed = taskspeed;
    speedCorrection();
    Task = (ticksToCM(WLeft.read())+ticksToCM(WRight.read()))/2;
    delay(30);
//    Serial2.print("Right  ");
//    Serial2.println(RSetSpeed);
//    Serial2.print("Left  ");
//    Serial2.println(LSetSpeed);
//    Serial2.print("Distance is : ");
//    Serial2.println((Task-currentDist));
  }
}

void backtask (int taskdist,int taskspeed){
  long currentDist = (ticksToCM(WLeft.read())+ticksToCM(WRight.read()))/2;
  int Task = currentDist;
    for(int d=0;d>=taskspeed;d--){
      RSetSpeed = d;
      LSetSpeed = d;
      delay(50);
  }
while(-(Task-currentDist)<taskdist){
    LSetSpeed = taskspeed;
    RSetSpeed = taskspeed;
 backspeedCorrection();
    Task = (ticksToCM(WLeft.read())+ticksToCM(WRight.read()))/2;
    delay(50);
  }
}

int ultSensor(){
  
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    ultdistance= duration*0.034/2;
    // Prints the distance on the Serial Monitor
    delay(50);
    return(ultdistance);
}

void tasknc(int taskdist,int taskspeed){
  long currentDist = (ticksToCM(WLeft.read())+ticksToCM(WRight.read()))/2;
  int Task = 0;
  int pos = 0;
while((Task-currentDist)<taskdist){
  
    if(pos<100){
    pos++;
    servoLeft.write(155-pos);
    }
    speedCorrection();
    LSetSpeed = taskspeed;
    RSetSpeed = taskspeed;
    Task = (ticksToCM(WLeft.read())+ticksToCM(WRight.read()))/2;
    delay(50);
  }
}

void taskft(int taskdist,int taskspeed,int frontset){
  long currentDist = (ticksToCM(WLeft.read())+ticksToCM(WRight.read()))/2;
  int Task = 0;
  startSpeed(taskspeed);
while((Task-currentDist)<taskdist){
    LSetSpeed = taskspeed;
    RSetSpeed = taskspeed;
    speedCorrection();
    Task = (ticksToCM(WLeft.read())+ticksToCM(WRight.read()))/2;
    delay(60);
  }
frontCheck(frontset);
}
