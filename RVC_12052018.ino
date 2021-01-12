#include <AFMotor.h>

AF_DCMotor motor(1), motor2(2), motor3(3), motor4(4);  // '1' denotes M1 in which the DC motor is connected to. '2' means M2 etc.

// Global variables
int g_flag;
const int trigPinN = 40; // Yellow cable
const int echoPinN = 41; // Green cable
const int trigPinE = 42; // Yellow cable
const int echoPinE = 43; // Green cable
const int trigPinW = 44; // Yellow cable
const int echoPinW = 19; // Green cable. Interrupt pin.
const int trigPinS = 46; // Yellow cable
const int echoPinS = 47; // Green cable
const int greenLED = 24; // Indicates 'within range'
const int redLED = 30;   // Indicates 'out of range'

float durationN;
float distanceN;
float durationE;
float distanceE;
float durationS;
float distanceS;
float durationW;
float distanceW;

void setup()
{
  // Declarations
  pinMode(trigPinN, OUTPUT);
  pinMode(echoPinN, INPUT);
  pinMode(trigPinE, OUTPUT);
  pinMode(echoPinE, INPUT);
  pinMode(trigPinS, OUTPUT);
  pinMode(echoPinS, INPUT);
  pinMode(trigPinW, OUTPUT);
  pinMode(echoPinW, INPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
//  attachInterrupt(digitalPinToInterrupt(19), distanceW_ISR, FALLING);
  Serial.begin(9600);  
  
  // Turn on motors
  motor.setSpeed(200),  motor.run(RELEASE); // Valid values for 'speed' are between 0 and 255 with 0 being off and 255 as full throttle.
  motor2.setSpeed(200), motor2.run(RELEASE);
  motor3.setSpeed(200), motor3.run(RELEASE);  
  motor4.setSpeed(200), motor4.run(RELEASE);
}

// Movement functions
void motorsForward()
{
 int i;
 motor.run(FORWARD), motor2.run(BACKWARD), motor3.run(BACKWARD), motor4.run(FORWARD); // Forwards

 for (i=200; i<=200; i++)
 {
  motor.setSpeed(i), motor2.setSpeed(i), motor3.setSpeed(i), motor4.setSpeed(i);
 }
}

void motorsBackward()
{
 int i;
 motor.run(BACKWARD), motor2.run(FORWARD), motor3.run(FORWARD), motor4.run(BACKWARD); // Backwards

 for (i=200; i<=200; i++)
 {
  motor.setSpeed(i), motor2.setSpeed(i), motor3.setSpeed(i), motor4.setSpeed(i);
 }
}

void motorsStrafeLeft()
{
 int i;
 motor.run(FORWARD), motor2.run(FORWARD), motor3.run(BACKWARD), motor4.run(BACKWARD); // Strafe left

 for (i=200; i<=200; i++)
 {
  motor.setSpeed(i), motor2.setSpeed(i), motor3.setSpeed(i), motor4.setSpeed(i);
 }
}

void motorsStrafeRight()
{
 int i;
 motor.run(BACKWARD), motor2.run(BACKWARD), motor3.run(FORWARD), motor4.run(FORWARD); // Strafe right

 for (i=200; i<=200; i++)
 {
  motor.setSpeed(i), motor2.setSpeed(i), motor3.setSpeed(i), motor4.setSpeed(i);
 }
}

void motorsAxisTurnCW()
{
 int i;
 motor.run(BACKWARD), motor2.run(BACKWARD), motor3.run(BACKWARD), motor4.run(BACKWARD); // Rotate clockwise (on central axis)

 for (i=200; i<=200; i++)
 {
  motor.setSpeed(i), motor2.setSpeed(i), motor3.setSpeed(i), motor4.setSpeed(i);
 }
}

void motorsAxisTurnCCW()
{
 int i;
 motor.run(FORWARD), motor2.run(FORWARD), motor3.run(FORWARD), motor4.run(FORWARD); // Rotate counter-clockwise (on central axis)

 for (i=200; i<=200; i++)
 {
  motor.setSpeed(i), motor2.setSpeed(i), motor3.setSpeed(i), motor4.setSpeed(i);
 }
}

void motorsNorthWest()
{
 int i;
 motor.run(FORWARD), motor2.run(RELEASE), motor3.run(BACKWARD), motor4.run(RELEASE); // North-west (direction)

 for (i=200; i<=200; i++)
 {
  motor.setSpeed(i), motor2.setSpeed(i), motor3.setSpeed(i), motor4.setSpeed(i);
 }
}

void motorsNorthEast()
{
 int i;
 motor.run(RELEASE), motor2.run(BACKWARD), motor3.run(RELEASE), motor4.run(FORWARD); // North-east (direction)

 for (i=200; i<=200; i++)
 {
  motor.setSpeed(i), motor2.setSpeed(i), motor3.setSpeed(i), motor4.setSpeed(i);
 }
}

void motorsSouthWest()
{
 int i;
 motor.run(RELEASE), motor2.run(FORWARD), motor3.run(RELEASE), motor4.run(BACKWARD); // South-west (direction)

 for (i=200; i<=200; i++)
 {
  motor.setSpeed(i), motor2.setSpeed(i), motor3.setSpeed(i), motor4.setSpeed(i);
 }
}

void motorsSouthEast()
{
 int i;
 motor.run(BACKWARD), motor2.run(RELEASE), motor3.run(FORWARD), motor4.run(RELEASE); // South-east (direction)

 for (i=200; i<=200; i++)
 {
  motor.setSpeed(i), motor2.setSpeed(i), motor3.setSpeed(i), motor4.setSpeed(i);
 }
}

void motorsStop()
{
 motor.run(RELEASE), motor2.run(RELEASE), motor3.run(RELEASE), motor4.run(RELEASE);
}

void cleaningSchemeL()
{
  // while(...); // add condition
  // {
    motorsStrafeLeft();
    delay(1000); // Adjust according to distance covered
    
    do
    {
      motorsBackward();
    }
    while(distanceN < distanceS && distanceS == !5);
    
//    delay(500);
    
    if(distanceS < distanceN && distanceS == 5)
    {
      motorsStrafeLeft();
      delay(1000); // Adjust according to distance covered
    }    
    
//    motorsStop();
//    delay(500);
    
//    do
//    {
//      motorsForward();
//    }
//    while(distanceS < distanceN && distanceN == !5);
//      
////    delay(500);
//    
//    if(distanceN < distanceS && distanceN == 5)
//    {
//      motorsStop();
      delay(500);
//    }
//  }
}

void cleaningSchemeR() // Delete if unecessary
{
  // while(...); // add condition
  // {
    motorsStrafeLeft();
    delay(1000); // Adjust according to distance covered
    
    do
    {
      motorsForward();
    }
    while(distanceN < distanceS && distanceS == !5);
    
//    delay(500);
    
    if(distanceS < distanceN && distanceS == 5)
    {
      motorsStrafeLeft();
      delay(1000); // Adjust according to distance covered
    }
    delay(500);
}

void distanceW_ISR()
{
  motorsStrafeRight();
}

void loop()
{
  // Sensor facing north, N
  digitalWrite(trigPinN, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinN, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinN, LOW);

  durationN = pulseIn(echoPinN, HIGH);
  distanceN = (durationN * 0.0343)/2; // 0.0343 is the speed of sound in centimetres per microsecond

  // Sensor facing east, E
  digitalWrite(trigPinE, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinE, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinE, LOW);

  durationE = pulseIn(echoPinE, HIGH);
  distanceE = (durationE * 0.0343)/2; // 0.0343 is the speed of sound in centimetres per microsecond

  // Sensor facing south, S
  digitalWrite(trigPinS, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinS, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinS, LOW);

  durationS = pulseIn(echoPinS, HIGH);
  distanceS = (durationS * 0.0343)/2; // 0.0343 is the speed of sound in centimetres per microsecond

  // Sensor facing West, W
  digitalWrite(trigPinW, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinW, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinW, LOW);

  durationW = pulseIn(echoPinW, HIGH);
  distanceW = (durationW * 0.0343)/2; // 0.0343 is the speed of sound in centimetres per microsecond

  // ??? Don't understand this code!
  if(distanceN < 5)
  {
    digitalWrite(redLED, HIGH); // ON
    digitalWrite(greenLED, LOW); // OFF

    if(g_flag == 1) // Statement shouldn't run as g_flag is still 0.
    {
      cleaningSchemeL();
//      motorsStop();
//      delay(500);
      motorsBackward();
    }
    else
    {
      digitalWrite(redLED, LOW); // OFF
      digitalWrite(greenLED, HIGH); // ON
      g_flag = 1;
      motorsForward();
    }
  }
  if(distanceE < 5) // MEASURE distance from outer wheel face to side of chassis and ADJUST accordingly
  {
    digitalWrite(redLED, HIGH); // ON
    digitalWrite(greenLED, LOW); // OFF

    if(g_flag == 1) // Statement shouldn't run as g_flag is still 0.
    {
      motorsStrafeLeft();
    }
    else
    {
      digitalWrite(redLED, LOW); // OFF
      digitalWrite(greenLED, HIGH); // ON
      g_flag = 1;
      motorsStrafeRight();
      // Need some sort of interrupt here so robot returns to cleaningScheme() routine.
    }
  }
  if(distanceS < 5)
  {
    digitalWrite(redLED, HIGH); // ON
    digitalWrite(greenLED, LOW); // OFF

    if(g_flag == 1) // Statement shouldn't run as g_flag is still 0.
    {
      cleaningSchemeR();
      motorsForward();
    }
    else
    {
      digitalWrite(redLED, LOW); // OFF
      digitalWrite(greenLED, HIGH); // ON
      g_flag = 1;
      motorsBackward();
    }
  }
    if(distanceW < 5) // MEASURE distance from outer wheel face to side of chassis and ADJUST accordingly.
  {
    digitalWrite(redLED, HIGH); // ON
    digitalWrite(greenLED, LOW); // OFF

    if(g_flag == 1) // Statement shouldn't run as g_flag is still 0.
    {
      // Need some sort of interrupt here.
//      distanceW_ISR();
      motorsStrafeRight();
    }
    else
    {
      digitalWrite(redLED, LOW); // OFF
      digitalWrite(greenLED, HIGH); // ON
      g_flag = 1;
      motorsStrafeLeft();
    }
  }

//  if (distanceN >= 100 || distanceN <=0)
//  {
//    Serial.println("Out of range!");
//  }
//  else
//  {
//  Serial.print(distanceN);
//  Serial.println(" cm");
//  }
  delay(100);
}

