/*MR Dir- 7
   MR PWM- 9
   ML Dir- 8
   ML PWM- 10
   Red LED - 6
   Start button - 11


   Line sensor on A0,A1,A2,A3,A4
   A0-left & A4 - right
*/
// Less than threshold = black , more than threshhold = white

#include <Arduino.h>

const int startButton = 11;

// Variables for turning
bool left = 0;
bool right = 0;
bool straight = 0;
bool uturn = 0;
int e = 0;
int paths = 0;

bool endFound = 0;

// Sensor variables
int blackValue = 200;
int whiteValue = 1000;
int threshold = blackValue + whiteValue * 0.5;
int leftMostSensor;
int leftMidSensor;
int rightMidSensor;
int rightMostSensor;
// int threshold = (blackValue + whiteValue) * 0.5;
int FT = 50;
int leftspeed = 100;
int rightspeed = 100;
int lfspeed = 190; // Line follow(forward) speed
int turnspeed;
int turnspeed1 = 140;
int turnspeed2 = 90;

// PID Variables
int P, D, I, previousError, PIDvalue, error;
float Kp = 0.04;
float Kd = 0.05;
float Ki = 0;

String str;

void setup()
{
  // Debug
  Serial.begin(9600);
  // Start Button
  pinMode(startButton, INPUT_PULLUP);
  // Motors
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  // LED
  pinMode(6, OUTPUT); // red
}

void lightsoff()
{
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
}

void red()
{
  digitalWrite(5, LOW);
  digitalWrite(6, HIGH); // RED
}

void PID()
{
  int error = leftMidSensor - rightMidSensor;

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  leftspeed = lfspeed - PIDvalue;
  rightspeed = lfspeed + PIDvalue;

  if (leftspeed > 250)
  {
    leftspeed = 250;
  }
  if (leftspeed < 0)
  {
    leftspeed = 0;
  }
  if (rightspeed > 250)
  {
    rightspeed = 250;
  }
  if (rightspeed < 0)
  {
    rightspeed = 0;
  }

  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  analogWrite(9, leftspeed);
  analogWrite(10, rightspeed);
}

void botleft()
{
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);
  analogWrite(9, turnspeed1);
  analogWrite(10, turnspeed1);
  delay(100);
  while (leftMidSensor < threshold)
  {
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
    analogWrite(9, turnspeed2);
    analogWrite(10, turnspeed2);
  }
  analogWrite(9, 0);
  analogWrite(10, 0);
  delay(50);
}

void botright()
{
  digitalWrite(7, LOW);
  digitalWrite(8, HIGH);
  analogWrite(9, turnspeed1);
  analogWrite(10, turnspeed1);
  delay(100);
  while (rightMidSensor < threshold)
  {
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    analogWrite(9, turnspeed2);
    analogWrite(10, turnspeed2);
  }
  analogWrite(9, 0);
  analogWrite(10, 0);
  delay(50);
}

void botstraight()
{
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  analogWrite(9, lfspeed);
  analogWrite(10, lfspeed);
}

void botinchforward()
{
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  analogWrite(9, turnspeed1);
  analogWrite(10, turnspeed1);
  delay(10);
}
void botstop()
{
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  analogWrite(9, 0);
  analogWrite(10, 0);
}
void botuturn()
{
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);
  analogWrite(9, (lfspeed * 0.7 * 0.8));
  analogWrite(10, lfspeed * 0.7);
  delay(250);
  while (leftMidSensor < threshold)
  {
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
    analogWrite(9, turnspeed2 * 0.8);
    analogWrite(10, turnspeed2);
  }
  analogWrite(9, 0);
  analogWrite(10, 0);
  delay(50);
}

void checknode()
{
  left = 0;
  right = 0;
  straight = 0;
  uturn = 0;
  e = 0;
  paths = 0;

  // checks whethere bot is on node and the number of exits possible

  if (rightMostSensor > threshold)
    right = 1;
  if (leftMostSensor > threshold)
    left = 1;
  if ((leftMostSensor < threshold && (rightMostSensor < threshold) && (leftMidSensor < threshold) && (rightMidSensor < threshold)))
  {
    uturn = 1;
  }
  if ((leftMidSensor > threshold) && (rightMidSensor) && (rightMidSensor > threshold) && (rightMostSensor > threshold))
  {
    e = 1;
  }

  if (uturn == 0)
  {
    for (int i = 0; i < FT; i++)
    {
      lfspeed = 90;
      PID();
      if (rightMostSensor > threshold)
        right = 1;
      if (leftMostSensor > threshold)
        left = 1;
    }

    for (int i = 0; i < FT; i++)
    {
      lfspeed = 90;
      PID();
      if ((leftMidSensor > threshold) && (rightMidSensor > threshold))
        straight = 1;
    }
    if ((e == 1) && (rightMidSensor > threshold) && (rightMostSensor > threshold) && (leftMidSensor > threshold) && (rightMidSensor))
      e = 2;
  }
  if (uturn == 1)
  {
    for (int i = 0; i < 3; i++)
    {
      botinchforward();
    }
  }

  paths = left + straight + right;
}

void linefollow()
{
  paths = 0;
  while ((leftMostSensor < threshold) && (rightMostSensor < threshold) && (leftMidSensor > threshold || rightMidSensor > threshold))
  {
    lfspeed = 190;
    PID();
  }
  lightsoff();
}

void readSensors()
{
  leftMostSensor = analogRead(A0);
  leftMidSensor = analogRead(A1);
  rightMidSensor = analogRead(A2);
  rightMostSensor = analogRead(A3);
}

void reposition()
{
  if (e == 2)
  {
    str += 'E';
    endFound = 1;
    red();
    botstop();
    delay(2000);
  }
  else if (right == 1)
  {
    if (paths > 1)
      str += 'R';
    botright(); // take left
  }

  else if (straight == 1)
  {
    if (paths > 1)
      str += 'S';
  }
  else if (left == 1)
  {
    if (paths > 1)
      str += 'L';
    botleft();
  }

  else if (uturn == 1)
  {
    str += 'U';
    botuturn();
  }

  lightsoff();
}

void loop()
{
  readSensors();
  while (digitalRead(startButton) == 1)
  { // Do nothing while waiting for button press
  }
  delay(300);

  while (endFound == 0)
  {
    linefollow();
    checknode();

    botstop();
    delay(90);

    reposition();
  }

  for (int x = 0; x < 10; x++)
  {
    str.replace("RURUS", "U");
    str.replace("RUSUR", "U");
    str.replace("RUR", "S");
    str.replace("SUR", "L");
    str.replace("RUS", "L");
    str.replace("RUL", "U");
    str.replace("LUR", "U");
  }
  int endpos = str.indexOf('E');

  while (digitalRead(startButton) == 1)
  { // Do nothing while waiting for button press
  }

  lightsoff();
  delay(300);

  for (int i = 0; i <= endpos; i++)
  {
    char node = str.charAt(i);
    paths = 0;
    while (paths < 2)
    {
      linefollow();
      checknode();
      if (paths == 1)
      {
        botstop();
        delay(75);
        reposition();
      }
    }
    switch (node)
    {
    case 'L':
      botstop();
      delay(75);
      botleft();
      break;

    case 'S':
      break;

    case 'R':
      botstop();
      delay(75);
      botright();
      break;

    case 'E':
      for (int i = 0; i < 10; i++)
      {
        botinchforward();
      }
      red();
      botstop();
      delay(1000);
      break;
    } //_________end of switch
  }   //_________end of for loop
}