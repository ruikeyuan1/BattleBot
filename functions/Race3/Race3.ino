/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-hc-sr04-ultrasonic-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();


Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

const int rightT = 19;
const int rightE = 23;

const int leftT = 12;
const int leftE = 14;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034

long durationR;
long durationL;
float distanceR;
float distanceL;

int leftSensor = 39;
int rightSensor = 34;

//Motors Pin
int RB = 5;
int RF = 18;
int LB = 17;
int LF = 16;

//Assign Motors to Channels
const int RBC = 0;
const int RFC = 1;
const int LBC = 2;
const int LFC = 3;

const int freq = 4000;
const int resolution = 8;

//Function to scan the colors and its MIN and MAX values
bool executed = false;
int MAX = 0;
int MIN = 9999;
int AVERAGE;
void scanMap();

//Main function
int LS, RS;
int increaseSpeed = 0;
int fixx = 0;
void track(int LS, int RS);

//Movement functions
void goStop(int d);
void goForward(int d);
void goBackward(int d);
void turnRight(int d, int x);
void turnLeft(int d, int x);
void rotateLeft(int d);
void rotateRight(int d);
void left(int d);
void right(int d);
void linetracking();
void scanMap();
void maze();
void race();
void ctf();
void displ();




void setup() {
  Serial.begin(115200);

 
//-------------------------------WIFI )--------------------------

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  pinMode(rightT, OUTPUT); // Sets the rightT as an Output
  pinMode(rightE, INPUT); // Sets the rightE as an Input
  pinMode(leftT, OUTPUT); // Sets the rightT as an Output
  pinMode(leftE, INPUT); // Sets the rightE as an Input
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  //Initiate INPUT for Color Sensors
  pinMode (leftSensor, INPUT);
  pinMode (rightSensor, INPUT);

  //Attach Motors to Channels
  ledcAttachPin(RB, RBC);
  ledcAttachPin(RF, RFC);
  ledcAttachPin(LB, LBC);
  ledcAttachPin(LF, LFC);

  //Setup the Channels
  ledcSetup(RBC, freq, resolution);
  ledcSetup(RFC, freq, resolution);
  ledcSetup(LBC, freq, resolution);
  ledcSetup(LFC, freq, resolution);

  //Stop Motors
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 0);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 0);
  
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  // Clears the rightT
  digitalWrite(rightT, LOW);
  delayMicroseconds(2);
  // Sets the rightT on HIGH state for 10 micro seconds
  digitalWrite(rightT, HIGH);
  delayMicroseconds(10);
  digitalWrite(rightT, LOW);

  durationR = pulseIn(rightE, HIGH);
  
  // Calculate the distance
  distanceR = durationR * SOUND_SPEED/2;
  
  
  digitalWrite(leftT, LOW);
  delayMicroseconds(2);
  // Sets the rightT on HIGH state for 10 micro seconds
  digitalWrite(leftT, HIGH);
  delayMicroseconds(10);
  digitalWrite(leftT, LOW);

  
  // Reads the rightE, returns the sound wave travel time in microseconds
  durationL = pulseIn(leftE, HIGH);
  
  // Calculate the distance
  distanceL = durationL * SOUND_SPEED/2;

  int distanceF = measure.RangeMilliMeter/10;
   Serial.println();
   Serial.println("-------------");
   Serial.println();
   if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Forward: "); Serial.print(distanceF);
    } else {
      Serial.println(" out of range ");
    }


  // Prints the distance in the Serial Monitor
  Serial.print(" || Right: ");
  Serial.print(distanceR);
  Serial.print(" cm ||");
  Serial.print("Left: ");
  Serial.print(distanceL);
  Serial.print(" cm");
    int disp = 8;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Left: ");
    display.print(distanceL);
    display.setCursor(0, disp);
    display.print("Right: ");
    display.print(distanceR);
    display.setCursor(0, disp*2);
    display.print("Forward: ");
    display.print(distanceF);
    display.display();
    Serial.println();
    int x = 1;
    int increase;

    Serial.println("0");
    if(distanceL < 10 && distanceF < 10 && distanceR < 10 )
    {
        x = 0;
        Serial.println("1.4");
        goStop();
    }
    else if(distanceF > distanceR && distanceF > distanceL)
    {
       x = 0;
       racegoForward();
    }
    else if(distanceF > 60)
    {
      x = 0;
      racegoForward();
    }
    else if(distanceF > 44)
    {
      Serial.println("1");
      if(distanceF > distanceR && distanceF > distanceL)
      {
        Serial.println("1.1");
        x = 0;
        racegoForward();
      }
  
      else if(distanceR > distanceL)
      {
        Serial.println("1.2");
        increase = sqrt(x);
        goRight(increase);
        if(!x>2010){(x+=(distanceF > 100)?(5):(100-distanceF));}
      }
  
      else if(distanceL > distanceR)
      {
        Serial.println("1.3");
        increase = sqrt(x);
        goLeft(increase);
        if(!x>2010){(x+=(distanceF > 100)?(5):(100-distanceF));}  
      }
      else if(distanceL < 10 && distanceF < 10 && distanceR < 10 )
      {
        x = 0;
        Serial.println("1.4");
        goStop();
      }
    } 
    else if(distanceF > 25)
    {
      x = 0;
      Serial.println("2");
      if(distanceF > distanceR && distanceF > distanceL)
      {
        Serial.println("2.1");
        x = 0;
        racegoForward();
      }
  
      else if(distanceR > distanceL)
      {
        x = 0;
        Serial.println("2.2");
        right();
      }
  
      else if(distanceL > distanceR)
      {
        x = 0;
        Serial.println("2.3");
        left(); 
      }
      else if(distanceL < 10 && distanceF < 10 && distanceR < 10 )
      { 
        x = 0;
        Serial.println("2.4");
        goStop();
      }
    }
    else if(distanceF <= 25)
    {
      x = 0;
      Serial.println("3");
      if(distanceF > distanceR && distanceF > distanceL)
      {
        Serial.println("2.1");
        x = 0;
        racegoForward();
      }
      else if(distanceR > distanceL)
      {
        x = 0;
        Serial.println("3.1");
        rotateRight();
      }
  
      else if(distanceL > distanceR)
      {
        x = 0;
        Serial.println("3.2");
        rotateLeft(); 
      }
      else if(distanceL < 10 && distanceF < 10 && distanceR < 10 )
      { 
        x = 0;
        Serial.println("3.3");
        goStop();
      }
    }
 
  
    Serial.println();
   Serial.println("-------------");
   Serial.println();

  
}

void left()
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 200);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 120);

}


void right()
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 120);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 200);
}

void rotateRight()
{
  ledcWrite(RBC, 200);
  ledcWrite(RFC, 0);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 200);
  
}

void rotateLeft()
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 200);
  ledcWrite(LBC, 200);
  ledcWrite(LFC, 0);
  
}

void racegoForward()
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 200);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 200);
  
}

void goLeft(int x)
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 210+x);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 200);
  
}

void goRight(int x)
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 200);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 210+x);
  
}

void goStop()
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 0);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 0);
  
}
//
//void displ()
//{
//    
//    int disp = 8;
//    display.clearDisplay();
//    display.setTextSize(1);
//    display.setTextColor(SSD1306_WHITE);
//    display.setCursor(0, 0);
//    display.print("Left: ");
//    display.print(analogRead(distanceL));
//    display.setCursor(0, disp);
//    display.print("Right: ");
//    display.print(analogRead(distanceR));
//    display.setCursor(0, disp*2);
//    display.print("Forward: ");
//    display.print(measure.RangeMilliMeter);
//    display.display();
//}
