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
int distanceF;

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
void left()
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 180);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 0);

}


void right()
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 0);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 180);
}

void sleft()
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 180);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 100);

}


void sright()
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 100);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 180);
}


void ssleft()
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 210);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 180);

}


void ssright()
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 180);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 210);
}


void rotateRight()
{
  ledcWrite(RBC, 180);
  ledcWrite(RFC, 0);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 180);
  
}

void rotateLeft()
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 180);
  ledcWrite(LBC, 180);
  ledcWrite(LFC, 0);
  
}

void racegoForward(int x)
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, x);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, x);
  
}

void goLeft(int x)
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 200+x);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 180);
  
}

void goRight(int x)
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 180);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 200+x);
  
}

void goStop()
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 0);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 0);
  
}
//




void setup() {
  Serial.begin(115200);

 
//-------------------------------WIFI )--------------------------

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  pinMode(rightT, OUTPUT); // Sets the rightT as an Output
  pinMode(rightE, INPUT); // Sets the rightE as an Input
  pinMode(leftT, OUTPUT); // Sets the rightT as an Output
  pinMode(leftE, INPUT); // Sets the rightE as an Input
  lox.begin();

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
    
    digitalWrite(rightT, LOW);
    delayMicroseconds(2);
    digitalWrite(rightT, HIGH);
    delayMicroseconds(10);
    digitalWrite(rightT, LOW);
  
    durationR = pulseIn(rightE, HIGH);
    distanceR = durationR * SOUND_SPEED/2;
    
    
    digitalWrite(leftT, LOW);
    delayMicroseconds(2);
    digitalWrite(leftT, HIGH);
    delayMicroseconds(10);
    digitalWrite(leftT, LOW);

    durationL = pulseIn(leftE, HIGH);
    distanceL = durationL * SOUND_SPEED/2;

    distanceF = measure.RangeMilliMeter/10;
    int x = 1;
    int increase;

    
    if(distanceL < 10 && distanceF < 10 && distanceR < 10 ) {
            
            x = 0;
            goStop();

        } else if(distanceF > distanceR && distanceF > distanceL) {

            x = 0;
            if(distanceF > 55) {
              racegoForward(255);
            } else if(distanceF > 44) {
              racegoForward(200);
            } else if(distanceF > 22) {
              racegoForward(180);
            } else if(distanceF <= 22) {
              racegoForward(150);
            }
            
            if(distanceL <= 11 && distanceR >= 33) {
                ssright();
            } else if(distanceR <= 11 && distanceL >= 33) {
                ssleft(); 
            }

        } else if(distanceR > distanceL) {

            if(distanceF > 55) {

                increase = sqrt(x);
                goRight(increase);
                if(!x>2010){(x+=(distanceF > 100)?(5):(500-distanceF));}

            } else if(distanceF > 44) {
                x = 0;
                sright(); 
            } else if(distanceF > 22) {
                x = 0;
                right();
            } else if(distanceF <= 22) {
                x = 0;
                rotateRight();
            }

        } else if(distanceL > distanceR) {
            
            if(distanceF > 55) {

                increase = sqrt(x);
                goLeft(increase);
                if(!x>2010){(x+=(distanceF > 100)?(5):(500-distanceF));}

            } else if(distanceF > 44) {
                x = 0;
                sleft();
            } else if(distanceF > 22) {
                x = 0;
                left();
            } else if(distanceF <= 22) {
                x = 0;
                rotateLeft();
            }
        }
    }

  


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
