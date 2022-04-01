//-------------------------------WIFI (--------------------------
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
//-------------------------------WIFI )--------------------------
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL53L0X.h"
#include <math.h>
#include <vector>
#include <numeric>
#include <analogWrite.h>

#define SOUND_SPEED 0.034

//Motor Pins
int RB = 5;
int RF = 18;
int LB = 17;
int LF = 16;

//Ultrasonic Pins for Race
const int rightT = 19;
const int rightE = 23;
const int leftT = 12;
const int leftE = 14;

//Light Sensor Pins
int leftSensor = 39;
int rightSensor = 34;




void setup()
{
    pinMode(RB, OUTPUT);
    pinMode(RF, OUTPUT);
    pinMode(LB, OUTPUT);
    pinMode(LF, OUTPUT);

    pinMode(rightT, OUTPUT); // Sets the rightT as an Output
    pinMode(rightE, INPUT); // Sets the rightE as an Input
    pinMode(leftT, OUTPUT); // Sets the rightT as an Output
    pinMode(leftE, INPUT);

    pinMode (leftSensor, INPUT);
    pinMode (rightSensor, INPUT);

    if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
    }

    analogWrite(RB, 0);
    analogWrite(RF, 0);
    analogWrite(LB, 0);
    analogWrite(LF, 0);

    delay(200);
}





class Linetracking
{
    bool executed = false;
    int MAX = 0;
    int MIN = 9999;
    int AVERAGE;
    int increaseSpeed = 0;

    void execute()
    {

    } 

    void scanMap()
    {

    }

};

class Race
{   
    long durationR;
    long durationL;
    float distanceR;
    float distanceL;
    int distanceF;

    void execute()
    {
        distanceF = getDistanceF();
        distanceL = getDistanceL();
        distanceR = getDistanceR();
        if(distanceL < 10 && distanceF < 10 && distanceR < 10 ) {
            
            x = 0;
            goStop();

        } else if(distanceF > distanceR && distanceF > distanceL) {

            x = 0;
            racegoForward(0);
            if(distanceL <= 11 && distanceR >= 33) {
                ssright();
            } else if(distanceR <= 11 && distanceL >= 33) {
                ssleft(); 
            }

        } else if(distanceR > distanceL) {

            if(distanceF > 55) {

                increase = sqrt(x);
                goRight(0, increase);
                if(!x>2010){(x+=(distanceF > 100)?(5):(100-distanceF));}

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
                goLeft(0, increase);
                if(!x>2010){(x+=(distanceF > 100)?(5):(100-distanceF));}

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

    int getDistanceF() {

        VL53L0X_RangingMeasurementData_t measure;

        lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

        if (measure.RangeStatus != 4) {  // phase failures have incorrect data 
            return (measure.RangeMilliMeter);
        } else {
            return -1;
        }
    }

    int getDistanceR() {

        digitalWrite(rightT, LOW);
        delayMicroseconds(2);
        // Sets the rightT on HIGH state for 10 micro seconds
        digitalWrite(rightT, HIGH);
        delayMicroseconds(10);
        digitalWrite(rightT, LOW);

        durationR = pulseIn(rightE, HIGH);
        
        // Calculate the distance
        return (durationR * SOUND_SPEED/2);
    }

    int getDistanceL() {

        digitalWrite(leftT, LOW);
        delayMicroseconds(2);
        // Sets the rightT on HIGH state for 10 micro seconds
        digitalWrite(leftT, HIGH);
        delayMicroseconds(10);
        digitalWrite(leftT, LOW);

        
        // Reads the rightE, returns the sound wave travel time in microseconds
        durationL = pulseIn(leftE, HIGH);
        
        // Calculate the distance
        return (durationL * SOUND_SPEED/2);
    }

};

class Maze
{


    void execute()
    {

    }  

};

class Game4
{


    void execute()
    {

    }  

};