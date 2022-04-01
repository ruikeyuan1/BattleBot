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

//-------------------------------WIFI (--------------------------

// Set these to your desired credentials.
const char *ssid = "yourAP";
const char *password = "yourPassword";

WiFiServer server(80);
String header;
//-------------------------------WIFI )--------------------------

#define SOUND_SPEED 0.034
Adafruit_VL53L0X lox = Adafruit_VL53L0X();


//Motor Pins
int RB = 5;
int RF = 18;
int LB = 17;
int LF = 16;

//Ultrasonic Sensor Pins for Race
const int rightT = 19;
const int rightE = 23;
const int leftT = 12;
const int leftE = 14;

//Light Sensor Pins
int leftSensor = 39;
int rightSensor = 34;

//Ultrasonic Sensor Pins for Maze
const int TrigPin = 26;
const int EchoPin = 25;



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

    pinMode(TrigPin, OUTPUT);
    pinMode(EchoPin, INPUT);

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


void loop()
{
    if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            Linetracking lt;
            Race race;
            Maze maze;
            Game4 game4;

             if (header.indexOf("GET /Lt") >= 0) {
                lt.execute();
            } else if (header.indexOf("GET /Maze") >= 0) {
                maze.execute();
            } else if (header.indexOf("GET /Race") >= 0) {
                race.execute();
            } else if (header.indexOf("GET /Ctf") >= 0) {
                game4.execute();
            } else if (header.indexOf("GET /Go") >= 0) {
                goForward(0);
            } else if (header.indexOf("GET /Back") >= 0) {
                goBackward(0);
            } else if (header.indexOf("GET /RL") >= 0) {
                rL(0);
            } else if (header.indexOf("GET /RR") >= 0) {
                rR(0);
            } else if (header.indexOf("GET /Stop") >= 0) {
                goStop(0);
            } else if (header.indexOf("GET /Exit") >= 0) {
              client.stop();
            }
            
  

            
            // the content of the HTTP response follows the header:
            client.print("<button><a href=\"/Lt\">Linetracking</a></button><br>");
            client.print("<button><a href=\"/Maze\">Maze</a></button><br>");
            client.print("<button><a href=\"/Race\">Race</a></button><br>");
            client.print("<button><a href=\"/Ctf\">Capture The Flag</a></button><br>");
            client.print("<button><a href=\"/Go\">Go</a></button><br>");
            client.print("<button><a href=\"/Back\">Back</a></button><br>");
            client.print("<button><a href=\"/RL\">Rotate Left</a></button><br>");
            client.print("<button><a href=\"/RR\">Rotate Right</a></button><br>");
            client.print("<button><a href=\"/Stop\">Stop</a></button><br>");
            client.print("<button><a href=\"/Exit\">Exit</a></button><br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":


       
        
      }
    }
    // close the connection:
      
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
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
                goLeft(0, increase);
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
        if (Distance() > 200) {
            stopRobot();
            delay(100);
            Forward();
        } else if(ultrasonic()>25 && ultrasonic()<125) {
            stopRobot();
            delay(250);
            turnRight();
            delay(350);
        } else {
            stopRobot();
            delay(250);
            turnLeft();
            delay(350);
        }
    }  

    int Distance() {

        VL53L0X_RangingMeasurementData_t measure;

        lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

        if (measure.RangeStatus != 4) {  // phase failures have incorrect data 
            return (measure.RangeMilliMeter);} 
            else {
            return -1;}
        }

    int ultrasonic() {    

        digitalWrite(TrigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(TrigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(TrigPin, LOW);
        int distance = pulseIn(EchoPin, HIGH)/58;
        Serial.print(distance);
        Serial.print( "cm" );
        Serial.println();
        return distance;
        //delay(1000);
        }

};

class Game4
{


    void execute()
    {

    }  

};