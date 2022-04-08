//-------------------------------WIFI (--------------------------
#include <WiFi.h>
//-------------------------------WIFI )--------------------------
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL53L0X.h"
#include <math.h>
#include <vector>
#include <numeric>
#include <analogWrite.h>
#include<esp_wifi.h>
#define SOUND_SPEED 0.034
#define GET_REQUEST_ARRAY_SIZE(array) ((sizeof(array))/(sizeof(array[0])))
//-------------------------------WIFI (--------------------------

// Set these to your desired credentials.

//-------------------------------WIFI )--------------------------

boolean flagG = false;

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
int middleSensor = 36;
int rightSensor = 34;

//Ultrasonic Sensor Pins for Maze
const int TrigPin = 2;
const int EchoPin = 4;



void goStop(int d);
void goForward(int d);
void goBackward(int d);
void turnRight(int d, int x);
void turnLeft(int d, int x);
void rotateLeft(int d);
void rotateRight(int d);
void left(int d);
void right(int d);













class Linetracking
{
  public: 
    
    bool executed = false;
    int MAX = 0;
    int MIN = 9999;
    int AVERAGE;
    int increaseSpeed = 0; 

    void goStop(int d)
    {
        analogWrite(RB, 0);
        analogWrite(RF, 0);
        analogWrite(LB, 0);
        analogWrite(LF, 0);
        if(d!=0)
        {
            delay(d);
        }
    }

    void goForward(int d)
    {
        analogWrite(RB, 0);
        analogWrite(RF, 170);
        analogWrite(LB, 0);
        analogWrite(LF, 170);
        if(d!=0)
        {
            delay(d);
        }
    }

    void goBackward(int d)
    {
        analogWrite(RB, 170);
        analogWrite(RF, 0);
        analogWrite(LB, 170);
        analogWrite(LF, 0);
        if(d!=0)
        {
            delay(d);
        }
    }

    void turnRight(int d, int x)
    {
        analogWrite(RB, 0);
        analogWrite(RF, 170);
        analogWrite(LB, 0);
        analogWrite(LF, (175+x));
        if(d!=0)
        {
            delay(d);
        }
    }

    void turnLeft(int d, int x)
    {
        analogWrite(RB, 0);
        analogWrite(RF, (175+x));
        analogWrite(LB, 0);
        analogWrite(LF, 170);
        if(d!=0)
        {
            delay(d);
        }
    }

    void rotateLeft(int d)
    {
        analogWrite(RB, 0);
        analogWrite(RF, 170);
        analogWrite(LB, 180);
        analogWrite(LF, 0);
        if(d!=0)
        {
            delay(d);
        }
    }

    void rotateRight(int d)
    {
        analogWrite(RB, 180);
        analogWrite(RF, 0);
        analogWrite(LB, 0);
        analogWrite(LF, 170);
        if(d!=0)
        {
            delay(d);
        }
    }

    void right(int d)
    {
        analogWrite(RB, 0);
        analogWrite(RF, 0);
        analogWrite(LB, 0);
        analogWrite(LF, 170);
        if(d!=0)
        {
            delay(d);
        }
    }

    void left(int d)
    {
        analogWrite(RB, 0);
        analogWrite(RF, 170);
        analogWrite(LB, 0);
        analogWrite(LF, 0);
        if(d!=0)
        {
            delay(d);
        }
    }
    
    void scanMap()
    {
        goStop(2000);
        int RS = analogRead(rightSensor);
        int LS = analogRead(leftSensor);
        std::vector<int> MIN_array;
        while(RS<200) {
            if(RS>MAX){MAX = RS;}
            if(LS>MAX){MAX = LS;}
            if(RS<100){MIN_array.push_back(RS);}
            if(LS<100){MIN_array.push_back(RS);}
            rotateLeft(0);
            LS = analogRead(leftSensor);
            RS = analogRead(rightSensor);
        }
        goStop(100);
        LS = analogRead(leftSensor);
        RS = analogRead(rightSensor);
        while(RS<500) {
            rotateRight(0);
            LS = analogRead(leftSensor);
            RS = analogRead(rightSensor);
        }
        MIN = (std::accumulate(MIN_array.begin(), MIN_array.end(), 0))/MIN_array.size();
        AVERAGE = (MAX+MIN)/2;
        goStop(2000);
    }


    void execute() {

        if(!executed)
        {
        executed = true;
        scanMap();
        }
        if(analogRead(rightSensor)>(MAX-(MAX/4)) && analogRead(leftSensor)>(MAX-(MAX/4))) { // BOTH SENSORS ON BLACK - STOP
            goStop(0);
        } else if(analogRead(leftSensor) >= (MAX-(AVERAGE))) { // LEFT SENSOR ON BLACK - ROTATE LEFT
            increaseSpeed = 0;
            left(0);  
        } else if(analogRead(rightSensor) >= (MAX-(AVERAGE))) { // RIGHT SENSOR ON BLACK - ROTATE RIGHT  
            increaseSpeed = 0;
            right(0); 
        } else if(analogRead(leftSensor) >= (MIN*3)) { // LEFT SENSOR APPROACHING BLACK - TURN LEFT
            if(increaseSpeed<=5500){increaseSpeed+=5;} //gradually increase speed of one wheel
            turnLeft(0, sqrt(increaseSpeed));
        } else if(analogRead(rightSensor) >= (MIN*3)) { // RIGHT SENSOR APPROACHING BLACK - TURN RIGHT
            if(increaseSpeed<=5500){increaseSpeed+=5;} //gradually increase speed of one wheel
            turnRight(0, sqrt(increaseSpeed));
        } else if(analogRead(leftSensor) < (MIN*3) && analogRead(rightSensor) < (MIN*3)) { // BOTH SENSORS ON WHITE - GO
            increaseSpeed = 0;
            goForward(0);    
        } 
    } 

    void execute2() {

        if(!executed)
        {
        executed = true;
        scanMap();
        }
        if(analogRead(rightSensor)>(MAX-(MAX/4)) && analogRead(leftSensor)>(MAX-(MAX/4))) { // BOTH SENSORS ON BLACK - STOP
            goStop(0); 
        } else if(analogRead(leftSensor) >= (MAX-(AVERAGE))) { // LEFT SENSOR APPROACHING BLACK - TURN LEFT
            left(0);
        } else if(analogRead(rightSensor) >= (MAX-(AVERAGE))) { // RIGHT SENSOR APPROACHING BLACK - TURN RIGHT
            right(0);
        } else if(analogRead(leftSensor) < (MAX-(AVERAGE)) && analogRead(rightSensor) < (MAX-(AVERAGE))) { // BOTH SENSORS ON WHITE - GO
            goForward(0);    
        } 
    }

};


//class Game4
//{
//  public: 
//
//    void execute()
//    {
//
//    }  
//
//};




















void goForward() {
    analogWrite(RB, 0);
    analogWrite(RF, 170);
    analogWrite(LB, 0);
    analogWrite(LF, 170);  
}

void goBack() {
    analogWrite(RB, 170);
    analogWrite(RF, 0);
    analogWrite(LB, 170);
    analogWrite(LF, 0);
}

void rL() {
    analogWrite(RB, 0);
    analogWrite(RF, 170);
    analogWrite(LB, 170);
    analogWrite(LF, 0);
}

void rR() {
    analogWrite(RB, 170);
    analogWrite(RF, 0);
    analogWrite(LB, 0);
    analogWrite(LF, 170);
}

void goStop() {
    analogWrite(RB, 0);
    analogWrite(RF, 0);
    analogWrite(LB, 0);
    analogWrite(LF, 0);
}

void turnRight() {
    analogWrite(RB, 170);
    analogWrite(RF, 0);
    analogWrite(LB, 0);
    analogWrite(LF, 170);
}

void turnLeft() {
    analogWrite(RB, 0);
    analogWrite(RF, 170);
    analogWrite(LB, 170);
    analogWrite(LF, 0);
}


const char* ssid     = "BattleBot";
const char* password = "43638253";

//const char* host = "battlebot3.serverict.nl/robotControl.php";
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
WiFiServer server(80);
String header;
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

    Serial.begin(115200);

    delay(10);

 
  
    //Stop Motors
    analogWrite(RB, 0);
    analogWrite(RF, 0);
    analogWrite(LB, 0);
    analogWrite(LF, 0);

    while (! Serial) {
    delay(1);
    }

    
     
    if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
    }
    WiFi.mode(WIFI_AP_STA);
//    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(500);
     }
    Serial.println("Connected!\nIP: ");
    Serial.println(WiFi.localIP());
    server.begin();

    
}
















class Maze
{

  public:  

    void rotateRight() {
        int time = millis(); //time=2000;  
        int timeToRotate = 440+time; // 2700
        while(time <= timeToRotate){ // 2000 <= 2700
            turnRight();
            time = millis(); // 2001 2002 2003 ... 2700
            }
    }

    void rotateLeft() {
        int time = millis(); //time=2000;  
        int timeToRotate = 700+time; // 2700
        while(time <= timeToRotate){ // 2000 <= 2700
            turnLeft();
            time = millis(); // 2001 2002 2003 ... 2700
            }
    }

    void Forward(){
        analogWrite(RF, 190);
        analogWrite(LF, 190);
        analogWrite(RB, 0);
        analogWrite(LB, 0);
    }

    void stopRobot(){
        analogWrite(RB, 0);
        analogWrite(LF, 0);
        analogWrite(RF, 0);
        analogWrite(LB, 0);
    }

    void turnRight(){
        analogWrite(RB, 190);
        analogWrite(LF, 190);
        analogWrite(RF, 0);
        analogWrite(LB, 0);      
    }

    void turnLeft(){
        analogWrite(RB, 0);
        analogWrite(LF, 0);
        analogWrite(RF, 190);
        analogWrite(LB, 190);      
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
        return distance;
        //delay(1000);
        }

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

};


class Race {
  public:
    long durationR;
    long durationL;
    float distanceR;
    float distanceL;
    long durationS;
    float distanceS;
    int distanceF;
    int x;
    int increase;

    void racegoForward(int x)
    {
      analogWrite(RB, 0);
      analogWrite(RF, x);
      analogWrite(LB, 0);
      analogWrite(LF, x); 
    }
    
    void left()
    {
    analogWrite(RB, 0);
    analogWrite(RF, 180);
    analogWrite(LB, 0);
    analogWrite(LF, 0);

    }


    void right()
    {
    analogWrite(RB, 0);
    analogWrite(RF, 0);
    analogWrite(LB, 0);
    analogWrite(LF, 180);
    }

    void sleft()
    {
    analogWrite(RB, 0);
    analogWrite(RF, 180);
    analogWrite(LB, 0);
    analogWrite(LF, 100);

    }


    void sright()
    {
    analogWrite(RB, 0);
    analogWrite(RF, 100);
    analogWrite(LB, 0);
    analogWrite(LF, 180);
    }


    void ssleft()
    {
    analogWrite(RB, 0);
    analogWrite(RF, 220);
    analogWrite(LB, 0);
    analogWrite(LF, 180);

    }


    void ssright()
    {
    analogWrite(RB, 0);
    analogWrite(RF, 180);
    analogWrite(LB, 0);
    analogWrite(LF, 220);
    }


    void rotateRight()
    {
    analogWrite(RB, 180);
    analogWrite(RF, 0);
    analogWrite(LB, 0);
    analogWrite(LF, 180);
    
    }

    void rotateLeft()
    {
    analogWrite(RB, 0);
    analogWrite(RF, 180);
    analogWrite(LB, 180);
    analogWrite(LF, 0);
    
    }

    void goLeft(int x)
    {
    analogWrite(RB, 0);
    analogWrite(RF, 200+x);
    analogWrite(LB, 0);
    analogWrite(LF, 180);
    
    }

    void goRight(int x)
    {
    analogWrite(RB, 0);
    analogWrite(RF, 180);
    analogWrite(LB, 0);
    analogWrite(LF, 200+x);
    
    }

    void goStop()
    {
    analogWrite(RB, 0);
    analogWrite(RF, 0);
    analogWrite(LB, 0);
    analogWrite(LF, 0);
    
    }

  
    int getDistanceF() {

        VL53L0X_RangingMeasurementData_t measure;

        lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

        if (measure.RangeStatus != 4) {  // phase failures have incorrect data 
            return (measure.RangeMilliMeter/10);
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

    int getDistanceS() {

        digitalWrite(TrigPin, LOW);
        delayMicroseconds(2);
        // Sets the rightT on HIGH state for 10 micro seconds
        digitalWrite(TrigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(TrigPin, LOW);

        
        // Reads the rightE, returns the sound wave travel time in microseconds
        durationS = pulseIn(EchoPin, HIGH);
        
        // Calculate the distance
        return (durationS * SOUND_SPEED/2);
    }

    void test()
    {
      Serial.println((String) "F: " + getDistanceF());
      Serial.println((String) "L: " + getDistanceL());
      Serial.println((String) "R: " + getDistanceR());
    }

    void execute()
    {
      while(true) {
        VL53L0X_RangingMeasurementData_t measure;

        lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

        if (measure.RangeStatus != 4) {  // phase failures have incorrect data 
            distanceF = measure.RangeMilliMeter/10;
        } 

        digitalWrite(rightT, LOW);
        delayMicroseconds(2);
        // Sets the rightT on HIGH state for 10 micro seconds
        digitalWrite(rightT, HIGH);
        delayMicroseconds(10);
        digitalWrite(rightT, LOW);

        durationR = pulseIn(rightE, HIGH);
        
        // Calculate the distance
        distanceR = (durationR * SOUND_SPEED/2);

        digitalWrite(leftT, LOW);
        delayMicroseconds(2);
        // Sets the rightT on HIGH state for 10 micro seconds
        digitalWrite(leftT, HIGH);
        delayMicroseconds(10);
        digitalWrite(leftT, LOW);

        
        // Reads the rightE, returns the sound wave travel time in microseconds
        durationL = pulseIn(leftE, HIGH);
        
        // Calculate the distance
        distanceL = (durationL * SOUND_SPEED/2);
        
        Serial.println((String) "F: " + distanceF);
        Serial.println((String) "L: " + distanceL);
        Serial.println((String) "R: " + distanceR);
        if(distanceL < 10 && distanceF < 10 && distanceR < 10 ) {
            
            x = 0;
            goStop();

        } else if(distanceF > distanceR && distanceF > distanceL) {

            Serial.println("FFF");
            x = 0;
            if(distanceF > 55) {
              this->racegoForward(255);
            } else if(distanceF > 44) {
              this->racegoForward(200);
            } else if(distanceF > 22) {
              this->racegoForward(180);
            } else if(distanceF <= 22) {
              this->racegoForward(150);
            }
            
            if(distanceL <= 11 && distanceR >= 33) {
                this->right();
            } else if(distanceR <= 11 && distanceL >= 33) {
                this->left(); 
            }

        } else if(distanceR > distanceL) {

            Serial.println("RRR");
            if(distanceF > 55) {

                increase = sqrt(x);
                this->goRight(increase);
                if(!x>2010){(x+=(distanceF > 100)?(5):(500-distanceF));}

            } else if(distanceF > 44) {
                Serial.println("R1");
                x = 0;
                this->sright(); 
            } else if(distanceF > 22) {
                Serial.println("R2");
                x = 0;
                this->right();
            } else if(distanceF <= 22) {
                Serial.println("R3");
                x = 0;
                this->rotateRight();
            }

        } else if(distanceL > distanceR) {

             Serial.println("LLL");
            if(distanceF > 55) {

                increase = sqrt(x);
                this->goLeft(increase);
                if(!x>2010){(x+=(distanceF > 100)?(5):(500-distanceF));}

            } else if(distanceF > 44) {
                Serial.println("L1");
                x = 0;
                this->sleft();
            } else if(distanceF > 22) {
                Serial.println("L1");
                x = 0;
                this->left();
            } else if(distanceF <= 22) {
                Serial.println("L1");
                x = 0;
                this->rotateLeft();
            }
        }
        WiFiClient client = server.available(); 
        if (client) {                             // if you get a client,
            String currentLine = "";                // make a String to hold incoming data from the client
            if (client.connected()) {            // loop while the client's connected
              if (client.available()) {             // if there's bytes to read from the client,
                char c = client.read();             // read a byte, then
//                Serial.write(c);                    // print it out the serial monitor
                if (c == 'G') {  // if you got anything else but a carriage return character,
                  break;      // add it to the end of the currentLine
                }
                
              }
            }
            client.stop();
          }
      }
    }
};













String acceptedRequests[] = 
{"GET /Forward", "GET /Back", "GET /Stop", "GET /TurnLeft", "GET /Turn90Left", "GET /TurnRight", "GET /Turn90Right", "GET /LineTrack", "GET /Maze", "GET /Race", "GET /ScanNetwork", "GET /SendHTTPRequest",
"GET /giveFlag", "GET /findFlag", "!SSIDSearchFor", "GET /turnLeftOrRight", "GET /printCalibration", "GET /calibrateMPU"};

String currentLine;

boolean testIfCommand(String line)
{
    if(line.endsWith("!SSIDSearchFor"))
    {
        return true;
    }
    
    for(int count=0; count < GET_REQUEST_ARRAY_SIZE(acceptedRequests); count++)
    {
        if(acceptedRequests[count] == line)
        {
            return true;
        }
    }
    return false;
}

boolean anotherRequest;
unsigned char TSSID[] = "ESP-32_test_E";
unsigned char* ssidToCatch = TSSID;
Linetracking lt;
Maze maze;
Race race;
void executeGETRequest(String currentLine)
{
          Serial.println((String) "\nyo: " + currentLine);
          if (currentLine.endsWith("GET /Forward")) {
  //            printHTMLServer(client, "The robot moves forward");
                // 0 means it will go forward forever
                Serial.println("test1");
                goForward();
          }
          if (currentLine.endsWith("GET /Back")) {
      //        printHTMLServer(client, "The robot moves backwards");
                // 0 means it will go back forever
                goBack();
          }
          if(currentLine.endsWith("GET /Stop"))
          {
      //        printHTMLServer(client, "The robot has stopped");
                goStop();
          }
          if(currentLine.endsWith("GET /TurnLeft"))
          {
      //        printHTMLServer(client, "The robot has turned left");
                turnLeft();
          }
          if(currentLine.endsWith("GET /TurnRight"))
          {
      //        printHTMLServer(client, "The robot has turned right");
                turnRight();
          }
          if(currentLine.endsWith("GET /LineTrack"))
          {
    //        printHTMLServer(client, "The robot is following the line");
              lt.execute();
          }
    //    if(currentLine.endsWith("GET /Turn90Right"))
    //    {
    //        turnRight90();
    //    }
    //    if(currentLine.endsWith("GET /Turn90Left"))
    //    {
    //        turnLeft90();
    //    }
          if(currentLine.endsWith("GET /Maze"))
          { 
              Serial.println("test2");
              maze.execute();
          }
          if(currentLine.endsWith("GET /Race"))
          {
              race.execute();
          }
    //    if(currentLine.endsWith("GET /ScanNetwork"))
    //    {
    //        getNetworkSignal();
    //    }
    //    if(currentLine.endsWith("GET /SendHTTPRequest"))
    //    {
    //        sendHTTPRequest("stopAll");
    //    }
    //    if(currentLine.endsWith("GET /printCalibration"))
    //    {
    //        print_calibration();
    //    }
    //    if(currentLine.endsWith("GET /calibrateMPU"))
    //    {
    //        calibrateMPU();
    //    }
    //
    //    // Game specific - Capture the flag
    //    if(currentLine.endsWith("GET /giveFlag"))
    //    {
    //        hasFlag = true;
    //        digitalWrite(led, HIGH);
    //        digitalWrite(LED_BUILTIN, HIGH);
    //        // If the robot has the flag, it can just "play" a normal maze
    //        //maze();
    //    }
    //    if(currentLine.endsWith("GET /findFlag"))
    //    {
    //        hasFlag = false;
    //        digitalWrite(led, LOW);
    //        digitalWrite(LED_BUILTIN, LOW);
    //        findFlag();
    //    }
    //    if(currentLine.endsWith("GET /turnLeftOrRight"))
    //    {
    //        turnLeftOrRight();
    //    }
          if(currentLine.startsWith("GET /") && currentLine.endsWith("!SSIDSearchFor")){
                // Get the middle ssid here and assign to the ssidToCatch variable
                int ssidLength = currentLine.length();
                ssidToCatch = (unsigned char*) currentLine.substring(5, ssidLength - 14).c_str();
                Serial.println("");
      //        Serial.println(ssidToCatch.toString());
          }
}

void loop()
{   

  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to turn the LED on pin 5 on.<br>");
            client.print("Click <a href=\"/L\">here</a> to turn the LED on pin 5 off.<br>");

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

//        Serial.println((String) "test: " + currentLine);
        if(testIfCommand(currentLine)) {
            client.stop();
            Serial.println();
            Serial.println("Client Disconnected.");
            executeGETRequest(currentLine);
            break;
        }
        
      }
    }
    // close the connection:
//    client.stop();
//    Serial.println("Client Disconnected.");
  }
}
