#include <WiFi.h>

#include <Adafruit_SSD1306.h>
#include "Adafruit_VL53L0X.h"

#include <math.h>
#include <vector>
#include <numeric>

#include <analogWrite.h>

#include<esp_wifi.h>

#include <iostream>
#include <string>

#define SOUND_SPEED 0.034
#define GET_REQUEST_ARRAY_SIZE(array) ((sizeof(array))/(sizeof(array[0])))

const char* ssid     = "BattleBot";
const char* password = "43638253";

const char* ssidCTF     = "Robot_B";

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
WiFiServer server(80);

String acceptedRequests[] = 
{"GET /Forward", "GET /Back", "GET /Stop", "GET /TurnLeft", "GET /Turn90Left", "GET /TurnRight", "GET /Turn90Right", "GET /LineTrack", "GET /Maze", "GET /Race", "GET /ScanNetwork", "GET /SendHTTPRequest",
"GET /giveFlag", "GET /findFlag", "!SSIDSearchFor", "GET /turnLeftOrRight", "GET /printCalibration", "GET /calibrateMPU"};


//Motor Pins
int RB = 5;
int RF = 18;
int LB = 17;
int LF = 16;

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);


void displ();


void goForward() {
    analogWrite(RB, 0);
    analogWrite(RF, 180);
    analogWrite(LB, 0);
    analogWrite(LF, 180);  
}

void goBack() {
    analogWrite(RB, 180);
    analogWrite(RF, 0);
    analogWrite(LB, 180);
    analogWrite(LF, 0);
}

void rL() {
    analogWrite(RB, 0);
    analogWrite(RF, 180);
    analogWrite(LB, 180);
    analogWrite(LF, 0);
}

void rR() {
    analogWrite(RB, 180);
    analogWrite(RF, 0);
    analogWrite(LB, 0);
    analogWrite(LF, 180);
}

void goStop() {
    analogWrite(RB, 0);
    analogWrite(RF, 0);
    analogWrite(LB, 0);
    analogWrite(LF, 0);
}

void turnRight() {
    analogWrite(RB, 180);
    analogWrite(RF, 0);
    analogWrite(LB, 0);
    analogWrite(LF, 180);
}

void turnLeft() {
    analogWrite(RB, 0);
    analogWrite(RF, 180);
    analogWrite(LB, 180);
    analogWrite(LF, 0);
}

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














void setup() {
  
    pinMode(RB, OUTPUT);
    pinMode(RF, OUTPUT);
    pinMode(LB, OUTPUT);
    pinMode(LF, OUTPUT);

    pinMode(rightT, OUTPUT); 
    pinMode(rightE, INPUT); 
    pinMode(leftT, OUTPUT); 
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
//  esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.begin(ssid, password);
    WiFi.softAP(ssidCTF, "DUCKOFF123");
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(500);
     }
    Serial.println("Connected!\nIP: ");
    Serial.println(WiFi.localIP());
    server.begin();

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

}


int MAX = 0;
int MIN = 9999;
int AVERAGE;

class Linetracking //LTV8_2!!!
{
  public: 
    
    bool executed = false;
    int increaseSpeed = 0; 
    int FULLBLACK = 2300;
    int BLACKK = 1200;
    //int WHITE = ;
    int approachingBLACK = 100;
    //int apprachingWHITE = ;
    int fixx;
    boolean check = false;

    void goStop(int d)
    {
      analogWrite(RB, 0);
      analogWrite(RF, 0);
      analogWrite(LB, 0);
      analogWrite(LF, 0);
      if(d!=0) {
        delay(d);
      }
    }
    
    void goForward()
    {
      analogWrite(RB, 0);
      analogWrite(RF, 180);
      analogWrite(LB, 0);
      analogWrite(LF, 180);
    }
    
    void goBackward()
    {
      analogWrite(RB, 180);
      analogWrite(RF, 0);
      analogWrite(LB, 180);
      analogWrite(LF, 0);
    }
    
    void turnRight(int x)
    {
      analogWrite(RB, 0);
      analogWrite(RF, 180);
      analogWrite(LB, 0);
      analogWrite(LF, (185+x));
    }
    
    void turnLeft(int x)
    {
      analogWrite(RB, 0);
      analogWrite(RF, (185+x));
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
    
    void rotateRight()
    {
      analogWrite(RB, 180);
      analogWrite(RF, 0);
      analogWrite(LB, 0);
      analogWrite(LF, 180);
    }
    
    void ArotateLeft()
    {
      analogWrite(RB, 0);
      analogWrite(RF, 180); //180
      analogWrite(LB, 160); //160
      analogWrite(LF, 0);
    }
    
    void ArotateRight()
    {
      analogWrite(RB, 160); //160
      analogWrite(RF, 0);
      analogWrite(LB, 0);
      analogWrite(LF, 180); //180
    }
    
    void right()
    {
      analogWrite(RB, 0);
      analogWrite(RF, 0);
      analogWrite(LB, 0);
      analogWrite(LF, 180);
    }
    
    void left()
    {
      analogWrite(RB, 0);
      analogWrite(RF, 180);
      analogWrite(LB, 0);
      analogWrite(LF, 0);
    }
    
    void Aright()
    {
      analogWrite(RB, 190);
      analogWrite(RF, 0);
      analogWrite(LB, 0);
      analogWrite(LF, 180);
    }
    
    void Aleft()
    {
      analogWrite(RB, 0);
      analogWrite(RF, 180);
      analogWrite(LB, 190);
      analogWrite(LF, 0);
    }
    
    void Sright()
    {
      analogWrite(RB, 0);
      analogWrite(RF, 100);
      analogWrite(LB, 0);
      analogWrite(LF, 180);
    }
    
    void Sleft()
    {
      analogWrite(RB, 0);
      analogWrite(RF, 180);
      analogWrite(LB, 0);
      analogWrite(LF, 100);
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
            rotateLeft();
            LS = analogRead(leftSensor);
            RS = analogRead(rightSensor);
        }
        goStop(100);
        LS = analogRead(leftSensor);
        RS = analogRead(rightSensor);
        while(RS<500) {
            rotateRight();
            LS = analogRead(leftSensor);
            RS = analogRead(rightSensor);
        }
        MIN = (std::accumulate(MIN_array.begin(), MIN_array.end(), 0))/MIN_array.size();
        AVERAGE = (MAX+MIN)/2;
        goStop(2000);
        Serial.println((String) MIN + " " + MAX);
    }

    
    void fixIt(int flagFix) {
      if(flagFix == 0) {
        while(analogRead(rightSensor)<(MAX-AVERAGE)) {
          rotateRight();
        }
      } else if(flagFix == 1) {
        while(analogRead(leftSensor)<(MAX-AVERAGE)) {
          rotateLeft();
        }
      }
    }

//    void execute() {
//
//        if(!executed)
//        {
//          executed = true;
//          scanMap();
//        }
//
//        while(true) {
//          
//          Serial.println((String) analogRead(rightSensor) + " " + analogRead(leftSensor));
//          if(analogRead(rightSensor)>(MAX-(MAX/4)) && analogRead(leftSensor)>(MAX-(MAX/4))) { // BOTH SENSORS ON BLACK - STOP
//              goStop(0);
//          } else if(analogRead(leftSensor) >= (MAX-(AVERAGE))) { // LEFT SENSOR ON BLACK - ROTATE LEFT
//              increaseSpeed = 0;
//              left1(0);  
//          } else if(analogRead(rightSensor) >= (MAX-(AVERAGE))) { // RIGHT SENSOR ON BLACK - ROTATE RIGHT  
//              increaseSpeed = 0;
//              right1(0); 
//          } else if(analogRead(leftSensor) >= (MIN*3)) { // LEFT SENSOR APPROACHING BLACK - TURN LEFT
//              if(increaseSpeed<=5500){increaseSpeed+=10;} //gradually increase speed of one wheel
//              turnLeft(0, sqrt(increaseSpeed));
//          } else if(analogRead(rightSensor) >= (MIN*3)) { // RIGHT SENSOR APPROACHING BLACK - TURN RIGHT
//              if(increaseSpeed<=5500){increaseSpeed+=10;} //gradually increase speed of one wheel
//              turnRight(0, sqrt(increaseSpeed));
//          } else if(analogRead(leftSensor) < (MIN*3) && analogRead(rightSensor) < (MIN*3)) { // BOTH SENSORS ON WHITE - GO
//              Serial.println("test");
//              increaseSpeed = 0;
//              goForward1(0);    
//          }
//
//          WiFiClient client = server.available(); 
//          if (client) {                             // if you get a client,
//              String currentLine = "";                // make a String to hold incoming data from the client
//              if (client.connected()) {            // loop while the client's connected
//                if (client.available()) {             // if there's bytes to read from the client,
//                  char c = client.read();             // read a byte, then
//  //                Serial.write(c);                    // print it out the serial monitor
//                  if (c == 'G') {  // if you got anything else but a carriage return character,
//                    break;      // add it to the end of the currentLine
//                  }
//                  
//                }
//              }
//              client.stop();
//            }
//          
//        } 
//    } 
//
//    void execute2() {
//
//        if(!executed)
//        {
//        executed = true;
//        scanMap();
//        }
//        while(true) {
//          if(analogRead(rightSensor)>(MAX-(MAX/4)) && analogRead(leftSensor)>(MAX-(MAX/4))) { // BOTH SENSORS ON BLACK - STOP
//              goStop(0); 
//          } else if(analogRead(leftSensor) >= (MAX-(AVERAGE))) { // LEFT SENSOR APPROACHING BLACK - TURN LEFT
//              left2(0);
//          } else if(analogRead(rightSensor) >= (MAX-(AVERAGE))) { // RIGHT SENSOR APPROACHING BLACK - TURN RIGHT
//              right2(0);
//          } else if(analogRead(leftSensor) < (MAX-(AVERAGE)) && analogRead(rightSensor) < (MAX-(AVERAGE))) { // BOTH SENSORS ON WHITE - GO
//              goForward(0);    
//          } 
//
//          WiFiClient client = server.available(); 
//          if (client) {                             // if you get a client,
//              String currentLine = "";                // make a String to hold incoming data from the client
//              if (client.connected()) {            // loop while the client's connected
//                if (client.available()) {             // if there's bytes to read from the client,
//                  char c = client.read();             // read a byte, then
//  //                Serial.write(c);                    // print it out the serial monitor
//                  if (c == 'G') {  // if you got anything else but a carriage return character,
//                    break;      // add it to the end of the currentLine
//                  }
//                  
//                }
//              }
//              client.stop();
//            }
//        }
//    }

    void execute3() {
//
        if(!executed)
        {
        executed = true;
        scanMap();
        }
          if(analogRead(rightSensor)>(MAX-(MAX/4)) && analogRead(leftSensor)>(MAX-(MAX/4))) // BOTH SENSORS ON BLACK - STOP
          {
              Serial.println("STOP");
              //reset values
              increaseSpeed = 0;
              
              goStop(0);
          }
          
          else if(analogRead(leftSensor) >= (MAX-(AVERAGE/1.5))) // LEFT SENSOR ON BLACK - ROTATE LEFT
          { 
              goStop(100);
              Serial.println("LEFT");
              //reset values
              increaseSpeed = 0;                                                       
              while(analogRead(leftSensor)<(MIN*3.5) && analogRead(rightSensor)<(MIN*3))
              {
                rotateLeft();
              }
              while(analogRead(leftSensor)>=(MIN*3.5) && analogRead(rightSensor)<(MIN*3))
              {
                left();
              }
         
          }
        
          else if(analogRead(rightSensor) >= (MAX-(AVERAGE/1.5))) // RIGHT SENSOR ON BLACK - ROTATE RIGHT
          {  
              goStop(100);
              Serial.println("RIGHT");
              //reset values
              increaseSpeed = 0;
              while(analogRead(rightSensor)<(MIN*3.5) && analogRead(leftSensor)<(MIN*3))
              {
                rotateRight();
              }
              while(analogRead(rightSensor)>=(MIN*3.5) && analogRead(leftSensor)<(MIN*3))
              {
                right();
              }
         
          }
        
          else if(analogRead(leftSensor) >= (MIN*4)) // LEFT SENSOR APPROACHING BLACK - TURN LEFT
          {
            Serial.println("APPROACHING LEFT");
              if(increaseSpeed<=2000){increaseSpeed+=7;} //gradually increase speed of one wheel
              turnLeft(sqrt(increaseSpeed));
          }
        
          else if(analogRead(rightSensor) >= (MIN*4)) // RIGHT SENSOR APPROACHING BLACK - TURN RIGHT
          {
            Serial.println("APPROACHING RIGHT");
              if(increaseSpeed<=2000){increaseSpeed+=7;} //gradually increase speed of one wheel
              turnRight(sqrt(increaseSpeed));
          }
          
          else if(analogRead(leftSensor) < (MIN*4) && analogRead(rightSensor) < (MIN*4)) // BOTH SENSORS ON WHITE - GO
          {
              Serial.println("GO");
              //reset values
              
              increaseSpeed = 0;
              goForward();    
          }
    }
//           
//          WiFiClient client = server.available(); 
//          if (client) {                             // if you get a client,
//              String currentLine = "";                // make a String to hold incoming data from the client
//              if (client.connected()) {            // loop while the client's connected
//                if (client.available()) {             // if there's bytes to read from the client,
//                  char c = client.read();             // read a byte, then
//  //                Serial.write(c);                    // print it out the serial monitor
//                  if (c == 'G') {  // if you got anything else but a carriage return character,
//                    break;      // add it to the end of the currentLine
//                  }
//                  
//                }
//              }
//              client.stop();
//            }
//        }
//    }
    
    void executeMAYBE() {
      
//        if(!executed)
//        {
//          executed = true;
//          scanMap();
//        }
        //OK STOP
        boolean fix = false;
        boolean black = false;
        boolean white = false;
        int flagFix;
        if(analogRead(rightSensor)>(FULLBLACK-300) && analogRead(leftSensor)>(FULLBLACK-300)) {
            check = false;
            goStop(0);
        } 
        
        if(analogRead(leftSensor) >= (BLACKK)) { 
            if(!check) {
              check = true;
              goStop(150); 
            }
            while(analogRead(leftSensor)<(approachingBLACK) && analogRead(rightSensor)<(approachingBLACK))
            {
              rotateLeft();
            } 
            rotateLeft();     
        }
      
        if(analogRead(rightSensor) >= (BLACKK)) {  
            if(!check) {
              check = true;
              goStop(150); 
            }
            while(analogRead(rightSensor)<(approachingBLACK) && analogRead(leftSensor)<(approachingBLACK))
            {
              rotateRight();
            } 
            rotateRight();   
        }
      
        if(analogRead(leftSensor) >= (approachingBLACK)) { 
            Sleft();
        }
      
        if(analogRead(rightSensor) >= (approachingBLACK)) { 
            Sright();
        }
      
        if(analogRead(leftSensor) < (approachingBLACK) && analogRead(rightSensor) < (approachingBLACK)) {
            check = false;
            goForward();
        }
      
        
       
      
        displ();
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

    void racegoForward(int x, int l, int r)
    {
      analogWrite(RB, 0);
      analogWrite(RF, x+r);
      analogWrite(LB, 0);
      analogWrite(LF, x+l); 
    }

    void goBackward(int d) {
      analogWrite(RB, 180);
      analogWrite(RF, 0);
      analogWrite(LB, 180);
      analogWrite(LF, 0);
      if(d!=0) {
        delay(d);
      }
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
    analogWrite(RF, 230);
    analogWrite(LB, 0);
    analogWrite(LF, 180);

    }


    void sright()
    {
    analogWrite(RB, 0);
    analogWrite(RF, 180);
    analogWrite(LB, 0);
    analogWrite(LF, 230);
    }


    void ssleft()
    {
    analogWrite(RB, 0);
    analogWrite(RF, 210);
    analogWrite(LB, 0);
    analogWrite(LF, 180);

    }


    void ssright()
    {
    analogWrite(RB, 0);
    analogWrite(RF, 180);
    analogWrite(LB, 0);
    analogWrite(LF, 210);
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
    analogWrite(RF, x);
    analogWrite(LB, 0);
    analogWrite(LF, 180);
    
    }

    void goRight(int x)
    {
    analogWrite(RB, 0);
    analogWrite(RF, 180);
    analogWrite(LB, 0);
    analogWrite(LF, x);
    
    }

    void goStop()
    {
    analogWrite(RB, 0);
    analogWrite(RF, 0);
    analogWrite(LB, 0);
    analogWrite(LF, 0);
    
    }
    int leeft = 0;
    int riight = 0;
    int sqrtLeft = 0;
    int sqrtRight = 0;
    void execute()
    {

      VL53L0X_RangingMeasurementData_t measure;
      lox.rangingTest(&measure, false); 
      if (measure.RangeStatus != 4) {  
          distanceF = measure.RangeMilliMeter/10;
      } 

      digitalWrite(rightT, LOW);
      delayMicroseconds(2);
      digitalWrite(rightT, HIGH);
      delayMicroseconds(10);
      digitalWrite(rightT, LOW);
      durationR = pulseIn(rightE, HIGH);
      distanceR = (durationR * SOUND_SPEED/2);

      digitalWrite(leftT, LOW);
      delayMicroseconds(2);
      digitalWrite(leftT, HIGH);
      delayMicroseconds(10);
      digitalWrite(leftT, LOW);
      durationL = pulseIn(leftE, HIGH);  
      distanceL = (durationL * SOUND_SPEED/2);
      
      Serial.println((String) "F: " + distanceF);
      Serial.println((String) "L: " + distanceL);
      Serial.println((String) "R: " + distanceR);
      
      if(distanceL < 5 && distanceF < 5 && distanceR < 5 ) {
          
          x = 0;
          goStop();

      } else if(distanceF > distanceR && distanceF > distanceL && distanceF > 5) {

          Serial.println("FFF");
          x = 0;
          if(distanceF > 50) {
            this->racegoForward(255, sqrtLeft, sqrtRight);
          } else if(distanceF > 40) {
            this->racegoForward(200, sqrtLeft, sqrtRight);
          } else if(distanceF > 20) {
            this->racegoForward(180, sqrtLeft, sqrtRight);
          } else if(distanceF <= 20) {
            this->racegoForward(150, sqrtLeft, sqrtRight);
          }
          
          if(distanceR > distanceL+5) {
            sqrtRight += sqrt(riight++);
          }
          if(distanceL > distanceR+5) {
            sqrtLeft += sqrt(leeft++);
          }

          if(distanceL <= 8 && distanceR >= 12) {
              this->ssright();
          }
          if(distanceR <= 8 && distanceL >= 12) {
              this->ssleft(); 
          }

      } else if(distanceR > distanceL && distanceR > 5) {
          sqrtRight = 0;
          sqrtLeft = 0;
          Serial.println("RRR");
          if(distanceF > 50) {

              increase = sqrt(x);
              this->goRight(190+increase);
              if(!x>3000){x+=5;}

          } else if(distanceF > 40) {
              x = 0;
              Serial.println("R1");
              this->sright(); 
          } else if(distanceF > 20) {
              x = 0;
              Serial.println("R2");
              this->right();
          } else if(distanceF <= 20) {
              x = 0;
              Serial.println("R3");
              this->rotateRight();
          }

      } else if(distanceL > distanceR && distanceL > 5) {
          sqrtRight = 0;
          sqrtLeft = 0; 
           Serial.println("LLL");
          if(distanceF > 50) {

              increase = sqrt(x);
              this->goLeft(190+increase);
              if(!x>3000){x+=5;}

          } else if(distanceF > 40) {
              x = 0;
              Serial.println("L1");
              this->sleft();
          } else if(distanceF > 20) {
              x = 0;
              Serial.println("L1");
              this->left();
          } else if(distanceF <= 20) {
              x = 0;
              Serial.println("L1");
              this->rotateLeft();
          }
       } else {
          goBackward(250); x = 0;
          sqrtRight = 0;
          sqrtLeft = 0;
       }
        
    }
};







class Maze
{

  public:  

    long durationS;
    float distanceS;
    int distanceF;
    long durationL;
    float distanceL;
    

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
        analogWrite(RF, 180);
        analogWrite(LF, 180);
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
        analogWrite(RB, 180);
        analogWrite(LF, 180);
        analogWrite(RF, 0);
        analogWrite(LB, 0);      
    }

    void turnLeft(){
        analogWrite(RB, 0);
        analogWrite(LF, 0);
        analogWrite(RF, 180);
        analogWrite(LB, 180);      
    }




    void execute()
    {   
        VL53L0X_RangingMeasurementData_t measure;
        lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
        if (measure.RangeStatus != 4) {  // phase failures have incorrect data 
            distanceF = (measure.RangeMilliMeter); 
        }
        
        digitalWrite(TrigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(TrigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(TrigPin, LOW);
        durationS = pulseIn(EchoPin, HIGH); 
        distanceS = durationS/58;
    
//        digitalWrite(leftT, LOW);
//        delayMicroseconds(2);
//        digitalWrite(leftT, HIGH);
//        delayMicroseconds(10);
//        digitalWrite(leftT, LOW);
//        durationL = pulseIn(leftE, HIGH);  
//        distanceL = durationL/58;
    
        
        if (distanceF > 200){
          stopRobot();
          delay(100);
          Forward();
        }
        else if(distanceS>25 && distanceS<125){
          stopRobot();
          delay(250);
          turnRight();
          delay(350);
          }
        else{
          stopRobot();
          delay(250);
          turnLeft();
          delay(350);
          }
    }

};

Linetracking lt;
Maze maze;
Race race;

String TSSID = "ESP-32_test_E";
unsigned char* ssidToCatch = (unsigned char*) TSSID.c_str();
//(unsigned char*) currentLine.substring(5, ssidLength - 14).c_str();
long rssidToCatch = -1000;

class CTF {

  public:

    long treshhold;
    long rssidToCatch;
    boolean found = false;
    long prevSignal = rssidToCatch;
    int foundRSSI;

    void goForward() {
      analogWrite(RB, 0);
      analogWrite(RF, 180);
      analogWrite(LB, 0);
      analogWrite(LF, 180);
    }

    void goBackward() {
      analogWrite(RB, 180);
      analogWrite(RF, 0);
      analogWrite(LB, 180);
      analogWrite(LF, 0);
    }

    void goStop(int d) {
      analogWrite(RB, 0);
      analogWrite(RF, 0);
      analogWrite(LB, 0);
      analogWrite(LF, 0);
      if(d!=0) {
        delay(d);
      }
    }
    
    void goBackwardLeft() {
      analogWrite(RB, 255);
      analogWrite(RF, 0);
      analogWrite(LB, 180);
      analogWrite(LF, 0);
    }

    void goBackwardRight() {
      analogWrite(RB, 180);
      analogWrite(RF, 0);
      analogWrite(LB, 255);
      analogWrite(LF, 0);
    }
    
    void findSignal() {
      Serial.println("scan start");
      // WiFi.scanNetworks will return the number of networks found
      int n = WiFi.scanNetworks();
      Serial.println("scan done");
      if (n == 0) {
          Serial.println("no networks found");
      } else {
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; i < n; ++i) {

          if(WiFi.SSID(i) == (const char*) ssidToCatch){
            found = true;
            rssidToCatch = WiFi.RSSI(i);
            foundRSSI = i;
          }
        }
      }
    }

    int foundA;
    int foundC;
    int foundD;
    int foundE;
    int foundF;
    void scan() {
      Serial.println("scan start");
      // WiFi.scanNetworks will return the number of networks found
      int n = WiFi.scanNetworks();
      Serial.println("scan done");
      if (n == 0) {
          Serial.println("no networks found");
      } else {
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; i < n; ++i) {
          if(WiFi.SSID(i) == "Robot_A"){
            found = true;
            rssidToCatch = WiFi.RSSI(i);
            foundA = i;
          }
          if(WiFi.SSID(i) == "Robot_C"){
            found = true;
            rssidToCatch = WiFi.RSSI(i);
            foundC = i;
          }
          if(WiFi.SSID(i) == "Robot_D"){
            found = true;
            rssidToCatch = WiFi.RSSI(i);
            foundD = i;
          }
          if(WiFi.SSID(i) == "Robot_E"){
            found = true;
            rssidToCatch = WiFi.RSSI(i);
            foundE = i;
          }
          if(WiFi.SSID(i) == "Robot_F"){
            found = true;
            rssidToCatch = WiFi.RSSI(i);
            foundF = i;
          }
        }
      }
    }

    std::vector<int> signals;
    boolean hasHighestPrev = false;
    long highestPrev;
    boolean foundAgain = false;
    
    void findFlag() {

      prevSignal = rssidToCatch;
      findSignal();
      rssidToCatch = WiFi.RSSI(foundRSSI);
      
      if(found) {
        
        if(rssidToCatch < prevSignal) {
          if(!hasHighestPrev) {
            hasHighestPrev = true;
            highestPrev = prevSignal;
          }
          int starttime = millis();
          int endtime = starttime;
          boolean foundAgain = false;
          int randMove = random(0,2);
          findSignal();
          while( (endtime - starttime) <= 2500 && WiFi.RSSI(foundRSSI) < highestPrev-3) {
            if(randMove == 0) {
              goBackwardLeft();
            } else { 
              goBackwardRight();
            }
            endtime = millis();
            findSignal();
            if(WiFi.RSSI(foundRSSI) >= highestPrev-3){foundAgain = true;}
            else {rssidToCatch = -1000;}
            findSignal();
          }
          if(foundAgain) {
            goStop(100);
            findSignal();
            if(WiFi.RSSI(foundRSSI) >= highestPrev-3) {
              prevSignal = highestPrev-4;
              findSignal();
              rssidToCatch = WiFi.RSSI(foundRSSI);
              while(rssidToCatch > prevSignal) {
                prevSignal = rssidToCatch;
                findSignal();
                rssidToCatch = WiFi.RSSI(foundRSSI);
                goBackward();
              }
            }
            
          }
          
          
        }
        
        hasHighestPrev = false;
        foundAgain = false;
        goForward();
        
      } else {
        findSignal();
        race.execute();
      }
      
    }

    void runAway() {

      scan();
      if(WiFi.RSSI(foundA) < treshhold-20 || WiFi.RSSI(foundC) < treshhold-20 || WiFi.RSSI(foundD) < treshhold-20 || WiFi.RSSI(foundE) < treshhold-20 || WiFi.RSSI(foundF) < treshhold-20)
      {
        goStop(0);
      }
      else 
      {
        race.execute();
      }
    }
  
  
};

CTF ctf;

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

boolean hasFlag = false;

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
              lt.executeMAYBE();
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
          if(currentLine.endsWith("GET /ScanNetwork"))
          {
              lt.execute3();
          }
    //    if(currentLine.endsWith("GET /SendHTTPRequest"))
    //    {
    //        sendHTTPRequest("stopAll");
    //    }
    //
    //    // Game specific - Capture the flag
          if(currentLine.endsWith("GET /giveFlag"))
          {
              hasFlag = true;
              ctf.runAway();
          }
          if(currentLine.endsWith("GET /findFlag"))
          {
              hasFlag = false;
              ctf.findFlag();
          }
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




boolean nextStep = false;
String currentLine = "";

void loop()
{   

  
  WiFiClient client = server.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.print("Reading: ");
        Serial.write(c);                    // print it out the serial monitor
        if(c == 'G'){    
          currentLine = "";
          nextStep = true;
          Serial.write("We got G");
        } else if (c == 'E' && nextStep) {  // if you got anything else but a carriage return character,
          nextStep = false;
          currentLine += 'G';
          Serial.write("We got E");
          while (client.connected()) {
            if(c != '\n') {
              if(client.available()) {
                currentLine += c;
                Serial.write("Next: ");
                c = client.read();
                Serial.write(c);
              }
            } else {break;}
          }
          for(int i = 0; i<10; i++)
          {
            int lastIndex = currentLine.length() - 1;
            currentLine.remove(lastIndex);
            Serial.println(currentLine);
          }
        }   
      }
    }
  }
  
  Serial.println(currentLine);
  if(testIfCommand(currentLine)) {
      client.stop();
      Serial.println();
      Serial.println("Client Disconnected.");
      executeGETRequest(currentLine);
  }
  
}

void displ()
{
    
    int disp = 8;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Left: ");
    display.print(analogRead(leftSensor));
    display.setCursor(0, disp);
    display.print("Right: ");
    display.print(analogRead(rightSensor));
    display.setCursor(0, disp*2);
    display.print("MIN: ");
    display.print(MIN);
    display.setCursor(0, disp*3);
    display.print("MAX: ");
    display.print(MAX);
    display.display();
}
