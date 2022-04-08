#include <Adafruit_SSD1306.h>
#include "Adafruit_VL53L0X.h"
#include <analogWrite.h>

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//MOTOR A
int rightBack=16;
int rightAhead=17;
int leftAhead=5;
int leftBack=18;
int lColor = 34;
int rColor = 39;
const int TrigPin = 26;
const int EchoPin = 25;

void setup() {
  // put your setup code here, to run once:
  pinMode(rightAhead, OUTPUT);
  pinMode(rightBack, OUTPUT);
  pinMode(leftAhead, OUTPUT);
  pinMode(leftBack, OUTPUT);
  
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  
  Serial.begin(115200);

  //starting the display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

   // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  if (!lox.begin()) {
    while(1);
  } 
}

long durationR;
long durationL;
float distanceR;
float distanceL;
long durationS;
float distanceS;
int distanceF;
int x;
int increase;


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




void loop()
{   
    
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data 
        distanceF = measure.RangeMilliMeter; 
    }

    digitalWrite(TrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(TrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin, LOW);
    durationS = pulseIn(EchoPin, HIGH); 
    distanceS = (durationS * SOUND_SPEED/2);

    digitalWrite(leftT, LOW);
    delayMicroseconds(2);
    digitalWrite(leftT, HIGH);
    delayMicroseconds(10);
    digitalWrite(leftT, LOW);
    durationL = pulseIn(leftE, HIGH);  
    distanceL = (durationL * SOUND_SPEED/2);

    
    if (distanceF > distanceS && distanceF > 20) {
        stopRobot();
        delay(100);
        Forward();
    } else if(distanceS > distanceL && distanceS > 20) {
        stopRobot();
        delay(100);
        turnRight();
        delay(350);
    } else {
        stopRobot();
        delay(100);
        turnLeft();
        delay(350);
    }
}
