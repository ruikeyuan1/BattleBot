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

void sensorInfo(){
  VL53L0X_RangingMeasurementData_t measure;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  //display.setCursor(0, 0);
  //display.print("R-color: ");
  //display.print(rColor);
  //display.setCursor(0, x);
  //display.print("L-color: ");
  //display.print(lColor);
  display.setCursor(0, 0);
  display.print("Distance forward: ");
  display.print(Distance())/10;
  display.setCursor(0, 16);
  display.print("Dist-sensor-right: ");
  display.setCursor(0, 24);
  display.print(ultrasonic());
  display.print(" cm");
  display.display();
}

int Distance(){
  VL53L0X_RangingMeasurementData_t measure;
  
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data 
    return (measure.RangeMilliMeter);} 
    else {
    return -1;}
  }

int ultrasonic(){       
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

void rotateRight(){
  int time = millis(); //time=2000;  
  int timeToRotate = 440+time; // 2700
  while(time <= timeToRotate){ // 2000 <= 2700
    turnRight();
    time = millis(); // 2001 2002 2003 ... 2700
    }
}

void rotateLeft(){
  int time = millis(); //time=2000;  
  int timeToRotate = 700+time; // 2700
  while(time <= timeToRotate){ // 2000 <= 2700
    turnLeft();
    time = millis(); // 2001 2002 2003 ... 2700
    }
}

void Forward(){
  analogWrite(rightAhead, 190);
  analogWrite(leftAhead, 190);
  analogWrite(rightBack, 0);
  analogWrite(leftBack, 0);
  }

void stopRobot(){
  analogWrite(rightBack, 0);
   analogWrite(leftAhead, 0);
   analogWrite(rightAhead, 0);
   analogWrite(leftBack, 0);
  }

void turnRight(){
   analogWrite(rightBack, 190);
   analogWrite(leftAhead, 190);
   analogWrite(rightAhead, 0);
   analogWrite(leftBack, 0);      
  }

void turnLeft(){
   analogWrite(rightBack, 0);
   analogWrite(leftAhead, 0);
   analogWrite(rightAhead, 190);
   analogWrite(leftBack, 190);      
  }
  
void loop() {
  // put your main code here, to run repeatedly:
  sensorInfo();

  /*stopRobot();
  delay(320);
  turnRight();
  delay(300);*/
  
  if (Distance() > 200){
      stopRobot();
      delay(100);
      Forward();
  }
  else if(ultrasonic()>25 && ultrasonic()<125){
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
