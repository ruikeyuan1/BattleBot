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
void setup() {
  // put your setup code here, to run once:
  pinMode(rightAhead, OUTPUT);
  Serial.begin(115200);
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
  //int x = 8;
  VL53L0X_RangingMeasurementData_t measure;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  //display.print("R-color: ");
  //display.print(rColor);
  //display.setCursor(0, x);
  //display.print("L-color: ");
  //display.print(lColor);
  display.setCursor(0, 0);
  display.print("Dist-sensor: ");
  display.print(Distance());
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

void Forward(){
  analogWrite(rightAhead, 200);
  analogWrite(leftAhead, 220);
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
   analogWrite(rightBack, 200);
   analogWrite(leftAhead, 170);
   analogWrite(rightAhead, 0);
   analogWrite(leftBack, 0);      
  }

void loop() {
  // put your main code here, to run repeatedly:
  sensorInfo();

  if(Distance()>= 200){
  Forward();    
    }
    else{
  //stop
   /*analogWrite(rightAhead, 0);
   analogWrite(leftAhead, 0);*/
   stopRobot();
   //rotate right
   delay(3000);
   turnRight();
      }
}
