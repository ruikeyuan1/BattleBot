//test
#include <Adafruit_VL53L0X.h>
#include <Adafruit_SSD1306.h>
#include <analogWrite.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

///////////////////////////////////////////////

int RB = 16;
int RF = 17;
int LF = 5;
int LB = 18;
int LC = 34;
int RC = 39;

////////////////////////////////////////////////

void setup() {
  pinMode(RF, OUTPUT);
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  while (! Serial) {
    delay(1);
  }
  if (!lox.begin()) {
    while(1);
  } 
}

int dist(){
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); //'true' to get debug data
  if (measure.RangeStatus != 3) {   //3 is a minimum distance error 
    return (measure.RangeMilliMeter);} 
    else {
      return -1;
    }
}

void Forward(){
  analogWrite(RF, 180);
  analogWrite(LF, 200);
  analogWrite(RB, 0);
  analogWrite(LB, 0);
}

void Stop(){ 
  analogWrite(RB, 0);
   analogWrite(LF, 0);
   analogWrite(RF, 0);
   analogWrite(LB, 0);
}

void Right(){
   analogWrite(RB, 180);
   analogWrite(LF, 150);
   analogWrite(RF, 0);
   analogWrite(LB, 0);      
}

void Left(){
   analogWrite(LB, 180);
   analogWrite(RF, 150);
   analogWrite(LF, 0);
   analogWrite(RB, 0);      
}

void Back(){
    analogWrite(RF, 0);
    analogWrite(LF, 0);
    analogWrite(RB, 180);
    analogWrite(LB, 200);
}

void sensorInfo(){
  VL53L0X_RangingMeasurementData_t measure;
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("Distance: ");
  display.print(dist());
  display.display();
}

void loop() {  
  if(dist()<= 300){
     Forward();    
  }
  else{
   Stop();
   delay(1000);
   if(dist()<= 300){
       Back();
       delay(500);
       Right();
   }
   else{
       Back();
       delay(500);
       Left();
   }
  }
}