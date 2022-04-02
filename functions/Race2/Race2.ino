
   
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

void setup() {
  // put your setup code here, to run once:
  pinMode(rightAhead, OUTPUT);
  pinMode(rightBack, OUTPUT);
  pinMode(leftAhead, OUTPUT);
  pinMode(leftBack, OUTPUT);

  
  Serial.begin(115200);

   // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

}


  
void loop() {
  Serial.println("test");
  analogWrite(rightAhead, 990);
  analogWrite(leftAhead, 990);
  analogWrite(rightBack, 0);
  analogWrite(leftBack, 0);
}
