//-------------------------------WIFI (--------------------------
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
//-------------------------------WIFI )--------------------------
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <vector>
#include <numeric>



//-------------------------------WIFI (--------------------------

// Set these to your desired credentials.
const char *ssid = "yourAP";
const char *password = "yourPassword";

WiFiServer server(80);
String header;
//-------------------------------WIFI )--------------------------

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

//Color Sensors Pin
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
void goStop(int d);
void goForward(int d);
void goBackward(int d);
void turnRight(int d, int x);
void turnLeft(int d, int x);
void rotateLeft(int d);
void rotateRight(int d);
void left(int d);
void right(int d);
void linetracking();
void scanMap();
void maze();
void race();
void ctf();
void displ();

void setup() {
//-------------------------------WIFI (--------------------------
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  Serial.println();
  Serial.println("Configuring access point...");

  // You can remove the password parameter if you want the AP to be open.
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();

  Serial.println("Server started");
//-------------------------------WIFI )--------------------------

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  //Initiate INPUT for Color Sensors
  pinMode (leftSensor, INPUT);
  pinMode (rightSensor, INPUT);

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
  delay(200);
}


void goStop(int d)
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 0);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 0);
  if(d!=0)
  {
    delay(d);
  }
}

void goForward(int d)
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 165);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 165);
  if(d!=0)
  {
    delay(d);
  }
}

void goBackward(int d)
{
  ledcWrite(RBC, 170);
  ledcWrite(RFC, 0);
  ledcWrite(LBC, 170);
  ledcWrite(LFC, 0);
  if(d!=0)
  {
    delay(d);
  }
}

void turnRight(int d, int x)
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, (175+x));
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 170);
  if(d!=0)
  {
    delay(d);
  }
}

void turnLeft(int d, int x)
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 170);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, (175+x));
  if(d!=0)
  {
    delay(d);
  }
}

void rotateLeft(int d)
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 170);
  ledcWrite(LBC, 180);
  ledcWrite(LFC, 0);
  if(d!=0)
  {
    delay(d);
  }
}

void rL(int d)
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 170);
  ledcWrite(LBC, 170);
  ledcWrite(LFC, 0);
  if(d!=0)
  {
    delay(d);
  }
}

void rotateRight(int d)
{
  ledcWrite(RBC, 180);
  ledcWrite(RFC, 0);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 170);
  if(d!=0)
  {
    delay(d);
  }
}

void rR(int d)
{
  ledcWrite(RBC, 170);
  ledcWrite(RFC, 0);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 170);
  if(d!=0)
  {
    delay(d);
  }
}

void right(int d)
{
  ledcWrite(RBC, 180);
  ledcWrite(RFC, 0);
  ledcWrite(LBC, 0);
  ledcWrite(LFC, 170);
  if(d!=0)
  {
    delay(d);
  }
}

void left(int d)
{
  ledcWrite(RBC, 0);
  ledcWrite(RFC, 170);
  ledcWrite(LBC, 180);
  ledcWrite(LFC, 0);
  if(d!=0)
  {
    delay(d);
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
