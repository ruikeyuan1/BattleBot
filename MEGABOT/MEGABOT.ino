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

void loop() {
  
  WiFiClient client = server.available();   // listen for incoming clients

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


             if (header.indexOf("GET /Lt") >= 0) {
              Serial.println("LINETRACKING");
              linetracking();
            } else if (header.indexOf("GET /Maze") >= 0) {
              Serial.println("MAZE");
              maze();
            } else if (header.indexOf("GET /Race") >= 0) {
              Serial.println("Race");
              race();
            } else if (header.indexOf("GET /Ctf") >= 0) {
              Serial.println("CTF");
              ctf();
            } else if (header.indexOf("GET /Go") >= 0) {
              Serial.println("GO");
              goForward(0);
            } else if (header.indexOf("GET /Back") >= 0) {
              Serial.println("Back");
              goBackward(0);
            } else if (header.indexOf("GET /RL") >= 0) {
              Serial.println("Rotating Left");
              rL(0);
            } else if (header.indexOf("GET /RR") >= 0) {
              Serial.println("Rotating Right");
              rR(0);
            } else if (header.indexOf("GET /Stop") >= 0) {
              Serial.println("STOP");
              goStop(0);
            } else if (header.indexOf("GET /Exit") >= 0) {
              Serial.println("exit");
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

void linetracking()
{
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
      fixx = 0;
      goStop(0);
  }
  
  else if(analogRead(leftSensor) >= (MAX-(AVERAGE/1.5))) // LEFT SENSOR ON BLACK - ROTATE LEFT
  {
      rotateLeft(50);
      goStop(100);
      Serial.println("LEFT");
      //reset values
      increaseSpeed = 0;                                                       
      while(analogRead(leftSensor)<(MIN*3.5) && analogRead(rightSensor)<(MIN*3))
      {
        rotateLeft(0);
      }
      while(analogRead(leftSensor)>=(MIN*3.5) && analogRead(rightSensor)<(MIN*3))
      {
        left(0);
      }
 
  }

  else if(analogRead(rightSensor) >= (MAX-(AVERAGE/1.5))) // RIGHT SENSOR ON BLACK - ROTATE RIGHT
  {  
      rotateRight(50);
      goStop(100);
      Serial.println("RIGHT");
      //reset values
      increaseSpeed = 0;
      while(analogRead(rightSensor)<(MIN*3.5) && analogRead(leftSensor)<(MIN*3))
      {
        rotateRight(0);
      }
      while(analogRead(rightSensor)>=(MIN*3.5) && analogRead(leftSensor)<(MIN*3))
      {
        right(0);
      }
 
  }

  else if(analogRead(leftSensor) >= (MIN*4)) // LEFT SENSOR APPROACHING BLACK - TURN LEFT
  {
    Serial.println("APPROACHING LEFT");
      if(increaseSpeed<=2000){increaseSpeed+=7;} //gradually increase speed of one wheel
      turnLeft(0, sqrt(increaseSpeed));
  }

  else if(analogRead(rightSensor) >= (MIN*4)) // RIGHT SENSOR APPROACHING BLACK - TURN RIGHT
  {
    Serial.println("APPROACHING RIGHT");
      if(increaseSpeed<=2000){increaseSpeed+=7;} //gradually increase speed of one wheel
      turnRight(0, sqrt(increaseSpeed));
  }
  
  else if(analogRead(leftSensor) < (MIN*4) && analogRead(rightSensor) < (MIN*4)) // BOTH SENSORS ON WHITE - GO
  {
      Serial.println("GO");
      //reset values
      fixx = 0;
      increaseSpeed = 0;
      goForward(0);    
  } 
  
 

  displ();
  
}

void scanMap()
{
    goStop(2000);
    displ();
    RS = analogRead(rightSensor);
    LS = analogRead(leftSensor);
    std::vector<int> MIN_array;
    while(RS<200)
    {
      if(RS>MAX){MAX = RS;}
      if(LS>MAX){MAX = LS;}
      if(RS<100){MIN_array.push_back(RS);}
      if(LS<100){MIN_array.push_back(RS);}
      rotateLeft(0);
      LS = analogRead(leftSensor);
      RS = analogRead(rightSensor);
      displ();
    }
    goStop(100);
    LS = analogRead(leftSensor);
    RS = analogRead(rightSensor);
    displ();
    while(RS<500)
    {
      rotateRight(0);
      LS = analogRead(leftSensor);
      RS = analogRead(rightSensor);
      displ();
    }
    MIN = (std::accumulate(MIN_array.begin(), MIN_array.end(), 0))/MIN_array.size();
    AVERAGE = (MAX+MIN)/2;
    displ();
    goStop(2000);
  
}

void maze()
{


}

void race()
{


}

void ctf()
{


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
