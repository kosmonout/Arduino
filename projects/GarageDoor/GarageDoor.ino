#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <TickerScheduler.h>
#include <ArduinoJson.h>

#define INCOMMING_SERVER "http://192.168.2.165/api/app/com.internet/garage"
const int port = 80;
const char* ssid = "kosmos";
const char* password = "funhouse";
//RELAYS
//d7
const int Rl3GarageButPin = 13;
//d4
const int Rl4PowerGaragePin = 2;
//d6
const int Rl2LightPin = 12;
//d5
const int RlGateOpenPin = 14;
//Inputs
//sd2
const int DoorSensorPin = 10;
//sd3
const int GarageButtonPin = 0;
//RGB led
//d1
const int grnPin  = 5;
//d2
const int  bluPin  = 4;
//d0
const int  redPin = 16;

const int ContactServerInterval_ms = 300000;
const int GarageButHold_ms = 500;
const int GarageDoorActive_ms = 21000;
const int GateDoorOpen_ms = 4000;
const int PowerGarageDoor_ms = 120000;
const int PowerBeforeButton_ms = 500;
const int FadeWaitus = 100;      // wait in us to fade

// Color arrays [R,G,B]
int black[3]  = { 0, 0, 0 };
int white[3]  = { 100, 100, 100 };
int red[3]    = { 100, 0, 0 };
int purple[3]    = { 0, 100, 100 };
int green[3]  = { 0, 100, 0 };
int blue[3]   = { 0, 0, 100 };
int yellow[3] = { 100, 10, 0 };
int dimWhite[3] = { 30, 30, 30 };

// Set initial color
int redVal = black[0];
int grnVal = black[1];
int bluVal = black[2];



// Initialize color variables
int prevR = redVal;
int prevG = grnVal;
int prevB = bluVal;

bool bLightOn;
int GarabeButton;
String sJSONreceiveCommand;
String sJSONsendCommand;
int DoorSensor;
bool firstrun = true;
bool WIFIconnected = false;
bool ClientConnected = false;
bool GarageDoorActivated = false;
bool GarageDoorOpen = false;

WiFiClient client;
WiFiServer server(port);

void crossFade(int color[3]);
void printWiFiStatus();
void printWiFiStatus();
void GarageHit();
void GateHit();
void SendUpdate();
void GarageDoorActive();
void GateActive();
void LightOff();
void LightOn();
void GarageDoorUnactive();
void Power220Off();
void GarageButtonHit();
TickerScheduler tsTimer(6);
bool readRequest(WiFiClient& client);
JsonObject& prepareResponse(JsonBuffer& jsonBuffer);
void writeResponse(WiFiClient& client, JsonObject& json);

////////////////////////////////////////
void setup(void) {
  crossFade(dimWhite);

  // Configure GPIO2 as OUTPUT.
  pinMode(grnPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(bluPin, OUTPUT);
  pinMode(Rl3GarageButPin, OUTPUT);
  pinMode (Rl2LightPin, OUTPUT);
  pinMode(Rl4PowerGaragePin, OUTPUT);
  pinMode (RlGateOpenPin, OUTPUT);
  pinMode (DoorSensorPin, INPUT_PULLUP);
  pinMode (GarageButtonPin, INPUT_PULLUP);
  digitalWrite(Rl3GarageButPin, HIGH);
  digitalWrite(Rl2LightPin, HIGH);
  digitalWrite(Rl4PowerGaragePin, HIGH);
  digitalWrite(RlGateOpenPin, HIGH);
  //  delay (500);
  //  digitalWrite(Rl3GarageButPin, LOW);
  //  digitalWrite(Rl2LightPin, HIGH);
  //  digitalWrite(Rl4PowerGaragePin, HIGH);
  //  digitalWrite(RlGateOpenPin, HIGH);
  //  delay (500);
  //  digitalWrite(Rl3GarageButPin, HIGH);
  //  digitalWrite(Rl2LightPin, LOW);
  //  digitalWrite(Rl4PowerGaragePin, HIGH);
  //  digitalWrite(RlGateOpenPin, HIGH);
  //  delay (500);
  //  digitalWrite(Rl3GarageButPin, HIGH);
  //  digitalWrite(Rl2LightPin, HIGH);
  //  digitalWrite(Rl4PowerGaragePin, LOW);
  //  digitalWrite(RlGateOpenPin, HIGH);
  //  delay (500);
  //  digitalWrite(Rl3GarageButPin, HIGH);
  //  digitalWrite(Rl2LightPin, HIGH);
  //  digitalWrite(Rl4PowerGaragePin, HIGH);
  //  digitalWrite(RlGateOpenPin, LOW);
  //  delay (500);
  //  digitalWrite(Rl3GarageButPin, HIGH);
  //  digitalWrite(Rl2LightPin, HIGH);
  //  digitalWrite(Rl4PowerGaragePin, HIGH);
  //  digitalWrite(RlGateOpenPin, HIGH);


  bLightOn = false;

  //Start network
  crossFade(yellow);
  Serial.begin(115200);
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  WIFIconnected = false;
  tsTimer.add(2, ContactServerInterval_ms, GetRequest, false);
  // Start TCP server.
}

////////////////////////////////////////////////////////////////////////////////
void loop(void)
{
  tsTimer.update();
  String sParsedJSON;
  // Check if module is still connected to WiFi.
  if (WiFi.status() != WL_CONNECTED)
  {
    crossFade(yellow);
    if (WIFIconnected == true)
    {
      //Serial.println("Wifi disconnected");
      server.close();
    }
    WIFIconnected = false;
  }
  else
  {
    // Print the new IP to Serial.
    crossFade(blue);
    if (WIFIconnected == false)
    {
      server.begin();
      WIFIconnected = true;
    }
    WiFiClient client = server.available();

    if (client)
    {
      crossFade(green);
      //Serial.println("Client.");
      bool success = readRequest(client);
      if (success)
      {
        String line = client.readStringUntil('\r');
        //Read JSON
        //Serial.println("line: " + line);
        if (line.length() > 0)
        {
          // Parse JSON
          int size = line.length() + 1;
          char jsonChar[size];
          line.toCharArray(jsonChar, size);
          StaticJsonBuffer<200> jsonReadBuffer;
          JsonObject& json_parsed = jsonReadBuffer.parseObject(jsonChar);
          if (!json_parsed.success())
          {
            Serial.println("parseObject() failed");
            return;
          }
          sJSONreceiveCommand = "Unrecognized command";
          //json_parsed.prettyPrintTo(Serial);
          String sParsedDOOR = json_parsed["door"];
          if (sParsedDOOR == "move")
          {
            sJSONreceiveCommand = "Rcv: " + sParsedDOOR;
            GarageDoorActive();
          }
          String sParsedGATE  = json_parsed["gate"];
          if (sParsedGATE == "open")
          {
            sJSONreceiveCommand = "Rcv: " + sParsedGATE;
            GateActive();
          }
          String sParsedLIGHT = json_parsed["light"];
          if (sParsedLIGHT == "on")
          {
            sJSONreceiveCommand = "Rcv: " + sParsedLIGHT;
            LightOn();
          }
          if (sParsedLIGHT == "off")
          {
            sJSONreceiveCommand = "Rcv: " + sParsedLIGHT;
            LightOff();
          }
          //Serial.println(sJSONreceiveCommand);
        }
        StaticJsonBuffer<200> jsonWriteBuffer;
        JsonObject& jsonWrite = prepareResponse(jsonWriteBuffer);
        writeResponse(client, jsonWrite);
        ClientConnected = true;
        delay(1);
        client.stop();
      }
    }
  }

  GarabeButton = digitalRead(GarageButtonPin);

  if ( GarabeButton == LOW )
  {
    //Serial.println("GB ");
    GarageDoorActive();
  }

  DoorSensor = digitalRead(DoorSensorPin);
  if (GarageDoorActivated == false) {
    if ( DoorSensor == LOW && GarageDoorOpen == true)
    {
      tsTimer.remove(2);
      //Serial.println("Close");
      GarageDoorOpen = false;
      GetRequest();
      tsTimer.add(2, ContactServerInterval_ms, GetRequest, false);

      //Garage door is dicht
    }
    else if (DoorSensor == HIGH &&  GarageDoorOpen == false)
    {
      //Garage door is open
      tsTimer.remove(2);
      GarageDoorOpen = true;
      GetRequest();
      tsTimer.add(2, ContactServerInterval_ms, GetRequest, false);
      //Serial.println("Open");
    }
  }
}


////////////////////////////////////////

void GarageDoorActive()
{
  digitalWrite(Rl4PowerGaragePin, LOW);
  tsTimer.remove(5);
  tsTimer.remove(2);
  tsTimer.add(5, PowerBeforeButton_ms, GarageButtonHit, false);
  GarageDoorActivated = true;
  GarageDoorOpen = !GarageDoorOpen;
}

void GarageButtonHit()
{
  tsTimer.remove(5);
  digitalWrite(Rl3GarageButPin, LOW);
  tsTimer.remove(0);
  tsTimer.add(0, GarageButHold_ms, GarageHit, false);
  tsTimer.remove(3);
  tsTimer.add(3, GarageDoorActive_ms, GarageDoorUnactive, false);
  tsTimer.remove(4);;
}

void GateActive()
{
  tsTimer.remove(1);
  tsTimer.remove(2);
  digitalWrite(RlGateOpenPin, LOW);
  tsTimer.add(1, GateDoorOpen_ms, GateHit, false);
}

void LightOff() {
  digitalWrite(Rl2LightPin, HIGH);
  if (bLightOn == true)
  {
    tsTimer.remove(2);
    bLightOn = false;
    GetRequest();
    tsTimer.add(2, ContactServerInterval_ms, GetRequest, false);
  }
}
void LightOn() {
  digitalWrite(Rl2LightPin, LOW);
  if (bLightOn == false)
  {
    tsTimer.remove(2);
    bLightOn = true;
    GetRequest();
    tsTimer.add(2, ContactServerInterval_ms, GetRequest, false);
  }
}

void GarageHit() {
  tsTimer.remove(0);
  digitalWrite(Rl3GarageButPin, HIGH);
  GetRequest();
}

void GarageDoorUnactive() {
  tsTimer.remove(3);
  GarageDoorActivated = false;
  if ( DoorSensor == LOW)
  {
    //Garage door is dicht
    GarageDoorOpen = false;
  } else
  {
    //Garage door is open
    GarageDoorOpen = true;
  }

  digitalWrite(Rl4PowerGaragePin, LOW);
  tsTimer.remove(4);
  tsTimer.add(4, PowerGarageDoor_ms, Power220Off, false);
  GetRequest();
  tsTimer.add(2, ContactServerInterval_ms, GetRequest, false);
}

void Power220Off() {
  tsTimer.remove(4);
  digitalWrite(Rl4PowerGaragePin, HIGH);
}

void GateHit() {
  tsTimer.remove(1);
  digitalWrite(RlGateOpenPin, HIGH);
  tsTimer.add(2, ContactServerInterval_ms, GetRequest, false);
}

int GetRequest()
{
  sJSONsendCommand = "Snd: GET";
  HTTPClient http;
  http.begin(INCOMMING_SERVER);
  int httpCode = http.GET();
  http.end();
  //Serial.println(sJSONsendCommand);
  return httpCode;
}

JsonObject& prepareResponse(JsonBuffer & jsonBuffer)
{
  sJSONsendCommand = "Snd Sensor status";
  JsonObject& root = jsonBuffer.createObject();
  if (GarageDoorActivated)
  {
    root["door"] = "moving";
  }
  else
  {
    if (GarageDoorOpen == false)
    {
      root["door"] = "closed";
    }
    else
    {
      root["door"] = "open";
    }
  }
  if (bLightOn == true)
  {
    root["light"] = "on";
  }
  else
  {
    root["light"] = "off";
  }
  //Serial.println(sJSONsendCommand);
  return root;
}

void writeResponse(WiFiClient & client, JsonObject & json) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();
  json.prettyPrintTo(client);
}

bool readRequest(WiFiClient & client) {
  bool currentLineIsBlank = true;
  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      if (c == '\n' && currentLineIsBlank) {
        //Serial.println("Client Request Confirmed.");
        return true;
      } else if (c == '\n') {
        currentLineIsBlank = true;
      } else if (c != '\r') {
        currentLineIsBlank = false;
      }
    }
  }
  return false;
}

/* BELOW THIS LINE IS THE MATH -- YOU SHOULDN'T NEED TO CHANGE THIS FOR THE BASICS

  The program works like this:
  Imagine a crossfade that moves the red LED from 0-10,
    the green from 0-5, and the blue from 10 to 7, in
    ten steps.
    We'd want to count the 10 steps and increase or
    decrease color values in evenly stepped increments.
    Imagine a + indicates raising a value by 1, and a -
    equals lowering it. Our 10 step fade would look like:

    1 2 3 4 5 6 7 8 9 10
  R + + + + + + + + + +
  G   +   +   +   +   +
  B     -     -     -

  The red rises from 0 to 10 in ten steps, the green from
  0-5 in 5 steps, and the blue falls from 10 to 7 in three steps.

  In the real program, the color percentages are converted to
  0-255 values, and there are 1020 steps (255*4).

  To figure out how big a step there should be between one up- or
  down-tick of one of the LED values, we call calculateStep(),
  which calculates the absolute gap between the start and end values,
  and then divides that gap by 1020 to determine the size of the step
  between adjustments in the value.
*/

int calculateStep(int prevValue, int endValue) {
  int step = endValue - prevValue; // What's the overall gap?
  if (step) {                      // If its non-zero,
    step = 1020 / step;            //   divide by 1020
  }
  return step;
}

/* The next function is calculateVal. When the loop value, i,
   reaches the step size appropriate for one of the
   colors, it increases or decreases the value of that color by 1.
   (R, G, and B are each calculated separately.)
*/

int calculateVal(int step, int val, int i) {

  if ((step) && i % step == 0) { // If step is non-zero and its time to change a value,
    if (step > 0) {              //   increment the value if step is positive...
      val += 1;
    }
    else if (step < 0) {         //   ...or decrement it if step is negative
      val -= 1;
    }
  }
  // Defensive driving: make sure val stays in the range 0-255
  if (val > 255) {
    val = 255;
  }
  else if (val < 0) {
    val = 0;
  }
  return val;
}

/* crossFade() converts the percentage colors to a
   0-255 range, then loops 1020 times, checking to see if
   the value needs to be updated each time, then writing
   the color values to the correct pins.
*/

void crossFade(int color[3]) {
  // Convert to 0-255
  int R = (color[0] * 255) / 100;
  int G = (color[1] * 255) / 100;
  int B = (color[2] * 255) / 100;

  if ((prevR != R) || (prevG != G) || (prevB != B)) {
    int stepR = calculateStep(prevR, R);
    int stepG = calculateStep(prevG, G);
    int stepB = calculateStep(prevB, B);
    for (int i = 0; i <= 1020; i++) {
      redVal = calculateVal(stepR, redVal, i);
      grnVal = calculateVal(stepG, grnVal, i);
      bluVal = calculateVal(stepB, bluVal, i);

      analogWrite(redPin, redVal);   // Write current values to LED pins
      analogWrite(grnPin, grnVal);
      analogWrite(bluPin, bluVal);

      delayMicroseconds(FadeWaitus); // Pause for 'wait' microseconds before resuming the loop
    }
    // Update current values for next loop
    prevR = redVal;
    prevG = grnVal;
    prevB = bluVal;
  }
}
void printWiFiStatus() {
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

