#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <TickerScheduler.h>
#include <ArduinoJson.h>

#define INCOMMING_SERVER "http://192.168.2.165/api/app/com.internet/frontdoor"
const int port = 80;
const char* ssid = "kosmos";
const char* password = "funhouse";
const int FrontDoorButtonPin = 0;

const int ContactServerInterval_ms = 300000;

int FrontDoorButton;
String sJSONreceiveCommand;
String sJSONsendCommand;
bool bFrontDoorHit = false;
bool firstrun = true;
bool WIFIconnected = false;
bool ClientConnected = false;

WiFiClient client;
WiFiServer server(port);

void printWiFiStatus();
TickerScheduler tsTimer(6);
bool readRequest(WiFiClient& client);
JsonObject& prepareResponse(JsonBuffer& jsonBuffer);
void writeResponse(WiFiClient& client, JsonObject& json);

////////////////////////////////////////
void setup(void) {

  // Configure GPIO2 as OUTPUT.
  pinMode (FrontDoorButtonPin, INPUT_PULLUP);

  //Start network
  Serial.begin(115200);
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  WIFIconnected = false;
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
    if (WIFIconnected == true)
    {
      Serial.println("Wifi disconnected");
      server.close();
    }
    WIFIconnected = false;
  }
  else
  {
    // Print the new IP to Serial.
    if (WIFIconnected == false)
    {
      server.begin();
      WIFIconnected = true;
    }
    WiFiClient client = server.available();

    if (client)
    {
      //Serial.println("Client.");
      bool success = readRequest(client);
      if (success)
      {
        String line = client.readStringUntil('\r');
        //Read JSON
        Serial.println("line: " + line);
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
          json_parsed.prettyPrintTo(Serial);
          String sParsedDOOR = json_parsed["door"];
          if (sParsedDOOR == "move")
          {
            sJSONreceiveCommand = "Rcv: " + sParsedDOOR;
           }
          Serial.println(sJSONreceiveCommand);
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

  FrontDoorButton = digitalRead(FrontDoorButtonPin);

  if ( FrontDoorButton == LOW )
  {
    bFrontDoorHit = true;
    Serial.println("GB ");
  }
}

////////////////////////////////////////

int GetRequest()
{
  sJSONsendCommand = "Snd: GET";
  HTTPClient http;
  http.begin(INCOMMING_SERVER);
  int httpCode = http.GET();
  http.end();
  Serial.println(sJSONsendCommand);
  return httpCode;
}

JsonObject& prepareResponse(JsonBuffer & jsonBuffer)
{
  sJSONsendCommand = "Snd Sensor status";
  JsonObject& root = jsonBuffer.createObject();
  if (bFrontDoorHit)
  {
    root["frontbutton"] = "hit";
  }
  else
  {
    root["frontbutton"] = "none";
  }
  Serial.println(sJSONsendCommand);
  bFrontDoorHit = false;
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

void printWiFiStatus() {
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
