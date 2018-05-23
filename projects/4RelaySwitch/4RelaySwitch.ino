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
const int Relay1Pin = 15;
//d3
const int Relay2Pin = 0;
//d2
const int Relay3Pin = 4;
//d1
const int Relay4Pin = 5;
//d8
const int LedOut  = 13;


const int ContactServerInterval_ms = 300000;
String sJSONreceiveCommand;
String sJSONsendCommand;
bool firstrun = true;
bool WIFIconnected = false;
bool ClientConnected = false;

WiFiClient client;
WiFiServer server(port);

void printWiFiStatus();
TickerScheduler tsTimer(1);
bool readRequest(WiFiClient& client);
int GetRequest();
JsonObject& prepareResponse(JsonBuffer& jsonBuffer);
void writeResponse(WiFiClient& client, JsonObject& json);

////////////////////////////////////////
void setup(void) {

  // Configure GPIO2 as OUTPUT.
  pinMode(LedOut, OUTPUT);
  digitalWrite(LedOut, LOW);
  pinMode(Relay4Pin, OUTPUT);
  pinMode (Relay3Pin, OUTPUT);
  pinMode(Relay1Pin, OUTPUT);
  pinMode (Relay2Pin, OUTPUT);
  digitalWrite(Relay4Pin, HIGH);
  digitalWrite(Relay3Pin, HIGH);
  digitalWrite(Relay1Pin, HIGH);
  digitalWrite(Relay2Pin, HIGH);

  //Start network
  Serial.begin(115200);
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  WIFIconnected = false;
  tsTimer.add(0, ContactServerInterval_ms, GetRequest, false);
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
    digitalWrite(Relay4Pin, LOW);
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
    digitalWrite(LedOut, LOW);
    if (WIFIconnected == false)
    {
      server.begin();
      WIFIconnected = true;
            printWiFiStatus();
    }
    WiFiClient client = server.available();

    if (client)
    {
      digitalWrite(LedOut, LOW);
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
          String sParsedRelay1 = json_parsed["relay1"];
          if (sParsedRelay1 == "on")
          {
            digitalWrite(Relay1Pin, LOW);
            sJSONreceiveCommand = "Rcv: " + Relay1Pin;
          } else if (sParsedRelay1 == "off")
          {
            digitalWrite(Relay1Pin, HIGH);
            sJSONreceiveCommand = "Rcv: " + Relay1Pin;
          }
          String sParsedRelay2 = json_parsed["relay2"];
          if (sParsedRelay2 == "on")
          {
            digitalWrite(Relay2Pin, LOW);
            sJSONreceiveCommand = "Rcv: " + sParsedRelay2;
          } else if (sParsedRelay2 == "off")
          {
            digitalWrite(Relay2Pin, HIGH);
            sJSONreceiveCommand = "Rcv: " + sParsedRelay2;
          }
          String sParsedRelay3 = json_parsed["relay3"];
          if (sParsedRelay3 == "on")
          {
            digitalWrite(Relay3Pin, LOW);
            sJSONreceiveCommand = "Rcv: " + sParsedRelay3;
          } else if (sParsedRelay3 == "off")
          {
            digitalWrite(Relay3Pin, HIGH);
            sJSONreceiveCommand = "Rcv: " + sParsedRelay3;
          }
          String sParsedRelay4 = json_parsed["relay4"];
          if (sParsedRelay4 == "on")
          {
            digitalWrite(Relay4Pin, LOW);
            sJSONreceiveCommand = "Rcv: " + sParsedRelay4;
          } else if (sParsedRelay4 == "Off")
          {
            digitalWrite(Relay4Pin, HIGH);
            sJSONreceiveCommand = "Rcv: " + sParsedRelay4;
          }
          Serial.println(sJSONreceiveCommand);
        }
        StaticJsonBuffer<200> jsonWriteBuffer;
        JsonObject& jsonWrite = prepareResponse(jsonWriteBuffer);
        writeResponse(client, jsonWrite);
        ClientConnected = true;
        delay(1);
        digitalWrite(Relay4Pin, HIGH);
        client.stop();
      }
    }
  }
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
    if (Relay1Pin == HIGH)
    {
      root["relay1"] = "on";
    }
    else
    {
      root["relay1"] = "off";
    }
    if (Relay2Pin == HIGH)
    {
      root["relay2"] = "on";
    }
    else
    {
      root["relay2"] = "off";
    }
    if (Relay3Pin == HIGH)
    {
      root["relay3"] = "on";
    }
    else
    {
      root["relay3"] = "off";
    }
    if (Relay4Pin == HIGH)
    {
      root["relay4"] = "on";
    }
    else
    {
      root["relay4"] = "off";
    }
    Serial.println(sJSONsendCommand);
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

