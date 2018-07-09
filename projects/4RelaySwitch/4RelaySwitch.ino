#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <TickerScheduler.h>
#include <ArduinoJson.h>

#define INCOMMING_SERVER "http://192.168.2.165/api/app/com.internet/4relay"
const int port = 80;
const char* ssid = "kosmos";
const char* password = "funhouse";
//RELAYS

//d6
const int Relay1Pin = 12;
//d5
const int Relay2Pin = 14;
//d2
const int Relay3Pin = 4;
//d1
const int Relay4Pin = 5;
//d8
const int LedOut  = 16;


const int ContactServerInterval_ms = 300000;
String sJSONreceiveCommand;
String sJSONsendCommand;
bool WIFIconnected = false;

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
  pinMode(LedOut, OUTPUT);
  digitalWrite(LedOut, HIGH);
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
  digitalWrite(LedOut, LOW);
  delay(200);
  digitalWrite(LedOut, HIGH);
  //tsTimer.add(0, ContactServerInterval_ms, GetRequest, false);

  // Start TCP server.
}

////////////////////////////////////////////////////////////////////////////////
void loop(void)
{
  tsTimer.update();
  // Check if module is still connected to WiFi.
  if (WiFi.status() != WL_CONNECTED)
  {
    if (WIFIconnected == true)
    {
      Serial.println("Wifi disconnected");
      digitalWrite(LedOut, LOW);
      server.close();
      tsTimer.remove(0);
    }
    WIFIconnected = false;
  }
  else
  {
    // Print the new IP to Serial.
    if (WIFIconnected == false)
    {
      digitalWrite(LedOut, HIGH);
      server.begin();
      WIFIconnected = true;
      GetRequest();
      printWiFiStatus();
    }
    WiFiClient client = server.available();
    client.setNoDelay(false);
    if (client)
    {
      bool success = readRequest(client);
      if (success)
      {
        String line = client.readStringUntil('\r');
        //Read JSON
        //Serial.println("line: " + line);
        digitalWrite(LedOut, HIGH);
        if (line.length() > 0)
        {
          // Parse JSON
          int size = line.length() + 1;
          char jsonChar[size];
          line.toCharArray(jsonChar, size);
          //DynamicJsonBuffer jsonReadBuffer;
          StaticJsonBuffer<200> jsonReadBuffer;
          JsonObject& json_parsed = jsonReadBuffer.parseObject(jsonChar);
          if (!json_parsed.success())
          {
            // Serial.println("parseObject() failed");
            return;
          }
          sJSONreceiveCommand = "Unrecognized command";
          // json_parsed.prettyPrintTo(Serial);
          if (json_parsed["relay1"] == F("on"))
          {
            digitalWrite(Relay1Pin, LOW);
          } else if (json_parsed["relay1"] == F("off"))
          {
            digitalWrite(Relay1Pin, HIGH);
          }
          if (json_parsed["relay2"] == F("on"))
          {
            digitalWrite(Relay2Pin, LOW);
          } else if (json_parsed["relay2"] == F("off"))
          {
            digitalWrite(Relay2Pin, HIGH);
          }
          if (json_parsed["relay3"] == F("on"))
          {
            digitalWrite(Relay3Pin, LOW);
          } else if (json_parsed["relay3"] == F("off"))
          {
            digitalWrite(Relay3Pin, HIGH);
          }
          if (json_parsed["relay4"] == F("on"))
          {
            digitalWrite(Relay4Pin, LOW);
          } else if (json_parsed["relay4"] == F("off"))
          {
            digitalWrite(Relay4Pin, HIGH);
          }
          GetRequest();
        }
        StaticJsonBuffer<200> jsonWriteBuffer;
        JsonObject& jsonWrite = prepareResponse(jsonWriteBuffer);
        writeResponse(client, jsonWrite);
        delay(1);
        client.stop();
        digitalWrite(LedOut, LOW);
      }
    }
  }
}

int GetRequest()
{
  HTTPClient http;
  tsTimer.remove(0);
  sJSONsendCommand = "Snd: GET";
  http.begin(INCOMMING_SERVER);
  int httpCode = http.GET();
  http.end();
  //Serial.println(sJSONsendCommand);
  tsTimer.add(0, ContactServerInterval_ms, GetRequest, false);
  return httpCode;
}

JsonObject& prepareResponse(JsonBuffer & jsonBuffer)
{
  JsonObject& root = jsonBuffer.createObject();
  if (digitalRead(Relay1Pin) == LOW)
  {
    root["relay1"] = "on";
    //Serial.println("relay1:on");
  }
  else
  {
    root["relay1"] = "off";
    //Serial.println("relay1:off");
  }
  if (digitalRead(Relay2Pin) == LOW)
  {
    root["relay2"] = "on";
    //Serial.println("relay2:on");
  }
  else
  {
    root["relay2"] = "off";
    //Serial.println("relay2:off");
  }
  if (digitalRead(Relay3Pin) == LOW)
  {
    root["relay3"] = "on";
    //Serial.println("relay3:on");
  }
  else
  {
    root["relay3"] = "off";
    //Serial.println("relay3:off");
  }
  if (digitalRead(Relay4Pin) == LOW)
  {
    root["relay4"] = "on";
    //Serial.println("relay4:on");
  }
  else
  {
    root["relay4"] = "off";
    //Serial.println("relay4:off");
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

void printWiFiStatus()
{
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

