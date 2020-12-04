#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#define EPD_CS SS
GxEPD2_BW<GxEPD2_213_B73, GxEPD2_213_B73::HEIGHT> display(GxEPD2_213_B73(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDEH0213B73
#include <StreamString.h>
#define PrintString StreamString
#define INCOMMING_SERVER "http://192.168.2.165/api/app/com.internet/garage"
const int port = 80;
const char* ssid = "kosmos";
const char* password = "funhouse";
bool WIFIconnected = false;
bool ClientConnected = false;
String sJSONreceiveCommand;
String sJSONsendCommand;
WiFiClient client;
WiFiServer server(port);

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("setup");
  delay(100);
  display.init(115200);
  display.setPartialWindow(0, 0, display.width(), display.height());
    int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds("Start up", 5, 20, &tbx, &tby, &tbw, &tbh);
  display.setPartialWindow(tbx - 2, tby + 2, display.width(), tbh + 2);
  PrintText("Start up1", &FreeMonoBold9pt7b, 5, 20);
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  WIFIconnected = false;
  display.getTextBounds("Wifi Started", 5, 20, &tbx, &tby, &tbw, &tbh);
  display.setPartialWindow(tbx - 2, tby + 2, display.width(), tbh + 2);
  PrintText("Wifi Started", &FreeMonoBold9pt7b, 5, 20);
}

void loop()
{
  String sParsedJSON;
  // Check if module is still connected to WiFi.
  if (WiFi.status() != WL_CONNECTED)
  {
    if (WIFIconnected == true)
    {
      int16_t tbx, tby; uint16_t tbw, tbh;
      display.getTextBounds("Wifi disconnected.", 5, 20, &tbx, &tby, &tbw, &tbh);
      display.setPartialWindow(tbx - 2, tby + 2, display.width(), tbh + 2);
      PrintText("Wifi disconnected.", &FreeMonoBold9pt7b, 5, 20);
      //Serial.println("Wifi disconnected");
      server.close();
    }
    WIFIconnected = false;
  }
  else
  {
    if (WIFIconnected == false)
    {
      int16_t tbx, tby; uint16_t tbw, tbh;
      display.getTextBounds("Wifi Connected.", 5, 20, &tbx, &tby, &tbw, &tbh);
      display.setPartialWindow(tbx - 2, tby + 2, display.width(), tbh + 2);
      PrintText("Wifi "+WiFi.localIP().toString(), &FreeMonoBold9pt7b, 5, 20);
      server.begin();
      WIFIconnected = true;
    }
    WiFiClient client = server.available();

    if (client)
    {
      int16_t tbx, tby; uint16_t tbw, tbh;
      display.getTextBounds("Client Comunication.", 5, 20, &tbx, &tby, &tbw, &tbh);
      display.setPartialWindow(tbx - 2, tby + 2, display.width(), tbh + 2);
      PrintText("Client Comunication.", &FreeMonoBold9pt7b, 5, 20);
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
          }
          String sParsedGATE  = json_parsed["gate"];
          if (sParsedGATE == "open")
          {
            sJSONreceiveCommand = "Rcv: " + sParsedGATE;
          }
          String sParsedLIGHT = json_parsed["light"];
          if (sParsedLIGHT == "on")
          {
            sJSONreceiveCommand = "Rcv: " + sParsedLIGHT;
          }
          if (sParsedLIGHT == "off")
          {
            sJSONreceiveCommand = "Rcv: " + sParsedLIGHT;
          }
          int16_t tbx, tby; uint16_t tbw, tbh;
          display.getTextBounds(sJSONreceiveCommand, 1, 15, &tbx, &tby, &tbw, &tbh);
          display.setPartialWindow(tbx - 2, tby + 2, display.width(), tbh + 2);
          PrintText(sJSONreceiveCommand, &FreeMonoBold9pt7b, 5, 20);
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
}

void PrintText(String sText, const GFXfont *f, int iX, int iY)
{
  display.setFont(f);
  display.setTextColor(GxEPD_BLACK);
  display.setRotation(1);
  // align with centered HelloWorld
  // height might be different

  /**************************************************************************/
  /*!
      @brief    Helper to determine size of a string with current font/size. Pass
     string and a cursor position, returns UL corner and W,H.
      @param    str     The ascii string to measure
      @param    x       The current cursor X
      @param    y       The current cursor Y
      @param    x1      The boundary X coordinate, set by function
      @param    y1      The boundary Y coordinate, set by function
      @param    w      The boundary width, set by function
      @param    h      The boundary height, set by function
  */
  /**************************************************************************/
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(sText, iX, iY, &tbx, &tby, &tbw, &tbh);
  display.setPartialWindow(tbx - 2, tby + 2, tbw + 2, tbh + 2);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(iX, iY);
    display.print(sText);
  }
  while (display.nextPage());
  Serial.println("helloFullScreenPartialMode done");
}

void showBox(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  //Serial.println("showBox");
  display.setPartialWindow(x, y, w, h);
  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  //display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.fillRect(x, y, w, h, GxEPD_BLACK);
  }
  while (display.nextPage());
  //Serial.println("showBox done");
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

JsonObject& prepareResponse(JsonBuffer & jsonBuffer)
{
  sJSONsendCommand = "Snd Sensor status";
  JsonObject& root = jsonBuffer.createObject();
  //  if (GarageDoorActivated)
  //  {
  //    root["door"] = "moving";
  //  }
  //  else
  //  {
  //    if (GarageDoorOpen == false)
  //    {
  //      root["door"] = "closed";
  //    }
  //    else
  //    {
  //      root["door"] = "open";
  //    }
  //  }
  //  if (bLightOn == true)
  //  {
  //    root["light"] = "on";
  //  }
  //  else
  //  {
  //    root["light"] = "off";
  //  }
  //  //Serial.println(sJSONsendCommand);
  return root;
}

void writeResponse(WiFiClient & client, JsonObject & json) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();
  json.prettyPrintTo(client);
}
