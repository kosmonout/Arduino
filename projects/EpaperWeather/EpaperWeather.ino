//Board ESP32 Dev Module
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#define EPD_CS SS
GxEPD2_BW<GxEPD2_213_B73, GxEPD2_213_B73::HEIGHT> display(GxEPD2_213_B73(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDEH0213B73
const char* ssid = "kosmos";
const char* password = "funhouse";
const char broker[] = "192.168.2.50";
int        port     = 1883;
const char topic[]  = "Temperature";
bool WIFIconnected = false;
bool MQTTconnected = false;
String Temperature;
String Humidity;
String Pressure;
String LightLux;
String WindDirection;
String WindAngle;
String WindSpeed;
String Rain;
String Alarm;
String Rad_CPM;
String Rad_usv;
String PM2_5;
String PM10;

WiFiClient client;
MqttClient mqttClient(client);

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
  MQTTconnected = false;
  display.getTextBounds("Wifi Started", 5, 20, &tbx, &tby, &tbw, &tbh);
  display.setPartialWindow(tbx - 2, tby + 2, display.width(), tbh + 2);
  PrintText("Wifi Started", &FreeMonoBold9pt7b, 5, 20);
}

void loop()
{
  // Check if module is still connected to WiFi.
  if (WiFi.status() != WL_CONNECTED)
  {
    if (WIFIconnected == true)
    {
      int16_t tbx, tby; uint16_t tbw, tbh;
      display.getTextBounds("Wifi disconnected.", 5, 20, &tbx, &tby, &tbw, &tbh);
      display.setPartialWindow(tbx - 2, tby + 2, display.width(), tbh + 2);
      PrintText("Wifi disconnected.", &FreeMonoBold9pt7b, 5, 20);
      delay(2000);
      //Serial.println("Wifi disconnected");
    }
    WIFIconnected = false;
    MQTTconnected = false;
  }
  else
  {
    if (WIFIconnected == false)
    {
      int16_t tbx, tby; uint16_t tbw, tbh;
      display.getTextBounds("Wifi Connected.", 5, 20, &tbx, &tby, &tbw, &tbh);
      display.setPartialWindow(tbx - 2, tby + 2, display.width(), tbh + 2);
      PrintText("Wifi " + WiFi.localIP().toString(), &FreeMonoBold9pt7b, 5, 20);
      WIFIconnected = true;
    }
  }
  
  if (WIFIconnected)
  {
    if (MQTTconnected == false)
    {
      Serial.print("Attempting to connect to the MQTT broker: ");
      Serial.println(broker);
      if (!mqttClient.connect(broker, port))
      {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());
        MQTTconnected = false;
        delay(2000);
      }
      else
      {
        MQTTconnected = true;
      }
      if (MQTTconnected == true)
      {
        Serial.println("You're connected to the MQTT broker!");
        Serial.print("Subscribing to topic: ");
        Serial.println(topic);
        // subscribe to a topic
        mqttClient.subscribe(topic);
      }
    }

    if (!mqttClient.connected())
    {
      Serial.println("mqtt client not connected");
      MQTTconnected = false;
    }
    else
    {
      int messageSize = mqttClient.parseMessage();
      if (messageSize) 
      {
        // we received a message, print out the topic and contents
        Serial.print("Received a message with topic '");
        Serial.print(mqttClient.messageTopic());
        Serial.print("', length ");
        Serial.print(messageSize);
        Serial.println(" bytes:");

        // use the Stream interface to print the contents
        while (mqttClient.available()) {
          Serial.print((char)mqttClient.read());
        }
        Serial.println();
      }
    }
  }
}

void PrintText(String sText, const GFXfont * f, int iX, int iY)
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
