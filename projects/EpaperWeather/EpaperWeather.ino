//Board ESP32 Dev Module
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <GxEPD2_BW.h>
//#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMono12pt7b.h>
#include <Fonts/FreeMono18pt7b.h>
#include <Fonts/FreeMono24pt7b.h>
#define DISPLAY_INTERVAL_MS 600000

#define EPD_CS SS
GxEPD2_BW<GxEPD2_213_B73, GxEPD2_213_B73::HEIGHT> display(GxEPD2_213_B73(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDEH0213B73
const char* ssid = "kosmos";
const char* password = "funhouse";
const char broker[] = "192.168.2.50";
int        port     = 1883;
const char topic[]  = "Temperature";
char *strTopic[] = {"Temperature", "Humidity", "Pressure", "LightLux", "WindDirection", "WindAngle",
                    "WindSpeed", "Rain", "Alarm", "Rad_CPM", "Rad_usv", "PM2_5", "PM10"
                   };
int iTopic = 0;
bool WIFIconnected = false;
bool MQTTconnected = false;
bool bUpdateDisplay = true;
bool bUpdateMQTTDisplay = true;
String Temperature = "INI";
String Humidity = "INI";
String Pressure = "INI";
String LightLux = "INI";
String WindDirection = "INI";
String WindAngle = "INI";
String WindSpeed = "INI";
String Rain = "INI";
String Alarm = "INI";
String Rad_CPM = "INI";
String Rad_usv = "INI";
String PM2_5 = "INI";
String PM10 = "INI";

hw_timer_t * timerDispl = NULL;
portMUX_TYPE timerMuxDispl = portMUX_INITIALIZER_UNLOCKED;

WiFiClient client;
MqttClient mqttClient(client);
void TopicChange(String sTopicValue);
void UpdateDisplayTimer();

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("setup");
  vTaskDelay( 100 / portTICK_PERIOD_MS);
  display.init(115200);
  display.setTextColor(GxEPD_BLACK);
  display.setRotation(1);
  display.setPartialWindow(0, 0, display.width(), display.height());
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds("Start up", 5, 20, &tbx, &tby, &tbw, &tbh);
  display.setPartialWindow(tbx - 2, tby + 2, display.width(), tbh + 2);
  PrintText("Start up.", &FreeMonoBold9pt7b, 5, 20);
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  WIFIconnected = false;
  MQTTconnected = false;
  display.getTextBounds("Wifi Started..", 5, 20, &tbx, &tby, &tbw, &tbh);
  display.setPartialWindow(tbx - 2, tby + 2, display.width(), tbh + 2);
  PrintText("Wifi Started", &FreeMonoBold9pt7b, 5, 20);
  vTaskDelay( 2000 / portTICK_PERIOD_MS);
  /* 1 tick take 1/(80MHZ/80000) = 1ms so we set divider 80000 and count up */
  timerDispl = timerBegin(0, 80, true);
  timerAttachInterrupt(timerDispl, &UpdateDisplayTimer, true);
  timerAlarmWrite(timerDispl, DISPLAY_INTERVAL_MS * 1000, true);
  //showBoxBlack(10,10,100,100);
}

void loop()
{
  // Check if module is still connected to WiFi.
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.disconnect();
    WiFi.reconnect();
    vTaskDelay( 15000 / portTICK_PERIOD_MS);
    if (WIFIconnected == true)
    {
      ClearDisplay();
      int16_t tbx, tby; uint16_t tbw, tbh;
      display.getTextBounds("Wifi disconnected.", 5, 20, &tbx, &tby, &tbw, &tbh);
      display.setPartialWindow(tbx - 2, tby + 2, display.width(), tbh + 2);
      PrintText("Wifi disconnected...", &FreeMonoBold9pt7b, 5, 20);
      //Serial.println("Wifi disconnected");
      WIFIconnected = false;
      MQTTconnected = false;
    }
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
      vTaskDelay( 2000 / portTICK_PERIOD_MS);
      PrintText("                                ", &FreeMonoBold9pt7b, 5, 20);
    }
  }

  if (WIFIconnected)
  {
    if (MQTTconnected == false)
    {
      Serial.print("Attempting to connect to the MQTT broker: ");
      PrintText("Connecting to MQTT...", &FreeMonoBold9pt7b, 5, 20);
      Serial.println(broker);
      if (!mqttClient.connect(broker, port))
      {
        if (bUpdateMQTTDisplay == true)
        {
          ClearDisplay();
          PrintText("MQTT connection failed!", &FreeMonoBold9pt7b, 5, 20);
          PrintText(" Error code = ", &FreeMonoBold9pt7b, 5, 40);
          PrintText(String(mqttClient.connectError()), &FreeMonoBold9pt7b, 100, 40);
          bUpdateMQTTDisplay == false;
        }
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());
        MQTTconnected = false;
        timerAlarmDisable(timerDispl);
        vTaskDelay( 2000 / portTICK_PERIOD_MS);
      }
      else
      {
        Serial.println("You're connected to the MQTT broker!");
        PrintText("MQTT client connected!", &FreeMonoBold9pt7b, 5, 20);
        // subscribe to a topic
        mqttClient.subscribe(String(strTopic[iTopic]));
        bUpdateDisplay = true;
        MQTTconnected = true;
        bUpdateMQTTDisplay == true;
        timerAlarmEnable(timerDispl);
      }
    }

    if (!mqttClient.connected())
    {
      MQTTconnected = false;
    }
    else
    {
      int messageSize = mqttClient.parseMessage();
      if (messageSize)
      {
        String sValue = "";
        // we received a message, print out the topic and contents
        //Serial.print("Received a message with topic '");
        //Serial.print(mqttClient.messageTopic());
        //Serial.print("', length ");
        //Serial.print(messageSize);
        //Serial.println(" bytes:");
        // use the Stream interface to print the contents
        while (mqttClient.available())
        {
          sValue = sValue + (char)mqttClient.read();
        }
        TopicChange(sValue);
        if ( bUpdateDisplay == true)
        {
          ClearDisplay();
          display.setTextColor(GxEPD_BLACK);
          display.setRotation(1);
          //TEMPERATURE
          PrintText("Temp   [C]", &FreeMonoBold9pt7b, 3, 18);
          PrintText(Temperature, &FreeMonoBold18pt7b, 3, 48);
          //Humidity
          PrintText("Humid. [%]", &FreeMonoBold9pt7b, 130, 18);
          PrintText(Humidity, &FreeMonoBold18pt7b, 130, 48);
          //Wind dir
          PrintText("Wind", &FreeMonoBold9pt7b, 3, 72);
          PrintText("Dir", &FreeMonoBold9pt7b, 3, 85);
          PrintText(WindDirection, &FreeMonoBold12pt7b, 62, 79);
          // Wind Speed
          PrintText("Wind", &FreeMonoBold9pt7b, 130, 72);
          PrintText("Spd", &FreeMonoBold9pt7b, 130, 85);
          PrintText(WindSpeed, &FreeMonoBold12pt7b, 188, 79);
          //Rain
          PrintText("Rain", &FreeMonoBold9pt7b, 3, 115);
          PrintText(Rain, &FreeMonoBold12pt7b, 62, 115);

          PrintText("PM25", &FreeMonoBold9pt7b, 130, 107);
          PrintText("ugm3", &FreeMonoBold9pt7b, 130, 122);
          PrintText(PM2_5, &FreeMonoBold12pt7b, 188, 115);
          //PrintText(Rad_CPM, &FreeMonoBold18pt7b, 140, 90);
          // Serial.print("Height: " + String(display.height()));
          //Serial.print("Width: " + String(display.width()));
          showBoxBlack(0, 55, display.width(), 2);
          showBoxBlack(0, 90, display.width(), 2);
          showBoxBlack(display.width() / 2, 0, 2, display.height());
          bUpdateDisplay = false;
        }
        // subscribe to a topic
      }
    }
  }
}

void TopicChange(String sTopicValue)
{
  mqttClient.unsubscribe(String(strTopic[iTopic]));
  switch (iTopic) {
    case 0:
      if (isValidNumber(sTopicValue))
      {
        Serial.println("TopicValue " + sTopicValue);
        Temperature = truncate(sTopicValue, 5, 1);
      }
      else
      {
        Temperature = "ERR  ";
      }
      break;
    case 1:
      if (isValidNumber(sTopicValue))
      {
        Serial.println("TopicValue " + sTopicValue);
        Humidity = truncate(sTopicValue, 5, 1);
      }
      else
      {
        Humidity = "ERR  ";
      }
      break;
    case 2:
      Pressure = sTopicValue;
      break;
    case 3:
      LightLux = sTopicValue;
      break;
    case 4:
      WindDirection = sTopicValue;
      break;
    case 5:
      WindAngle = sTopicValue;
      break;
    case 6:
      if (isValidNumber(sTopicValue))
      {
        Serial.println("TopicValue " + sTopicValue);
        WindSpeed = truncate(sTopicValue, 4, 1) ;
      }
      else
      {
        WindSpeed = "ERR ";
      }
      break;
    case 7:
      if (sTopicValue == "LOW")
      {
        Rain = "LO ";
      }
      else if (sTopicValue == "Medium")
      {
        Rain = "MED";
      }
      else if (sTopicValue == "High")
      {
        Rain = "HI ";
      }
      else if (sTopicValue == "None")
      {
        Rain = "NO ";
      }
      else if (sTopicValue == "ERROR")
      {
        Rain = "ERR";
      }
      break;
    case 8:
      Alarm = sTopicValue;
      break;
    case 9:
      if (isValidNumber(sTopicValue))
      {
        Serial.println("TopicValue " + sTopicValue);
        Rad_CPM = truncate(sTopicValue, 4, 0);
      }
      else
      {
        Rad_CPM = "ERR ";
      }
      break;
    case 10:
      Rad_usv = sTopicValue;
      break;
    case 11:
      if (isValidNumber(sTopicValue))
      {
        Serial.println("TopicValue " + sTopicValue);
        PM2_5 = truncate(sTopicValue, 3, 0);
      }
      else
      {
        PM2_5 = "ERR";
      }
      break;
    case 12:
      if (isValidNumber(sTopicValue))
      {
        Serial.println("TopicValue " + sTopicValue);
        PM10 = truncate(sTopicValue, 3, 0);
      }
      else
      {
        PM10 = "ERR";
      }
      break;
    default:
      // statements
      break;
  }
  Serial.print("Topic Name: ");
  Serial.print(strTopic[iTopic]);
  Serial.print(" Value: ");
  Serial.println(sTopicValue);
  iTopic++;
  if (iTopic > 12)
  {
    iTopic = 0;
  };
  mqttClient.subscribe(String(strTopic[iTopic]));
}

void IRAM_ATTR UpdateDisplayTimer()
{
  portENTER_CRITICAL_ISR(&timerMuxDispl);
  bUpdateDisplay = true;
  portEXIT_CRITICAL_ISR(&timerMuxDispl);
}

void PrintText(String sText, const GFXfont * f, int iX, int iY)
{
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
  display.setFont(f);
  display.getTextBounds(sText, iX, iY, &tbx, &tby, &tbw, &tbh);
  display.setPartialWindow(tbx - 1, tby + 1, tbw + 1, tbh + 1);
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

void ClearDisplay()
{
  display.setPartialWindow(0, 0, display.width(), display.height());
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
  }
  while (display.nextPage());
  Serial.println("helloFullScreenPartialMode done");
}

void showBoxBlack(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  //Serial.println("showBox");
  display.setPartialWindow(x, y, w, h);
  display.firstPage();
  do
  {
    //display.fillScreen(GxEPD_BLACK);
    //display.fillRect(x, y, w, h, GxEPD_WHITE);
    display.fillScreen(GxEPD_WHITE);
    display.fillRect(x, y, w, h, GxEPD_BLACK );
  }
  while (display.nextPage());
  //Serial.println("showBox done");
}

boolean isValidNumber(String str)
{
  boolean isNum = false;
  if (!(str.charAt(0) == '+' || str.charAt(0) == '-' || isDigit(str.charAt(0)))) return false;

  for (byte i = 1; i < str.length(); i++)
  {
    if (!(isDigit(str.charAt(i)) || str.charAt(i) == '.')) return false;
  }
  return true;
}
String truncate(String sTopicValue, int iCharLength, byte dec)
{
  float number = sTopicValue.toFloat();
  float x = number * pow(10, dec);
  float y = round(x);
  float z = x - y;
  if ((int)z == 5)
  {
    y++;
  } else {}
  x = y / pow(10, dec);
  char buffer[iCharLength];
  dtostrf(x, iCharLength, dec, buffer);
  Serial.println("TopicValue " + String(buffer));
  return String(buffer);
}
