#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <TickerScheduler.h>
#include <ArduinoJson.h>
#include "SSD1306.h"
#include "BMP280.h"
#include "SHT31.h"
#include "MAX44009.h"
#include "ADS1015.h"
#include <PubSubClient.h>

//#define BLYNK_PRINT Serial
//#define BLYNK_GREEN     "#23C48E"
//#define BLYNK_BLUE      "#04C0F8"
//#define BLYNK_YELLOW    "#ED9D00"
//#define BLYNK_RED       "#D3435C"
//#define BLYNK_DARK_BLUE "#5F7CD8"
//#include <BlynkSimpleEsp8266.h>
//Inputs
//D2
#define  PIN_SDA 4
//D1
#define PIN_SCL 5
//D0
#define PIN_HEATERON 16
//D5
#define PIN_POWERWIND 14


#define WIND_N 0.04
#define WIND_NNE 0.35
#define WIND_NE 0.66
#define WIND_ENE 0.87
#define WIND_E 1.28
#define WIND_ESE 1.58
#define WIND_SE 1.89
#define WIND_SSE 2.20
#define WIND_S 2.51
#define WIND_SSW 2.82
#define WIND_SW 3.13
#define WIND_WSW 3.44
#define WIND_W 3.75
#define WIND_WNW 4.06
#define WIND_NW 4.37
#define WIND_NNW 4.68
#define WIND_ACCURACY 0.12

#define DISPLAY_INTERVAL_MS 2500
#define CLEAR_JSON_INTERVAL_MS 3000
#define CONTACT_SERVER_INTERVAL_MS 20000
#define UPDATE_SENSOR_MS 3000
#define CHECK_BLYNK_MS 1000
#define HTTP_PORT 80
#define HEATER_ON_MOISTURE 75
#define HEATER_OFF_MOISTURE 70
#define TEMPERATURE_CAL -0.35
#define DELTA_AIR_PRESURE_ARROW 0.5
#define RAIN_MAX 1.3
#define RAIN_MEDIUM 1.7
#define RAIN_LOW 2.2
#define RAIN_NO 3.0
#define SSID "kosmos"
#define PASSWORD "funhouse"
//#define BLYNK_AUTH "63fb3008df63415784b2284c087c64bd"
#define INCOMMING_SERVER "http://192.168.2.165/api/app/com.internet/weather"
#define MQTT_SERVER "m23.cloudmqtt.com"
#define MQTT_PORT 17768
#define MQTT_USERNAME   "nzubqmrm"
#define MQTT_KEY        "QQiE_DzdaWEu"

//Start network
WiFiClient client;
PubSubClient MQTTclient(client);
WiFiServer server(HTTP_PORT);
#define WIND_ALARM 20 // 
#define COUT_WIND_ALARM 50

#define ADC_IN_WINDPOWER 0
#define ADC_IN_WINDIRECTION 1
#define ADC_IN_RAIN 2
#define ADDR_SHT31 0x44 // sht31 bus address Moisture/Temp
#define ADDR_SSD1306 0x3c // SSD1306 bus address Display
#define ADDR_MAX44009 0x4a // MAX44009 bus address Light lux
SSD1306  display(ADDR_SSD1306, PIN_SDA, PIN_SCL);
BMP280 bmp(PIN_SDA, PIN_SCL);
SHT31 sht31(PIN_SDA, PIN_SCL);
Max44009 myLux(ADDR_MAX44009);
ADS1115 ads;

char readBytes(unsigned char *values, char length);
char writeBytes(unsigned char *values, char length);
char readInt(char address, int16_t &value);
char readUInt(char address, uint16_t &value);
void DisplayStatus();
void UpdateDisplay();
void UpdateSensors();
void ClearJSON();
void UpdateMQTT();
bool checkMQTT();

//void UpdateBlynk();
//bool checkBlynk();

uint8_t crc8(const uint8_t *data, uint8_t len);

bool readRequest(WiFiClient& client);
JsonObject& prepareResponse(JsonBuffer& jsonBuffer);
void writeResponse(WiFiClient& client, JsonObject& json);

int GetRequest();
int iDisplay = 1;
bool WIFIconnected = false;
bool ClientConnected = false;
double dTemperature;
double dHumidity;
double dPressure;
double dPressurePrev = 0;
int iPressureDirection = 0;
double dLightLux;
double dWindPower;
String sWindDirection;
bool bTemperatureAvail;
bool bHumidityAvail;
bool bPressureAvail;
bool bLightLuxAvail;
bool bWindPowerAvail;
bool bWindDirectionAvail;
bool bRainAvail;
boolean bHeaterRain = false;
int iRainLevel = 0;
int iAlarmCount;
int iMQTTUpdateScreen = 1;
int iWindAngle = 0;
boolean bWAlarm = false;
String sJSONsendCommand;
TickerScheduler tsTimer(4);

////////////////////////////////////////
void setup(void)
{
  display.init();
  display.setContrast(255);
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawStringMaxWidth(0, 0, 128, "Booting.....");
  display.display();
  pinMode(PIN_HEATERON, OUTPUT);
  pinMode(PIN_POWERWIND, OUTPUT);
  digitalWrite(PIN_HEATERON, LOW);
  digitalWrite(PIN_POWERWIND, LOW);
  Serial.begin(115200);
  WiFi.disconnect();
  WiFi.begin(SSID, PASSWORD);
  iRainLevel = 0;
  ads.begin();
  if (ads.readADC_SingleEnded(ADC_IN_WINDPOWER) == 0xFFFF)
  {
    display.drawStringMaxWidth(0, 16, 128, "NOT found AD conv");
  }
  else
  {
    display.drawStringMaxWidth(0, 16, 128, "Found AD conv");
    ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  }
  if (!bmp.begin())
  {
    display.drawStringMaxWidth(0, 28, 128, "NOT found Presure sensor");
  }
  else
  {
    display.drawStringMaxWidth(0, 28, 128, "Found Presure sensor");
  }
  display.display();

  sht31.begin(ADDR_SHT31);
  uint32_t serialNo;
  if ( !sht31.readSerialNo(serialNo))
  {
    {
      display.drawStringMaxWidth(0, 40, 128, "NO I2C SHT31");
    }
  }
  else
  {
    if (!sht31.readTempHum())
    {
      display.drawStringMaxWidth(0, 40, 128, "NO VDD SHT31");
    }
    else
    {
      display.drawStringMaxWidth(0, 40, 128, "Found SHT31");
    }
  }

  display.display();
  dLightLux = myLux.getLux(); //dummy read to generate an error
  if (myLux.getError() != 0)
  {
    display.drawStringMaxWidth(0, 52, 128, "NOT found MAX77009");
  }
  else
  {
    display.drawStringMaxWidth(0, 52, 128, "Found MAX77009");
  }
  display.display();
  //Blynk.config(BLYNK_AUTH);
  // Blynk.connect(10);
  Wire.begin(PIN_SDA, PIN_SCL); // the SDA and SCL
  WIFIconnected = false;
  // Initialising the UI will init the display too.
  display.clear();
  //Display update interval
  tsTimer.add(0, DISPLAY_INTERVAL_MS, UpdateDisplay, false);
  //Servercontact update interval
  tsTimer.add(1, CONTACT_SERVER_INTERVAL_MS, GetRequest, false);
  //UpdateSensors
  tsTimer.add(2, UPDATE_SENSOR_MS, UpdateSensors, false);
  //Check blynk status and reconnect
  tsTimer.add(3, CHECK_BLYNK_MS, checkMQTT, false);
  UpdateSensors();
  MQTTclient.setServer(MQTT_SERVER, MQTT_PORT);
  // Blynk.run();
  //  checkBlynk();
  delay(2000);
}

////////////////////////////////////////////////////////////////////////////////
void loop(void)
{
  // Blynk.run();
  tsTimer.update();
  // Check if module is still connected to WiFi.
  if (WiFi.status() != WL_CONNECTED)
  {
    if (WIFIconnected == true)
    {
      Serial.println("Wifi disconnected");
      server.close();
    }
    WIFIconnected = false;
    DisplayStatus();
  }
  else
  {
    // Print the new IP to Serial.
    if (WIFIconnected == false)
    {
      server.begin();
      WIFIconnected = true;
      DisplayStatus();
    }
    WiFiClient client = server.available();
    if (client)
    {
      bool success = readRequest(client);
      if (success)
      {
        //Send JSON
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

//#####################################################################################################

void UpdateDisplay()
{
  DisplayStatus();
  iDisplay++;
  if (iDisplay > 4) iDisplay = 1;
}

void UpdateSensors()
{
  digitalWrite(PIN_POWERWIND, HIGH);
  dTemperature = sht31.readTemperature() + TEMPERATURE_CAL;
  if (dTemperature < -40 || dTemperature > 125 )
  {
    bTemperatureAvail = false;
  } else
  {
    bTemperatureAvail = true;
  }
  dHumidity = sht31.readHumidity();
  if (dHumidity < 0 || dHumidity > 100)
  {
    digitalWrite(PIN_HEATERON, HIGH);
    bHumidityAvail = false;
  }
  else
  {
    bHumidityAvail = true;
  }
  //If we do not know the moisture level put the heater on
  //Put the heater if the moisture level is too high or the rainsensor start to conduct
  if (dHumidity > HEATER_ON_MOISTURE || iRainLevel > 0 || bHumidityAvail == false)
  {
    digitalWrite(PIN_HEATERON, HIGH);
    bHeaterRain = true;
  }
  else
  {
    digitalWrite(PIN_HEATERON, LOW);
    bHeaterRain = false;
  }

  dPressure = bmp.readPressure() / 100;
  if (dPressure >= 1200 || dPressure <= 900)
  {
    bPressureAvail = false;
  } else
  {
    bPressureAvail = true;
    if (abs(dPressure - dPressurePrev) > 100)
    {
      dPressurePrev = dPressure;
      iPressureDirection = 0;
    } else if ((dPressure - dPressurePrev) > DELTA_AIR_PRESURE_ARROW)
    {
      //Presure up
      dPressurePrev = dPressure;
      iPressureDirection = 1;
    } else if ((dPressure - dPressurePrev) < -DELTA_AIR_PRESURE_ARROW)
    {
      //Presure down
      dPressurePrev = dPressure;
      iPressureDirection = 2;
    }
  }
  dLightLux = myLux.getLux();
  if (myLux.getError() != 0)
  {
    bLightLuxAvail = false;
  } else
  {
    bLightLuxAvail = true;
  }
  double dWindPwrVoltage = ads.readADC_SingleEnded(ADC_IN_WINDPOWER);
  if (dWindPwrVoltage == 0xFFFF)
  {
    bWindPowerAvail = false;
  }
  else
  {
    bWindPowerAvail = true;
    dWindPwrVoltage = dWindPwrVoltage * 0.1875e-3;
    dWindPower = dWindPwrVoltage * 10;
    if (dWindPower < 1)
    {
      dWindPower = 0;
    }
  }

  double dWindDirVoltage = ads.readADC_SingleEnded(ADC_IN_WINDIRECTION);
  if (dWindDirVoltage == 0xFFFF)
  {
    bWindDirectionAvail = false;
  }
  else
  {
    bWindDirectionAvail = true;
    dWindDirVoltage = dWindDirVoltage * 0.1875e-3;
    if (((WIND_N - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_N + WIND_ACCURACY)))
    {
      sWindDirection = "N";
      iWindAngle = 0;
    }
    else if (((WIND_NNE - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_NNE + WIND_ACCURACY)))
    {
      sWindDirection = "NNE";
      iWindAngle = 22;
    }
    else if (((WIND_NE - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_NE + WIND_ACCURACY)))
    {
      sWindDirection = "NE";
      iWindAngle = 45;
    }
    else if (((WIND_ENE - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_ENE + WIND_ACCURACY)))
    {
      sWindDirection = "ENE";
      iWindAngle = 67;
    }
    else if (((WIND_E - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_E + WIND_ACCURACY)))
    {
      sWindDirection = "E";
      iWindAngle = 90;
    }
    else if (((WIND_ESE - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_ESE + WIND_ACCURACY)))
    {
      sWindDirection = "ESE";
      iWindAngle = 112;
    }
    else if (((WIND_SE - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_SE + WIND_ACCURACY)))
    {
      sWindDirection = "SE";
      iWindAngle = 135;
    }
    else if (((WIND_SSE - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_SSE + WIND_ACCURACY)))
    {
      sWindDirection = "SSE";
      iWindAngle = 147;
    }
    else if (((WIND_S - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_S + WIND_ACCURACY)))
    {
      sWindDirection = "S";
      iWindAngle = 180;
    }
    else if (((WIND_SSW - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_SSW + WIND_ACCURACY)))
    {
      sWindDirection = "SSW";
      iWindAngle = 202;
    }
    else if (((WIND_SW - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_SW + WIND_ACCURACY)))
    {
      sWindDirection = "SW";
      iWindAngle = 225;
    }
    else if (((WIND_WSW - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_WSW + WIND_ACCURACY)))
    {
      sWindDirection = "WSW";
      iWindAngle = 247;
    }
    else if (((WIND_W - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_W + WIND_ACCURACY)))
    {
      sWindDirection = "W";
      iWindAngle = 270;
    }
    else if (((WIND_WNW - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_WNW + WIND_ACCURACY)))
    {
      sWindDirection = "WNW";
      iWindAngle = 292;
    }
    else if (((WIND_NW - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_NW + WIND_ACCURACY)))
    {
      sWindDirection = "NW";
      iWindAngle = 315;
    }
    else if (((WIND_NNW - WIND_ACCURACY) < dWindDirVoltage) && (dWindDirVoltage < (WIND_NNW + WIND_ACCURACY)))
    {
      sWindDirection = "NNW";
      iWindAngle = 337;
    } else
    {
      iWindAngle = 999;
      sWindDirection = "ERROR";
    }
  }
  digitalWrite(PIN_POWERWIND, LOW);
  double dRainVoltage = ads.readADC_SingleEnded(ADC_IN_RAIN);
  if (dRainVoltage == 0xFFFF)
  {
    bRainAvail = false;
  }
  else
  {
    bRainAvail = true;
    dRainVoltage = dRainVoltage * 0.1875e-3;
    if (dRainVoltage < RAIN_MAX)
    {
      iRainLevel = 4;
    } else if (dRainVoltage > RAIN_MAX && dRainVoltage < RAIN_MEDIUM )
    {
      iRainLevel = 3;
    }
    else if (dRainVoltage > RAIN_MEDIUM && dRainVoltage < RAIN_LOW )
    {
      iRainLevel = 2;
    }
    else if (dRainVoltage > RAIN_LOW && dRainVoltage < RAIN_NO )
    {
      //To detect if there is moisture to set already the heater on
      iRainLevel = 1;
    }
    else
    {
      iRainLevel = 0;
    }
  }

  if ((dWindPower > WIND_ALARM || iRainLevel > 1 ) && bWAlarm == false )
  {
    bWAlarm = true;
    iAlarmCount = COUT_WIND_ALARM;
  } else if (iAlarmCount == 0 && iRainLevel < 2 )
  {
    bWAlarm = false;
  }

  if (iAlarmCount >= 0)
  {
    iAlarmCount--;
  }
}

void UpdateMQTT()
{
  //Upload to MQTT
  //In steps othewise the device will be loose connection with to many virtualwrites at once.

  switch (iMQTTUpdateScreen)
  {
    case 1:
      if (bTemperatureAvail)
      {
        MQTTclient.publish("Temperature", String(dTemperature).c_str());
      }
      else
      {
        MQTTclient.publish("Temperature", "error");
      }
      break;
    case 2:
      if (bHumidityAvail)
      {
        MQTTclient.publish("Humidity", String(dHumidity).c_str());
      }
      else
      {
        MQTTclient.publish("Humidity", "error");
      }
      break;
    case 3:
      if (bPressureAvail)
      {
        MQTTclient.publish("Pressure", String(dPressure).c_str());
      }
      else
      {
        MQTTclient.publish("Pressure", "error");
      }
      break;
    case 4:
      if (bLightLuxAvail)
      {
        MQTTclient.publish("LightLux", String(dLightLux).c_str());
      }
      else
      {
        MQTTclient.publish("LightLux", "error");
      }
      break;
    case 5:
      if (bWindDirectionAvail)
      {
        MQTTclient.publish("WindDirection", sWindDirection.c_str());
        MQTTclient.publish("WindAngle", String(iWindAngle).c_str());
      }
      else
      {
        MQTTclient.publish("WindDirection", "error");
        MQTTclient.publish("WindAngle", "999");
      }
      break;
    case 6:
      if (bWindPowerAvail)
      {
        MQTTclient.publish("WindPower", String(dWindPower).c_str());
      }
      else
      {
        MQTTclient.publish("WindPower", "error");
      }
      break;
    case 7:
      if (bRainAvail)
      {
        if (iRainLevel == 2)
        {
          MQTTclient.publish("Rain", "LOW");
        } else if (iRainLevel == 3)
        {
          MQTTclient.publish("Rain", "Medium");
        } else if (iRainLevel == 4)
        {
          MQTTclient.publish("Rain", "High");
        }
        else
        {
          MQTTclient.publish("Rain", "None");
        }
      }
      else
      {
        MQTTclient.publish("Rain", "ERROR");
      }
      break;
    case 8:
      if (bWAlarm == true)
      {
        MQTTclient.publish("Alarm", "Triggered");
      }
      else
      {
        MQTTclient.publish("Alarm", "None");
      }
      break;
  }
  iMQTTUpdateScreen++;
  if (iMQTTUpdateScreen > 8) iMQTTUpdateScreen = 1;
}


//void UpdateBlynk()
//{
//  //Upload to Blynk
//  //In steps othewise the device will be loose connection with to many virtualwrites at once.
//
//  switch (iMQTTUpdateScreen)
//  {
//    case 1:
//      if (bTemperatureAvail)
//      {
//        Blynk.virtualWrite(V1, dTemperature);
//        Blynk.setProperty(V1, "color", BLYNK_GREEN);
//      }
//      else
//      {
//        Blynk.virtualWrite(V1, "error");
//        Blynk.setProperty(V1, "color", BLYNK_RED);
//      }
//      break;
//    case 2:
//      if (bHumidityAvail)
//      {
//        Blynk.setProperty(V2, "color", BLYNK_GREEN);
//        Blynk.virtualWrite(V2, dHumidity);
//      }
//      else
//      {
//        Blynk.setProperty(V2, "color", BLYNK_RED);
//        Blynk.virtualWrite(V2, "error");
//      }
//      break;
//    case 3:
//      if (bPressureAvail)
//      {
//        Blynk.setProperty(V3, "color", BLYNK_GREEN);
//        Blynk.virtualWrite(V3, dPressure);
//      }
//      else
//      {
//        Blynk.setProperty(V3, "color", BLYNK_RED);
//        Blynk.virtualWrite(V3, "error");
//      }
//      break;
//    case 4:
//      if (bLightLuxAvail)
//      {
//        Blynk.setProperty(V4, "color", BLYNK_GREEN);
//        Blynk.virtualWrite(V4, dLightLux);
//      }
//      else
//      {
//        Blynk.setProperty(V4, "color", BLYNK_RED);
//        Blynk.virtualWrite(V4, "error");
//      }
//      break;
//    case 5:
//      if (bWindDirectionAvail)
//      {
//        Blynk.setProperty(V6, "color", BLYNK_GREEN);
//        Blynk.virtualWrite(V6, sWindDirection);
//      }
//      else
//      {
//        Blynk.setProperty(V6, "color", BLYNK_RED);
//        Blynk.virtualWrite(V6, "error");
//      }
//      break;
//    case 6:
//      if (bWindPowerAvail)
//      {
//        Blynk.setProperty(V5, "color", BLYNK_GREEN);
//        Blynk.virtualWrite(V5, dWindPower);
//      }
//      else
//      {
//        Blynk.setProperty(V5, "color", BLYNK_RED);
//        Blynk.virtualWrite(V5, "error");
//      }
//      break;
//    case 7:
//      if (bRainAvail)
//      {
//        if (bRain)
//        {
//          Blynk.setProperty(V7, "color", BLYNK_GREEN);
//          if (iRainLevel == 2)
//          {
//            Blynk.virtualWrite(V7, "LOW");
//          } else if (iRainLevel == 3)
//          {
//            Blynk.virtualWrite(V7, "MEDIUM");
//          } else if (iRainLevel == 4)
//          {
//            Blynk.virtualWrite(V7, "HIGH");
//          }
//        }
//        else
//        {
//          Blynk.setProperty(V7, "color", BLYNK_GREEN);
//          Blynk.virtualWrite(V7, "NONE");
//        }
//      }
//      else
//      {
//        Blynk.setProperty(V7, "color", BLYNK_RED);
//        Blynk.virtualWrite(V7, "ERROR");
//      }
//      break;
//    case 8:
//      if (bWAlarm == true)
//      {
//        Blynk.virtualWrite(V8, "Triggered");
//      }
//      else
//      {
//        Blynk.virtualWrite(V8, "None");
//      }
//      break;
//  }
//  iMQTTUpdateScreen++;
//  if (iMQTTUpdateScreen > 8) iMQTTUpdateScreen = 1;
//}

void DisplayStatus() {
  //Remove screen update
  tsTimer.remove(4);
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  if (WIFIconnected == true)
  {
    display.drawString(0, 0, "CONNECTED");
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, 0, String(WiFi.RSSI()) + " dBm");
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    if (sJSONsendCommand == "")
    {
      display.drawString(0, 11, "IP: " + WiFi.localIP().toString());
      if (MQTTclient.connected())
      {
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString(128, 11, "mqttCON");
        display.setTextAlignment(TEXT_ALIGN_LEFT);
      }
      else
      {
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString(128, 11, "mqttDIS");
        display.setTextAlignment(TEXT_ALIGN_LEFT);
      }
    }
    else
    {
      display.drawString(0, 11, sJSONsendCommand);
      sJSONsendCommand = "";
    }
  }
  else
  {
    display.drawString(0, 0,  "DISCONNECTED");
  }
  display.drawLine(63, 23, 63, 64);
  display.drawLine(0, 23, 128, 23);
  display.setTextAlignment(TEXT_ALIGN_CENTER);

  //Debug only
  //iDisplay = 3;

  switch (iDisplay)
  {
    case 1:
      display.drawString(31, 30, "Temp [ºC]");
      display.drawString(95, 30, "Moist [%]");
      if (bTemperatureAvail)
      {
        display.drawString(31, 47, String(dTemperature));
      }
      else
      {
        display.drawString(31, 47, "ERROR");
      }
      if (bHumidityAvail)
      {
        display.drawString(95, 47, String(dHumidity));
      }
      else
      {
        display.drawString(95, 47, "ERROR");
      }
      break;
    case 2:
      display.drawString(31, 30, "Pres [mBar]");
      display.drawString(95, 30, "Light [Lux]");
      if (bPressureAvail)
      {
        //Straigth line
        display.drawLine(52, 48, 52, 58);
        if (iPressureDirection == 1)
        {
          //Presure up
          display.drawLine(52, 48, 49, 55);
          display.drawLine(52, 48, 55, 55);
        } else if (iPressureDirection == 2)
        {
          //Presure down
          display.drawLine(52, 58, 49, 55);
          display.drawLine(52, 58, 55, 55);
        }
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(5, 47, String(double(dPressure)));
        display.setTextAlignment(TEXT_ALIGN_CENTER);
      }
      else
      {
        display.drawString(31, 47, "ERROR");
      }
      if (bLightLuxAvail)
      {
        display.drawString(95, 47, String(dLightLux));
      }
      else
      {
        display.drawString(95, 47, "ERROR");
      }
      break;
    case 3:
      display.drawString(31, 30, "WndP [km/h]");
      display.drawString(95, 30, "WndDir");
      if (bWindPowerAvail)
      {
        display.drawString(31, 47, String(dWindPower));
      }
      else
      {
        display.drawString(31, 47, "ERROR");
      }
      if (bWindDirectionAvail)
      {
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(72, 47, sWindDirection);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        int Xloc = 118;
        int Yloc = 52;
        int LineLengthStraight = 14;
        int LineLengthDiagonal = 10;
        int CircleRadius = 2;
        display.drawCircle(Xloc, Yloc, 1);
        if (sWindDirection == "N" || sWindDirection == "NNW" || sWindDirection == "NNE")
        {
          //N//NWW//NNE
          //Straight line
          display.drawLine(Xloc, Yloc - (LineLengthStraight / 2), Xloc, Yloc + (LineLengthStraight / 2));
          //Arrow Up
          display.drawCircle(Xloc, Yloc - (LineLengthStraight / 2), CircleRadius);
        } else if (sWindDirection == "S" || sWindDirection == "SSW" || sWindDirection == "SSE")
        {
          //S//SSW//SSE
          //Straight line
          display.drawLine(Xloc, Yloc + (LineLengthStraight / 2), Xloc, Yloc - (LineLengthStraight / 2));
          //Arrow down
          display.drawCircle(Xloc, Yloc + (LineLengthStraight / 2), CircleRadius);
        } else if (sWindDirection == "W" || sWindDirection == "WNW" || sWindDirection == "WSW")
        {
          //W//WNW//WSW
          //Straight line
          display.drawLine(Xloc - (LineLengthStraight / 2), Yloc, Xloc + (LineLengthStraight / 2), Yloc);
          //Arrow Left
          display.drawCircle(Xloc - (LineLengthStraight / 2), Yloc, CircleRadius);
        } else if (sWindDirection == "E" || sWindDirection == "ENE" || sWindDirection == "ESE")
        {
          //E//ENE//ESE
          //Straight line
          display.drawLine(Xloc + (LineLengthStraight / 2), Yloc, Xloc - (LineLengthStraight / 2), Yloc);
          //Arrow Right
          display.drawCircle(Xloc + (LineLengthStraight / 2), Yloc, CircleRadius);
        } else if (sWindDirection == "NE")
        {
          //NE
          //Cross line
          display.drawLine(Xloc + (LineLengthDiagonal / 2), Yloc - (LineLengthDiagonal / 2), Xloc - (LineLengthDiagonal / 2), Yloc + (LineLengthDiagonal / 2));
          //Arrow Top right
          display.drawCircle(Xloc + (LineLengthDiagonal / 2), Yloc - (LineLengthDiagonal / 2), CircleRadius);
        } else if (sWindDirection == "NW")
        {
          //NW
          //Cross line
          display.drawLine(Xloc - (LineLengthDiagonal / 2), Yloc - (LineLengthDiagonal / 2), Xloc + (LineLengthDiagonal / 2), Yloc + (LineLengthDiagonal / 2));
          //Arrow Top left
          display.drawCircle(Xloc - (LineLengthDiagonal / 2), Yloc - (LineLengthDiagonal / 2), CircleRadius);
        } else if (sWindDirection == "SW")
        {
          //SW
          //Cross line
          display.drawLine(Xloc - (LineLengthDiagonal / 2), Yloc + (LineLengthDiagonal / 2), Xloc + (LineLengthDiagonal / 2), Yloc - (LineLengthDiagonal / 2));
          //Arrow Bottom left
          display.drawCircle(Xloc - (LineLengthDiagonal / 2), Yloc + (LineLengthDiagonal / 2), CircleRadius);
        } else if (sWindDirection == "SE")
        {
          //SE
          //Cross line
          display.drawLine(Xloc + (LineLengthDiagonal / 2), Yloc + (LineLengthDiagonal / 2), Xloc - (LineLengthDiagonal / 2), Yloc - (LineLengthDiagonal / 2));
          //Arrow Bottom left
          display.drawCircle(Xloc + (LineLengthDiagonal / 2), Yloc + (LineLengthDiagonal / 2), CircleRadius);
        }
        // display.drawString(95, 55, String(ads.readADC_SingleEnded(ADC_IN_WINDIRECTION) * 0.1875e-3));
      }
      else
      {
        display.drawString(95, 47, "ERROR");
      }
      break;
    case 4:

      if (bHeaterRain)
      {
        display.drawString(31, 30, "Rain [HtON]");
      }
      else
      {
        display.drawString(31, 30, "Rain [HtOff]");
      }
      display.drawString(95, 30, "Alarm");
      if (bRainAvail)
      {
        display.drawString(31, 55, String(ads.readADC_SingleEnded(ADC_IN_RAIN) * 0.1875e-3));

        if (iRainLevel == 2)
        {
          display.drawString(31, 47, "LOW");
        } else if (iRainLevel == 3)
        {
          display.drawString(31, 47, "MEDIUM");
        } else if (iRainLevel == 4)
        {
          display.drawString(31, 47, "HIGH");
        }
        else
        {
          display.drawString(31, 47, String("NONE"));
        }
      }
      else
      {
        display.drawString(31, 47, "ERROR");
      }
      if (bWAlarm) {
        display.drawString(95, 47, String("Triggered"));
      }
      else
      {
        display.drawString(95, 47, String("None"));
      }
      break;
  }
  display.display();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  //Activiate screen update
  tsTimer.add(0, DISPLAY_INTERVAL_MS, UpdateDisplay, false);
}

int GetRequest()
{
  sJSONsendCommand = "Snd: GET";
  DisplayStatus();
  HTTPClient http;
  http.begin(INCOMMING_SERVER);
  int httpCode = http.GET();
  http.end();
  return httpCode;
}

JsonObject& prepareResponse(JsonBuffer & jsonBuffer)
{
  JsonObject& root = jsonBuffer.createObject();
  if (bTemperatureAvail)
  {
    root["dTempOut"] = String(dTemperature);
  }
  else
  {
    root["dTempOut"] = String(-999);
  }
  if (bHumidityAvail)
  {
    root["dHumidityOut"] = String(dHumidity);
  }
  else
  {
    root["dHumidityOut"] = String(-999);
  }
  if (bPressureAvail)
  {
    root["dPressureOut"] = String(dPressure);
  }
  else
  {
    root["dPressureOut"] = String(-999);
  }
  if (bLightLuxAvail)
  {
    root["dLightLuxOut"] = String(dLightLux);
  }
  else
  {
    root["dLightLuxOut"] = String(-999);
  }
  if (bWindDirectionAvail)
  {
    root["sWindDirection"] = sWindDirection;
  }
  else
  {
    root["sWindDirection"] = "ERROR";
  }
  if (bWindPowerAvail)
  {
    root["dWindPower"] = String(dWindPower);
  }
  else
  {
    root["dWindPower"] = String(-999);
  }
  if (bRainAvail)
  {
    if (iRainLevel == 2)
    {
      root["bRain"] = "LOW";
    } else if (iRainLevel == 3)
    {
      root["bRain"] = "MEDIUM";
    } else if (iRainLevel == 4)
    {
      root["bRain"] =  "HIGH";
    }
    else
    {
      root["bRain"] = "NONE";
    }
  }
  else
  {
    root["bRain"] = "ERROR";
  }
  if (bWAlarm == true)
  {
    root["bWAlarm"] = "Triggered";
  }
  else
  {
    root["bWAlarm"] = "None";
  }
  sJSONsendCommand = "Snd Sensor status";
  DisplayStatus();
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
        Serial.println("Client Request Confirmed.");
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

bool checkMQTT()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    if (MQTTclient.connected() == false)
    {
      if (MQTTclient.connect("BokkeLul", MQTT_USERNAME, MQTT_KEY))
      {
        UpdateMQTT();
        return true;
      } else
      {
        return false;
      }
    }
    else
    {
      UpdateMQTT();
      return true;
    }
  }
  return false;
}


//bool checkBlynk()
//{
//  if (WiFi.status() == WL_CONNECTED)
//  {
//    if (Blynk.connected() == false) {
//      Blynk.connect();
//      return false;
//    }
//    else
//    {
//      UpdateBlynk();
//      return true;
//    }
//  }
//}




