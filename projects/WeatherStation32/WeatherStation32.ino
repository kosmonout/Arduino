#ifdef ARDUINO_ESP32_DEV
#include <WiFi.h>
#include <HTTPClient.h>
#else
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#endif

/* create a hardware timer */
hw_timer_t * timerDispl = NULL;
hw_timer_t * timerMinute = NULL;
hw_timer_t * timerSensors = NULL;
portMUX_TYPE timerMuxDispl = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMuxMinute = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMuxSensors = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE InteruptMux = portMUX_INITIALIZER_UNLOCKED;

#define DISPLAY_INTERVAL_MS 2500
#define CONTACT_MINUTE_INTERVAL_MS 60000
#define UPDATE_SENSOR_MS 3000
#define CHECK_MQTT_MS 1000

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

#define MAXIMUM_MINUTE_LOOP 30
#define GEIGER_COUNT_STARTS_MINUTE 0
#define GEIGER_SAMPLE_MINUTES 2
#define AIR_QUALITY_COUNT_STARTS_MINUTE 3
#define AIR_QUALITY_SAMPLE_MINUTES 2
#define CPM_TO_USV_HOUR 0.0064648101265823
#define WIND_ALARM 20 // 
#define COUT_WIND_ALARM 50

#include <ArduinoJson.h>
#include "SSD1306.h"
#include "BMP280.h"
#include "SHT31.h"
#include "MAX44009.h"
#include "ADS1015.h"
#include <SDS011.h>
#include <PubSubClient.h>

//#define BLYNK_PRINT Serial
//#define BLYNK_GREEN     "#23C48E"
//#define BLYNK_BLUE      "#04C0F8"
//#define BLYNK_YELLOW    "#ED9D00"
//#define BLYNK_RED       "#D3435C"
//#define BLYNK_DARK_BLUE "#5F7CD8"
//#include <BlynkSimpleEsp8266.h>
//Inputs
#define PIN_SDA 21
#define PIN_SCL 22
#define SERIAL1_RXPIN 12
#define SERIAL1_TXPIN 13
#define PIN_HEATERON 16
#define PIN_POWERWIND 14
#define PIN_GEIGER_TRIGGERED 19
#define PIN_GEIGER_ON 2
#define PIN_AIR_QUALITY_ON 4

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

#define ADC_IN_WINDPOWER 0
#define ADC_IN_WINDIRECTION 1
#define ADC_IN_RAIN 2
#define ADDR_SHT31 0x44 // sht31 bus address Moisture/Temp
#define ADDR_SSD1306 0x3c // SSD1306 bus address Display
#define ADDR_MAX44009 0x4a // MAX44009 bus address Light lux
#define ADDR_BMP280 0x77
#define CHIPID_BMP280 0x58
#define ADDR_ADS1115 0x48

//Start network
WiFiClient client;
PubSubClient MQTTclient(client);
WiFiServer server(HTTP_PORT);
SSD1306  display(ADDR_SSD1306);
BMP280 bmp;
SHT31 sht31;
Max44009 myLux(ADDR_MAX44009);
ADS1115 ads(ADDR_ADS1115);
HTTPClient http;
HardwareSerial Serial1(1);
SDS011 my_sds(Serial1);

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
void ExecuteCore0(void * parameter);
void ExecuteCore1(void * parameter);
void MinuteTimer();
void ISR_geiger();
void CheckMQTTTimer();
void UpdateDisplayTimer();
void UpdateSensorsTimer();

//void UpdateBlynk();
//bool checkBlynk();

uint8_t crc8(const uint8_t *data, uint8_t len);

bool readRequest(WiFiClient& client);
JsonObject& prepareResponse(JsonBuffer& jsonBuffer);
void writeResponse(WiFiClient& client, JsonObject& json);

int GetRequest();
int iDisplay = 1;
bool bWIFIconnected = false;
bool bClientConnected = false;
double dTemperature;
double dHumidity;
double dPressure;
double dPressurePrev = 0;
int iPressureDirection = 0;
double dLightLux;
double dWindPower;
String sWindDirection;
int iRadiationCount = 0;
int iRadiationTotal = 0;
int iPreviousTime = 0;
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
bool bUpdateDisplayTimer = false;
bool bUpdateSensorsTimer = false;
bool bCheckMQTTTimer = false;
int iWiFiStrength = 0;
bool bMQTT_connected = false;
String sWiFiIP;
double dUsvHour;
float fPm2_5, fPm10;
int iMinuteCountDebug = 0;
bool bMinuteTimer = true;
int iMinuteCount = 0;
int iCountHTTPInterval = 0;
bool bAirQualitySampleStart = false;

TaskHandle_t Task1, Task2;

////////////////////////////////////////
void setup(void)
{
  xTaskCreatePinnedToCore
  (
    ExecuteCore0, /* Task function. */
    "ExecuteCore0", /* name of task. */
    1024 * 8, /* Stack size of task */
    NULL, /* parameter of the task */
    1, /* priority of the task */
    NULL, /* Task handle to keep track of created task */
    0); /* pin task to core 0 */

  vTaskDelay( 2000 / portTICK_PERIOD_MS);

  xTaskCreatePinnedToCore
  (
    ExecuteCore1, /* Task function. */
    "ExecuteCore1", /* name of task. */
    1024 * 8, /* Stack size of task */
    NULL, /* parameter of the task */
    1, /* priority of the task */
    NULL, /* Task handle to keep track of created task */
    1); /* pin task to core 0 */
}

////////////////////////////////////////////////////////////////////////////////
void loop(void)
{

}

//#####################################################################################################

void  ExecuteCore1(void * parameter )
{
  Wire.begin(PIN_SDA, PIN_SCL); // the SDA and SCL
  Wire.setClock(400000);
  Serial1.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  Serial.begin(115200);
  pinMode(PIN_HEATERON, OUTPUT);
  pinMode(PIN_GEIGER_ON, OUTPUT);
  pinMode(PIN_AIR_QUALITY_ON, OUTPUT);
  pinMode(PIN_POWERWIND, OUTPUT);
  digitalWrite(PIN_HEATERON, LOW);
  digitalWrite(PIN_POWERWIND, LOW);
  digitalWrite(PIN_GEIGER_ON, LOW);
  digitalWrite(PIN_AIR_QUALITY_ON, LOW);
  display.init();
  display.setContrast(255);
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawStringMaxWidth(0, 0, 128, "Booting.....");
  display.display();
  iRainLevel = 0;

  sht31.begin(ADDR_SHT31);
  uint32_t serialNo;
  if ( !sht31.readSerialNo(serialNo))
  {
    {
      display.drawStringMaxWidth(0, 16, 128, "NO I2C SHT31");
    }
  }
  else
  {
    if (!sht31.readTempHum())
    {
      display.drawStringMaxWidth(0, 16, 128, "NO VDD SHT31");
    }
    else
    {
      display.drawStringMaxWidth(0, 16, 128, "Found SHT31");
    }
  }
  display.display();
  dLightLux = myLux.getLux(); //dummy read to generate an error
  if (myLux.getError() != 0)
  {
    display.drawStringMaxWidth(0, 28, 128, "NOT found MAX77009");
  }
  else
  {
    display.drawStringMaxWidth(0, 28, 128, "Found MAX77009");
  }
  display.display();
  Wire.beginTransmission(ADDR_ADS1115);
  uint8_t ec = Wire.endTransmission(true); // if device exists on bus, it will aCK
  if (ec != 0)  // Device ACK'd
  {
    display.drawStringMaxWidth(0, 40, 128, "NOT found AD conv");
  }
  else
  {
    display.drawStringMaxWidth(0, 40, 128, "Found AD conv");
    ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  }
  display.display();

  if (!bmp.begin(ADDR_BMP280, CHIPID_BMP280))
  {
    display.drawStringMaxWidth(0, 53, 128, "NOT found Presure sensor");
  }
  else
  {
    display.drawStringMaxWidth(0, 52, 128, "Found Presure sensor");
  }
  display.display();

  /* Use 1st timer of 4 */
  /* 1 tick take 1/(80MHZ/80000) = 1ms so we set divider 80000 and count up */
  timerDispl = timerBegin(0, 80, true);
  timerAttachInterrupt(timerDispl, &UpdateDisplayTimer, true);
  timerAlarmWrite(timerDispl, DISPLAY_INTERVAL_MS * 1000, true);

  timerMinute = timerBegin(1, 80, true);
  timerAttachInterrupt(timerMinute, &MinuteTimer, true);
  timerAlarmWrite(timerMinute, CONTACT_MINUTE_INTERVAL_MS * 1000, true);

  timerSensors = timerBegin(2, 80, true);
  timerAttachInterrupt(timerSensors, &UpdateSensorsTimer, true);
  timerAlarmWrite(timerSensors, UPDATE_SENSOR_MS * 1000, true);


  // Blynk.run();
  //  checkBlynk();
#ifdef ARDUINO_ESP32_DEV
  vTaskDelay( 2000 / portTICK_PERIOD_MS);
#else
  delay(2000);
#endif
  timerAlarmEnable(timerDispl);
  timerAlarmEnable(timerMinute);
  //For the first 0 minute activation
  timerAlarmEnable(timerSensors);

  attachInterrupt(digitalPinToInterrupt(PIN_GEIGER_TRIGGERED), ISR_geiger, FALLING);
  // Initialising the UI will init the display too.
  display.clear();
  UpdateSensors();
  //LOOP FUNCTION *****************************************************************
  for (;;) {
    DisplayStatus();
    // String taskMessage = "loop running on core ";
    //  taskMessage = taskMessage + xPortGetCoreID();
    // Serial.println(taskMessage);

    if (bUpdateDisplayTimer == true)
    {
      // Serial.println("Update Display");
      bUpdateDisplayTimer = false;
      timerStop(timerDispl);
      timerStop(timerSensors);
      UpdateDisplay();
      timerStart(timerDispl);
      timerStart(timerSensors);
    }

    if (bUpdateSensorsTimer)
    {
      //  Serial.println("UpdateSensorsTimer");
      bUpdateSensorsTimer = false;
      timerStop(timerDispl);
      timerStop(timerSensors);
      UpdateSensors();
      timerStart(timerDispl);
      timerStart(timerSensors);
    }

    if (bAirQualitySampleStart == true)
    {
      int error;
      error = my_sds.read(&fPm2_5, &fPm10);
      if (! error)
      {
        bAirQualitySampleStart == true;
      }
      else
      {
        fPm2_5 = -999;
        fPm10 = -999;
      }
    }

    if (bMinuteTimer)
    {
      bMinuteTimer = false;
      if (iMinuteCount >= MAXIMUM_MINUTE_LOOP) iMinuteCount = 0;
      //GEIGER COUNT STARTS
      if (iMinuteCount == GEIGER_COUNT_STARTS_MINUTE)
      {
        digitalWrite(PIN_GEIGER_ON, HIGH);
        iRadiationCount = 0;
      }
      else if (iMinuteCount >= (GEIGER_COUNT_STARTS_MINUTE + GEIGER_SAMPLE_MINUTES))
      {
        digitalWrite(PIN_GEIGER_ON, LOW);
        //0.2 samples per second self inducted false reading
        if (iRadiationCount > 0)
        {
          iRadiationTotal = (iRadiationCount - (12 * GEIGER_SAMPLE_MINUTES)) / GEIGER_SAMPLE_MINUTES;
          dUsvHour = iRadiationTotal  * CPM_TO_USV_HOUR;
          if (iRadiationTotal < 0) iRadiationTotal = 0;
        }
        else
        {
          // No counts being made cannot be so broken communication
          iRadiationTotal = -999;
        }
      }
      //AIR QUALITY COUNT STARTS
      if (iMinuteCount == AIR_QUALITY_COUNT_STARTS_MINUTE)
      {
        digitalWrite(PIN_AIR_QUALITY_ON, HIGH);
        bAirQualitySampleStart = true;
      }
      else if (iMinuteCount >= (AIR_QUALITY_COUNT_STARTS_MINUTE + AIR_QUALITY_SAMPLE_MINUTES))
      {
        digitalWrite(PIN_AIR_QUALITY_ON, LOW);
        bAirQualitySampleStart = false;
      }
      Serial.println("iMinuteCount");
      Serial.println(iMinuteCount);
      Serial.println("iRadiationTotal");
      Serial.println(iRadiationTotal);
      //timerRestart(timerMinute);
    }
  }
}

void  ExecuteCore0(void * parameter )
{
  WiFi.disconnect();
  WiFi.begin(SSID, PASSWORD);
  sWiFiIP = WiFi.localIP().toString();
  MQTTclient.setServer(MQTT_SERVER, MQTT_PORT);
  //Blynk.config(BLYNK_AUTH);
  // Blynk.connect(10);
  bWIFIconnected = false;

  //LOOP FUNCTION *****************************************************************
  for (;;) {
    //    String taskMessage = "HTTPClient on core ";
    //    taskMessage = taskMessage + xPortGetCoreID();
    //    Serial.println(taskMessage);
    // Check if module is still connected to WiFi.
    if (WiFi.status() != WL_CONNECTED)
    {
      if (bWIFIconnected == true)
      {
        server.close();
      }
      bWIFIconnected = false;
      bMQTT_connected = false;
      bClientConnected = false;
    }
    else
    {
      bWIFIconnected = true;
      iWiFiStrength = WiFi.RSSI();
      server.begin();
      sWiFiIP = WiFi.localIP().toString();
      bWIFIconnected = true;
      if (iCountHTTPInterval == 10)
      {
        GetRequest();
        iCountHTTPInterval = 0;
      }
      bMQTT_connected = checkMQTT();
      iCountHTTPInterval++;
      WiFiClient client = server.available();
      if (client)
      {
        bool success = readRequest(client);
        if (success)
        {
          //Send JSON
          //Serial.println("readRequest success");
          StaticJsonBuffer<280> jsonWriteBuffer;
          JsonObject& jsonWrite = prepareResponse(jsonWriteBuffer);
          writeResponse(client, jsonWrite);
          bClientConnected = true;

#ifdef ARDUINO_ESP32_DEV
          vTaskDelay( 1 / portTICK_PERIOD_MS);
#else
          delay(1);
#endif
          client.stop();
        }
        else
        {
          bClientConnected = false;
          //Serial.println("readRequest unsuccessfull");
        }
      }
    }
#ifdef ARDUINO_ESP32_DEV
    vTaskDelay( 1000 / portTICK_PERIOD_MS);
#else
    delay(1000);
#endif
  }
}

void ISR_geiger() { // Captures count of events from Geiger counter board
  portENTER_CRITICAL_ISR(&InteruptMux);
  if ((iMinuteCount >= GEIGER_COUNT_STARTS_MINUTE) && iMinuteCount < (GEIGER_COUNT_STARTS_MINUTE + GEIGER_SAMPLE_MINUTES))
  {
    int iCurrentTime = millis();
    //detachInterrupt(PIN_GEIGER_TRIGGERED);
    if ((iCurrentTime - iPreviousTime) > 1)iRadiationCount++;
    // attachInterrupt(digitalPinToInterrupt(PIN_GEIGER_TRIGGERED), ISR_geiger, RISING);
    int iTotalTime = iCurrentTime - iPreviousTime;
    //String printalles = String(iRadiationCount) + "_"  + String(iTotalTime);
    //Serial print will hang the device
    //Serial.println(printalles);
    iPreviousTime = iCurrentTime;
  }
  portEXIT_CRITICAL_ISR(&InteruptMux);
}

void IRAM_ATTR UpdateDisplayTimer()
{
  portENTER_CRITICAL_ISR(&timerMuxDispl);
  bUpdateDisplayTimer = true;
  portEXIT_CRITICAL_ISR(&timerMuxDispl);
}

void IRAM_ATTR MinuteTimer()
{
  portENTER_CRITICAL_ISR(&timerMuxMinute);
  bMinuteTimer = true;
  iMinuteCount++;
  iMinuteCountDebug++;
  portEXIT_CRITICAL_ISR(&timerMuxMinute);
}

void IRAM_ATTR UpdateSensorsTimer()
{
  portENTER_CRITICAL_ISR(&timerMuxSensors);
  bUpdateSensorsTimer = true;
  portEXIT_CRITICAL_ISR(&timerMuxSensors);
}

void  UpdateDisplay()
{
  iDisplay++;
  if (iDisplay > 6) iDisplay = 1;
  sJSONsendCommand = "";
}

void UpdateSensors()
{
  //vTaskSuspend(timerDispl);
  // vTaskSuspend(timerMinute);
  // vTaskSuspend(timerSensors);
  digitalWrite(PIN_POWERWIND, HIGH);

  dTemperature = sht31.readTemperature() + TEMPERATURE_CAL;
  Serial.println("readTemperature");
  Serial.println(String(dTemperature));
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
  Serial.println(String(dPressure));
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
  // vTaskResume(timerDispl);
  //  vTaskResume(timerMinute);
  // vTaskResume(timerSensors);
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
    case 9:
      if (iRadiationTotal  >= 0)
      {
        MQTTclient.publish("Rad_CPM", String(iRadiationTotal).c_str());
      }
      else
      {
        MQTTclient.publish("Rad_CPM", "Error");
      }
      break;
    case 10:
      if (iRadiationTotal  >= 0)
      {
        MQTTclient.publish("Rad_usv", String(dUsvHour).c_str());
      }
      else
      {
        MQTTclient.publish("Rad_usv", "Error");
      }
      break;
    case 11:
      if (iRadiationTotal  >= 0)
      {
        MQTTclient.publish("PM2_5", String(fPm2_5).c_str());
      }
      else
      {
        MQTTclient.publish("PM2_5", "Error");
      }
      break;
    case 12:
      if (iRadiationTotal  >= 0)
      {
        MQTTclient.publish("PM10", String(fPm10).c_str());
      }
      else
      {
        MQTTclient.publish("PM10", "Error");
      }
      break;
  }
  iMQTTUpdateScreen++;
  if (iMQTTUpdateScreen > 12) iMQTTUpdateScreen = 1;
}

void  DisplayStatus() {
  //vTaskSuspend(timerDispl);
  // vTaskSuspend(timerMinute);
  // vTaskSuspend(timerSensors);
  //Remove screen update
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  if (bWIFIconnected == true)
  {
    if (sJSONsendCommand == "")
    {
      display.drawString(0, 0, sWiFiIP);
      display.setTextAlignment(TEXT_ALIGN_RIGHT);
      display.drawString(128, 0, String(iWiFiStrength) + " dBm");
      display.setTextAlignment(TEXT_ALIGN_LEFT);
    }
    else
    {
      display.drawString(0, 0, sJSONsendCommand);
    }
    if (bMQTT_connected)
    {
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.drawString(0, 11, "mqtt CON");
      //display.setTextAlignment(TEXT_ALIGN_LEFT);
    }
    else
    {
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.drawString(0, 11, "mqtt DIS");
      //display.setTextAlignment(TEXT_ALIGN_LEFT);
    }
    if (bClientConnected)
    {
      display.setTextAlignment(TEXT_ALIGN_RIGHT);
      display.drawString(128, 11, "clnt CON");
      display.setTextAlignment(TEXT_ALIGN_LEFT);
    }
    else
    {
      display.setTextAlignment(TEXT_ALIGN_RIGHT);
      display.drawString(128, 11, "clnt DIS");
      display.setTextAlignment(TEXT_ALIGN_LEFT);
    }
    //Debug Minute
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 11, String(iMinuteCountDebug));
    display.setTextAlignment(TEXT_ALIGN_LEFT);
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
    case 5 :
      display.drawString(31, 30, "Rad [CPM]");
      display.drawString(95, 30, "Rad [µSv/h]");
      if (iRadiationTotal  >= 0)
      {
        display.drawString(31, 47, String(iRadiationTotal));
      }
      else
      {
        display.drawString(31, 47, "ERROR");
      }
      if (iRadiationTotal  >= 0)
      {
        display.drawString(95, 47, String(dUsvHour));
      }
      else
      {
        display.drawString(95, 47, "ERROR");
      }
      break;
    case 6 :
      display.drawString(31, 30, "PM2.5 [μg/m3]");
      display.drawString(95, 30, "PM10 [μg/m3]");
      if (iRadiationTotal >= 0)
      {
        display.drawString(31, 47, String(fPm2_5));
      }
      else
      {
        display.drawString(31, 47, "ERROR");
      }
      if (iRadiationTotal  >= 0)
      {
        display.drawString(95, 47, String(fPm10));
      }
      else
      {
        display.drawString(95, 47, "ERROR");
      }
      break;
  }
  display.display();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  //Activiate screen update
  // vTaskResume(timerDispl);
  // vTaskResume(timerMinute);
  // vTaskResume(timerSensors);
}

int GetRequest()
{
  sJSONsendCommand = "Snd: GET";
  //DisplayStatus();
  http.begin(INCOMMING_SERVER);
  int httpCode = http.GET();
  http.end();
  return httpCode;
}

JsonObject& prepareResponse(JsonBuffer & jsonBuffer)
{
  sJSONsendCommand = "Snd Sensor status";
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
  if (iRadiationTotal  >= 0)
  {
    root["iRad_CPM"] = String(iRadiationTotal);
  }
  else
  {
    root["iRad_CPM"] = String(-999);
  }
  if (iRadiationTotal  >= 0)
  {
    root["dRad_usv"] = String(dUsvHour);
  }
  else
  {
    root["dRad_usv"] = String(-999);
  }
  if (iRadiationTotal  >= 0)
  {
    root["fPM2_5"] = String(fPm2_5);
  }
  else
  {
    root["fPM2_5"] = String(-999);
  }
  if (iRadiationTotal >= 0)
  {
    root["fPM10"] = String(fPm10);
  }
  else
  {
    root["fPM10"] = String(-999);
  }
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

bool checkMQTT()
{
  if (bWIFIconnected)
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





