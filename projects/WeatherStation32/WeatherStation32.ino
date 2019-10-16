#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "SSD1306.h"
#include "BMP280.h"
#include "SHT31.h"
#include "MAX44009.h"
#include "ADS1015.h"
#include <SDS011.h>
#include <PubSubClient.h>

/* create a hardware timer */
hw_timer_t * timerDispl = NULL;
hw_timer_t * timerMinute = NULL;
hw_timer_t * timerSensors = NULL;
hw_timer_t * timerWatchDog = NULL;
portMUX_TYPE timerMuxDispl = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMuxMinute = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMuxSensors = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMuxWatchDog = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE InteruptMux = portMUX_INITIALIZER_UNLOCKED;


#define DISPLAY_INTERVAL_MS 2500
#define CONTACT_MINUTE_INTERVAL_MS 60000
#define UPDATE_SENSOR_MS 3000
#define WATCHDOG_TIMEOUT_MS 10000
#define HHTP_REQUEST_INTERVAL_SEC 60

#define HTTP_PORT 80
#define HEATER_ON_MOISTURE 85
#define HEATER_OFF_MOISTURE 83
#define TEMPERATURE_CAL -0.35
#define DELTA_AIR_PRESURE_ARROW 0.5
#define RAIN_MAX 3.0
#define RAIN_MEDIUM 4.0
#define RAIN_LOW 4.8
#define RAIN_NO 5.1
#define SSID "kosmos"
#define PASSWORD "funhouse"
#define INCOMMING_SERVER "http://192.168.2.165/api/app/com.internet/weather"
#define MQTT_SERVER "m23.cloudmqtt.com"
#define MQTT_PORT 17768
#define MQTT_USERNAME   "nzubqmrm"
#define MQTT_KEY        "QQiE_DzdaWEu"

#define MAXIMUM_MINUTE_LOOP 30
#define GEIGER_COUNT_STARTS_MINUTE 3
#define GEIGER_SAMPLE_MINUTES 2
#define AIR_QUALITY_COUNT_STARTS_MINUTE 0
#define AIR_QUALITY_SAMPLE_MINUTES 1
#define CPM_TO_USV_HOUR 0.0064648101265823
#define WIND_ALARM 6 // in m/s
#define AIR_QUALITY_ERROR_COUNT 10
#define AIR_SAMPLE_TIME_START_AFTER_SECONDS 30



//#define BLYNK_PRINT Serial
//#define BLYNK_GREEN     "#23C48E"
//#define BLYNK_BLUE      "#04C0F8"
//#define BLYNK_YELLOW    "#ED9D00"
//#define BLYNK_RED       "#D3435C"
//#define BLYNK_DARK_BLUE "#5F7CD8"
//#include <BlynkSimpleEsp8266.h>
//Inputs
#define PIN_SDA 18
#define PIN_SCL 19
#define SERIAL1_RXPIN 16
#define SERIAL1_TXPIN 17
#define PIN_HEATER_ON  33
#define PIN_SPEEDWIND 32
#define PIN_GEIGER_TRIGGERED 21
#define PIN_GEIGER_ON 2
#define PIN_AIR_QUALITY_ON 4

#define WIND_ANGLE_0 0.04
#define WIND_ANGLE_1 0.35
#define WIND_ANGLE_2 0.66
#define WIND_ANGLE_3 0.87
#define WIND_ANGLE_4 1.28
#define WIND_ANGLE_5 1.58
#define WIND_ANGLE_6 1.89
#define WIND_ANGLE_7 2.20
#define WIND_ANGLE_8 2.51
#define WIND_ANGLE_9 2.82
#define WIND_ANGLE_10 3.13
#define WIND_ANGLE_11 3.44
#define WIND_ANGLE_12 3.75
#define WIND_ANGLE_13 4.06
#define WIND_ANGLE_14 4.37
#define WIND_ANGLE_15 4.68
#define NORTH_POS_ANGLE WIND_ANGLE_13

#define WIND_DIR_0 "N"
#define WIND_DIR_1 "NNE"
#define WIND_DIR_2 "NE"
#define WIND_DIR_3 "ENE"
#define WIND_DIR_4 "E"
#define WIND_DIR_5 "ESE"
#define WIND_DIR_6 "SE"
#define WIND_DIR_7 "SSE"
#define WIND_DIR_8 "S"
#define WIND_DIR_9 "SSW"
#define WIND_DIR_10 "SW"
#define WIND_DIR_11 "WSW"
#define WIND_DIR_12 "W"
#define WIND_DIR_13 "WNW"
#define WIND_DIR_14 "NW"
#define WIND_DIR_15 "NNW"


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
extern HardwareSerial Serial1;
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
void CoreI2C(void * parameter);
void CoreWiFi(void * parameter);
void MinuteTimer();
void ISR_geiger();
void CheckMQTTTimer();
void UpdateDisplayTimer();
void UpdateSensorsTimer();
void resetModule();


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
double dWindSpeed;
String sWindDirection;
int iRadiationCount = 0;
int iRadiationTotal = 0;
int iPreviousTime = 0;
bool bTemperatureAvail;
bool bHumidityAvail;
bool bPressureAvail;
bool bLightLuxAvail;
bool bWindSpeedAvail;
bool bWindDirectionAvail;
bool bRainAvail;
double dRainVoltage;
double dWindDirVoltage;
double dWindSpdVoltage;
boolean bHeaterRain = false;
int iRainLevel = 0;
int iAlarmCount;
int iMQTTUpdateScreen = 1;
int dWindAngle = 0;
boolean bWAlarm = false;
boolean bFirstWalarm = true;;
String sJSONsendCommand;
String sRebootMessage;
bool bUpdateDisplayTimer = false;
bool bUpdateSensorsTimer = false;
bool bCheckMQTTTimer = false;
int iWiFiStrength = 0;
bool bMQTT_connected = false;
String sWiFiIP;
double dUsvHour;
float fPm2_5, fPm10;
int iMinuteCountDebug = 0;
int iHourCountDebug = 0;
bool bMinuteTimer = true;
int iMinuteCount = 0;
int iCountHTTPInterval = 0;
bool bAirQualitySampleStart = false;
int iAirReadErrorCount;
unsigned long ulAirQualStartSampleTimeSeconds  ;

TaskHandle_t Task1, Task2;

////////////////////////////////////////

//extern "C" void app_main()
void setup(void)
{
  xTaskCreatePinnedToCore
  (
    CoreWiFi, /* Task function. */
    "CoreWiFi", /* name of task. */
    12 * 1024, /* Stack size of task */
    NULL, /* parameter of the task */
    1, /* priority of the task */
    NULL, /* Task handle to keep track of created task */
    0 /* pin task to core */
  );

  delay(2000);

  xTaskCreatePinnedToCore
  (
    CoreI2C, /* Task function. */
    "CoreI2C", /* name of task. */
    8 * 1024, /* Stack size of task */
    NULL, /* parameter of the task */
    1, /* priority of the task */
    NULL, /* Task handle to keep track of created task */
    1 /* pin task to core  */
  );

}
////////////////////////////////////////////////////////////////////////////////

void loop(void)
{
}


//#####################################################################################################

void  CoreI2C(void * parameter )
{
  Wire.begin(PIN_SDA, PIN_SCL); // the SDA and SCL
  Wire.setClock(400000);
  Serial1.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  //Air Quality in sleep mode
  my_sds.sleep();
  Serial.begin(115200);
  pinMode(PIN_HEATER_ON , OUTPUT);
  pinMode(PIN_GEIGER_ON, OUTPUT);
  pinMode(PIN_AIR_QUALITY_ON, OUTPUT);
  pinMode(PIN_SPEEDWIND, OUTPUT);
  digitalWrite(PIN_HEATER_ON , LOW);
  digitalWrite(PIN_SPEEDWIND, LOW);
  digitalWrite(PIN_GEIGER_ON, LOW);
  digitalWrite(PIN_AIR_QUALITY_ON, LOW);
  //Reboot when the i2c communication for the display is not started
  Serial.println("Start");

  Wire.beginTransmission(ADDR_SSD1306);
  if (Wire.endTransmission(true) != 0)  // Device ACK'd
  {
    Serial.println("i2C communication failed... Rebooting..");
    ESP.restart();
  }
  //
  display.init();
  display.setContrast(255);
  //display.flipScreenVertically();
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
  if (Wire.endTransmission(true) != 0)  // Device ACK'd
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

  timerWatchDog = timerBegin(3, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timerWatchDog, &resetModule, true);  //attach callback
  timerAlarmWrite(timerWatchDog, WATCHDOG_TIMEOUT_MS * 1000, false); //set time in us
  timerAlarmEnable(timerWatchDog); //enable interrupt


  vTaskDelay( 2000 / portTICK_PERIOD_MS);

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

    //Reset Wachtdog Timer
    timerWrite(timerWatchDog, 0);

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
      if ((millis() / 1000) > (ulAirQualStartSampleTimeSeconds   + AIR_SAMPLE_TIME_START_AFTER_SECONDS))
      {
        int error;
        error = my_sds.read(&fPm2_5, &fPm10);
        if (! error)
        {
          // Serial.println("AIR QUALITY sample");
          iAirReadErrorCount = 0;
          bAirQualitySampleStart == false;
        }
        else
        {
          iAirReadErrorCount++;
          if (iAirReadErrorCount > AIR_QUALITY_ERROR_COUNT)
          {
            // Serial.println("AIR QUALITY error");
            fPm2_5 = -999;
            fPm10 = -999;
            iAirReadErrorCount = 0;
            bAirQualitySampleStart == false;
          }
        }
      }
    }

    if (bMinuteTimer)
    {
      bMinuteTimer = false;
      if (iMinuteCount >= MAXIMUM_MINUTE_LOOP) iMinuteCount = 0;
      //GEIGER COUNT STARTS
      if (iMinuteCount == GEIGER_COUNT_STARTS_MINUTE)
      {
        //Serial.println("Geiger starts");
        digitalWrite(PIN_GEIGER_ON, HIGH);
        iRadiationCount = 0;
      }
      else if (iMinuteCount == (GEIGER_COUNT_STARTS_MINUTE + GEIGER_SAMPLE_MINUTES))
      {
        // Serial.println("Geiger stops");
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
        //Serial.println("AIR QUALITY starts");
        digitalWrite(PIN_AIR_QUALITY_ON, HIGH);
        bAirQualitySampleStart = true;
        my_sds.wakeup();
        ulAirQualStartSampleTimeSeconds   = millis() / 1000;
      }
      else if (iMinuteCount == (AIR_QUALITY_COUNT_STARTS_MINUTE + AIR_QUALITY_SAMPLE_MINUTES))
      {
        //Serial.println("AIR QUALITY stops");
        my_sds.sleep();
        digitalWrite(PIN_AIR_QUALITY_ON, LOW);
        bAirQualitySampleStart = false;
      }
      //Serial.println("iMinuteCount");
      //Serial.println(iMinuteCount);
      //Serial.println("iRadiationTotal");
      //Serial.println(iRadiationTotal);
      //timerRestart(timerMinute);
    }
  }
}

void  CoreWiFi(void * parameter )
{
  WiFi.setAutoReconnect(true);
  WiFi.begin(SSID, PASSWORD);
  WiFi.waitForConnectResult();
  sWiFiIP = WiFi.localIP().toString();
  MQTTclient.setServer(MQTT_SERVER, MQTT_PORT);
  int iTimeToReboot = 20;
  //LOOP FUNCTION *****************************************************************
  for (;;)
  {
    //    String taskMessage = "HTTPClient on core ";
    //    taskMessage = taskMessage + xPortGetCoreID();
    //    Serial.println(taskMessage);
    // Check if module is still connected to WiFi.
    if (!WiFi.isConnected())
    {
      sRebootMessage = "Reset in " + String(iTimeToReboot) + "  seconds";
      iTimeToReboot--;
      //Serial.println("Disconnected");
      if (bWIFIconnected == true)
      {
        server.close();
      }
      WiFi.disconnect(true);
      WiFi.begin(SSID, PASSWORD);
      bWIFIconnected = false;
      bMQTT_connected = false;
      bClientConnected = false;
      if (iTimeToReboot <= 0)
      {
        iTimeToReboot = 0;
        Serial.println("Network communication failed... Rebooting..");
        ESP.restart();
      }
    }
    else
    {
      //Serial.println("Connected");
      if (bWIFIconnected == false)
      {
        server.begin();
      }
      bWIFIconnected = true;
      iWiFiStrength = WiFi.RSSI();
      sWiFiIP = WiFi.localIP().toString();
      if (iCountHTTPInterval == HHTP_REQUEST_INTERVAL_SEC || (bWAlarm == true && bFirstWalarm == true))
      {
        GetRequest();
        iCountHTTPInterval = 0;
        bFirstWalarm = false;
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
          StaticJsonBuffer<280> jsonWriteBuffer;
          JsonObject& jsonWrite = prepareResponse(jsonWriteBuffer);
          writeResponse(client, jsonWrite);
          bClientConnected = true;
          Serial.println("readRequest success");

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

void IRAM_ATTR resetModule() {
  portENTER_CRITICAL_ISR(&timerMuxWatchDog);
  Serial.println("Watch Dog Reboot!");
  esp_restart();
  portEXIT_CRITICAL_ISR(&timerMuxWatchDog);
}

void IRAM_ATTR MinuteTimer()
{
  portENTER_CRITICAL_ISR(&timerMuxMinute);
  bMinuteTimer = true;
  iMinuteCount++;
  iMinuteCountDebug++;
  if (iMinuteCountDebug > 59)
  {
    iMinuteCountDebug = 0;
    iHourCountDebug++;
  }
  // Reset after one year of operational time
  if (iHourCountDebug > 8759)
  {
    iHourCountDebug = 0;
  }
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
  digitalWrite(PIN_SPEEDWIND, HIGH);
  dTemperature = sht31.readTemperature() + TEMPERATURE_CAL;
  //Serial.println("readTemperature");
  //Serial.println(String(dTemperature));
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
    bHumidityAvail = false;
  }
  else
  {
    bHumidityAvail = true;
  }
  //If we do not know the moisture level put the heater on
  //Put the heater if the moisture level is too high or the rainsensor start to conduct
  if (dHumidity >= HEATER_ON_MOISTURE || iRainLevel > 0 || bHumidityAvail == false)
  {
    digitalWrite(PIN_HEATER_ON , HIGH);
    bHeaterRain = true;
  }
  else if ((dHumidity <= HEATER_OFF_MOISTURE) && (iRainLevel == 0))
  {
    digitalWrite(PIN_HEATER_ON , LOW);
    bHeaterRain = false;
  }
  dPressure = bmp.readPressure() / 100;
  //Serial.println(String(dPressure));
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
  dWindSpdVoltage = ads.readADC_SingleEnded(ADC_IN_WINDPOWER);
  if (dWindSpdVoltage == 0xFFFF)
  {
    bWindSpeedAvail = false;
  }
  else
  {
    // 0 - 30 m/s     0-5V     0.167 mV 60mV offset
    bWindSpeedAvail = true;
    dWindSpdVoltage = dWindSpdVoltage * 0.1875e-3;
    dWindSpeed = (dWindSpdVoltage - 0.07) * 6;
    if (dWindSpeed < 0)
    {
      dWindSpeed = 0;
    }
    Serial.println("dWindSpdVoltage");
    Serial.println(dWindSpdVoltage);

  }

  dWindDirVoltage = ads.readADC_SingleEnded(ADC_IN_WINDIRECTION);
  if (dWindDirVoltage == 0xFFFF)
  {
    bWindDirectionAvail = false;
  }
  else
  {
    bWindDirectionAvail = true;

    if (((dWindDirVoltage * 0.1875e-3) + NORTH_POS_ANGLE ) < WIND_ANGLE_15)
    {
      dWindDirVoltage = (dWindDirVoltage * 0.1875e-3) + NORTH_POS_ANGLE;
    }
    else
    {
      dWindDirVoltage = (dWindDirVoltage * 0.1875e-3) + NORTH_POS_ANGLE - WIND_ANGLE_15;
    }

    if ((WIND_ANGLE_0 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_1))
    {
      sWindDirection = WIND_DIR_0;
      dWindAngle = 0;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_1 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_2))
    {
      sWindDirection = WIND_DIR_1;
      dWindAngle = 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_2 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_3))
    {
      sWindDirection = WIND_DIR_2;
      dWindAngle = 2 * 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_3 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_4))
    {
      sWindDirection = WIND_DIR_3;
      dWindAngle = 3 * 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_4 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_5))
    {
      sWindDirection = WIND_DIR_4;
      dWindAngle = 4 * 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_5 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_6))
    {
      sWindDirection = WIND_DIR_5;
      dWindAngle = 5 * 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_6 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_7))
    {
      sWindDirection = WIND_DIR_6;
      dWindAngle = 6 * 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_7  < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_8))
    {
      sWindDirection = WIND_DIR_7;
      dWindAngle = 7 * 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_8 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_9))
    {
      sWindDirection = WIND_DIR_8;
      dWindAngle = 8 * 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_9 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_10))
    {
      sWindDirection = WIND_DIR_9;
      dWindAngle = 9 * 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_10 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_11))
    {
      sWindDirection = WIND_DIR_10;
      dWindAngle = 10 * 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_11 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_12))
    {
      sWindDirection = WIND_DIR_11;
      dWindAngle = 11 * 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_12 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_13))
    {
      sWindDirection = WIND_DIR_12;
      dWindAngle = 12 * 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_13 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_14))
    {
      sWindDirection = WIND_DIR_13;
      dWindAngle = 13 * 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if ((WIND_ANGLE_14 < dWindDirVoltage) && (dWindDirVoltage <= WIND_ANGLE_15))
    {
      sWindDirection = WIND_DIR_14;
      dWindAngle = 14 * 22.5;
      
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    }
    else if (WIND_ANGLE_15 < dWindDirVoltage)
    {
      sWindDirection = WIND_DIR_15;
      dWindAngle = 15 * 22.5;
      if (dWindAngle >= 360) dWindAngle = dWindAngle - 360;
    } else
    {
      dWindAngle = 999;
      sWindDirection = "ERROR";
    }
    Serial.println("dWindDirVoltage");
    Serial.println(dWindDirVoltage);
    Serial.println("dRainVoltage");
    Serial.println(dRainVoltage);
    Serial.println("bHeaterRain");
    Serial.println(bHeaterRain);

  }
  //digitalWrite(PIN_SPEEDWIND, LOW);
  dRainVoltage = ads.readADC_SingleEnded(ADC_IN_RAIN);
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
      iRainLevel = 0;
    }
  }

  if ((dWindSpeed > WIND_ALARM || iRainLevel > 1 ) && bWAlarm == false )
  {
    bWAlarm = true;
    iAlarmCount = 5;
  } else if (iAlarmCount == 0)
  {
    bWAlarm = false;
    bFirstWalarm = true;
  }
  iAlarmCount--;
  if (iAlarmCount < 0)
  {
    iAlarmCount = 0;
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
        MQTTclient.publish("WindAngle", String(dWindAngle).c_str());
      }
      else
      {
        MQTTclient.publish("WindDirection", "error");
        MQTTclient.publish("WindAngle", "999");
      }
      //Debug Only
      if (dWindDirVoltage  > 0)
      {
        MQTTclient.publish("WindDirVoltage", String(dWindDirVoltage).c_str());
      }
      else
      {
        MQTTclient.publish("WindDirVoltage", "Error");
      }
      break;
    case 6:
      if (bWindSpeedAvail)
      {
        MQTTclient.publish("WindSpeed", String(dWindSpeed).c_str());
      }
      else
      {
        MQTTclient.publish("WindSpeed", "error");
      }
      // Debug only
      if (dWindSpdVoltage  > 0)
      {
        MQTTclient.publish("WindSpdVoltage", String(dWindSpdVoltage).c_str());
      }
      else
      {
        MQTTclient.publish("WindSpdVoltage", "Error");
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
      //Debug only
      if (dRainVoltage  > 0)
      {
        MQTTclient.publish("RainVoltage", String(dRainVoltage).c_str());
      }
      else
      {
        MQTTclient.publish("RainVoltage", "Error");
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
      if (fPm2_5  > 0)
      {
        MQTTclient.publish("PM2_5", String(fPm2_5).c_str());
      }
      else
      {
        MQTTclient.publish("PM2_5", "Error");
      }
      break;
    case 12:
      if (fPm10  > 0)
      {
        MQTTclient.publish("PM10", String(fPm10).c_str());
      }
      else
      {
        MQTTclient.publish("PM10", "Error");
      }
      break;
    case 13:
      if (bHeaterRain == true)
      {
        MQTTclient.publish("HeaterRain", "ON");
      }
      else
      {
        MQTTclient.publish("HeaterRain", "OFF");
      }
      break;
  }
  iMQTTUpdateScreen++;
  if (iMQTTUpdateScreen > 13) iMQTTUpdateScreen = 1;
}

void  DisplayStatus() {
  //vTaskSuspend(timerDispl);
  // vTaskSuspend(timerMinute);
  // vTaskSuspend(timerSensors);
  //Remove screen update
  Wire.beginTransmission(ADDR_SSD1306);
  if (Wire.endTransmission(true) != 0)  // Device ACK'd
  {
    ESP.restart();
  }
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
  }
  else
  {
    display.drawString(0, 0,  "DISCONNECTED");
  }
  //Debug Minute
  if (sRebootMessage != "")
  {
    display.drawString(0, 11, sRebootMessage);
  }
  else
  {
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 11, String(iHourCountDebug));
  }
  display.setTextAlignment(TEXT_ALIGN_LEFT);
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
      if (bWindSpeedAvail)
      {
        display.drawString(31, 47, String(dWindSpeed));
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
      if (fPm2_5 > 0)
      {
        display.drawString(31, 47, String(fPm2_5));
      }
      else
      {
        display.drawString(31, 47, "ERROR");
      }
      if (fPm10  > 0)
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
  if (bWindSpeedAvail)
  {
    root["dWindSpeed"] = String(dWindSpeed);
  }
  else
  {
    root["dWindSpeed"] = String(-999);
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
  root["fPM2_5"] = String(fPm2_5);
  root["fPM10"] = String(fPm10);
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
