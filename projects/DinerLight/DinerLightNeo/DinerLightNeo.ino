#include <Adafruit_NeoPixel.h>

#define NUM_PIXELS 22
#define START_RANDOM_PIXEL 0
#define STOP_RANDOM_PIXEL 1
#define DELAY_START 50
#define DELAY_LOOP_STOP 30000
#define DELAY_STOP 200
#define BRIGHT_START 40
#define BRIGHT_STOP 80
#define CANDLE_COLOR 1
#define GPIOPIN 0

Adafruit_NeoPixel pixels(NUM_PIXELS, GPIOPIN, NEO_GRB | NEO_KHZ800);
int iRed;
int iGreen;
bool bLedSet[NUM_PIXELS];


void setup()
{
  Serial.begin(115200);
  pixels.begin();
  for (int i = 0; i < NUM_PIXELS; i++)
  {
    pixels.setPixelColor(i, 0);
    pixels.show();
  }
  delay (1000);
}

void loop()
{
  //Walk through delay loop
  for (int iCount = 0; iCount < DELAY_LOOP_STOP; iCount++)
  {
    for (int iLedNr = 0; iLedNr <= 12; iLedNr++)
    {
      Sparkle(iLedNr, iCount, iLedNr * 250 , 100);
    }
    //Redstart,Redstop,GreenStart,GreenStop,BlueStart,BlueStop
    //LedNumber,TimerCount,TimerStart,TimerDuration

    //Purple
    for (int iLedNr = 0; iLedNr <= 6; iLedNr++)
    {
      //Redstart,Redstop,GreenStart,GreenStop,BlueStart,BlueStop
      //LedNumber,TimerCount,TimerStart,TimerDuration
      LedFade(0, 105, 0, 20, 0, 100,
              iLedNr, iCount, 4000 + (iLedNr * 1000), 500);
    }
    for (int iLedNr = 7; iLedNr <= 12; iLedNr++)
    {
      LedFade(0, 105, 0, 20, 0, 100,
              iLedNr, iCount, 16000 - (iLedNr * 1000), 500);
    }

    //Green
    for (int iLedNr = 0; iLedNr <= 6; iLedNr++)
    {
      LedFade(105, 51, 20, 102, 100, 0,
              iLedNr, iCount, 17000 - (iLedNr * 1000), 500);
    }
    for (int iLedNr = 7; iLedNr <= 12; iLedNr++)
    {
      LedFade(105, 51, 20, 102, 100, 0,
              iLedNr, iCount, 5000 + (iLedNr * 1000), 500);
    }

     //Green
    for (int iLedNr = 0; iLedNr <= 12; iLedNr=iLedNr+3)
    {
      LedFade(0, 0, 0, 0, 0, 255,
              iLedNr, iCount, 20000 + (iLedNr * 500), 500);
              
    }
     for (int iLedNr = 1; iLedNr <= 12; iLedNr=iLedNr+3)
    {
      LedFade(0, 0, 0, 255, 0, 0,
              iLedNr, iCount, 20000 + (iLedNr * 500), 500);
              
    }
     for (int iLedNr = 2; iLedNr <= 12; iLedNr=iLedNr+3)
    {
      LedFade(0, 255, 0, 0, 0, 0,
              iLedNr, iCount, 20000 + (iLedNr * 500), 500);
          
    }
    
    //In huis
    pixels.setPixelColor(13, pixels.Color(255, 255, 255));

    //Redstart,Redstop,GreenStart,GreenStop,BlueStart,BlueStop
    //LedNumber,TimerCount,TimerStart,TimerDuration
    LedFade(0, 255, 0, 0, 0, 0,
            14, iCount, 4000 , 500);

    if (iCount % 500 == 0)
    {
      pixels.setPixelColor(15, pixels.Color(255, 0, 0));
      iRed = random(BRIGHT_START, BRIGHT_STOP);
      iGreen = iRed * CANDLE_COLOR;
      pixels.setPixelColor(16, pixels.Color(255, 255, 255));
      iRed = random(BRIGHT_START, BRIGHT_STOP);
      iGreen = iRed * CANDLE_COLOR;
      pixels.setPixelColor(17, pixels.Color(255, 255, 255));
      pixels.setPixelColor(18, pixels.Color(0, 255, 0));
      iRed = random(BRIGHT_START, BRIGHT_STOP);
      iGreen = iRed * CANDLE_COLOR;
      pixels.setPixelColor(19, pixels.Color(iRed, iGreen, 0));
      iRed = random(BRIGHT_START, BRIGHT_STOP);
      iGreen = iRed * CANDLE_COLOR;
      pixels.setPixelColor(20, pixels.Color(iRed, iGreen, 0));
      //      if (bLedSet[21] == false)
      //      {
      //        pixels.setPixelColor(21, pixels.Color(255, 0, 0));
      //        bLedSet[21] = true;
      //      }
      //      else
      //      {
      //        pixels.setPixelColor(21, pixels.Color(0, 255, 0));
      //        bLedSet[21] = false;
      //      }
      //Redstart,Redstop,GreenStart,GreenStop,BlueStart,BlueStop
      //LedNumber,TimerCount,TimerStart,TimerDuration
    }
    if (bLedSet[21] == true)
    {
      LedFade(255, 255, 0, 0, 0, 0,
              21, iCount, (iCount % 500) * 500 , 500);
              bLedSet[21] = false;
    }
    else
    {
      LedFade(0, 0, 255, 255, 0, 0,
              21, iCount, (iCount % 500) * 500 , 500);
              bLedSet[21] = true;
    }
    pixels.show();
    delay(1); //ms
  }
}
void  LedFade(int iRedstart, int iRedstop, int iGrnStrt, int iGrnStop, int iBlueStart, int iBlueStop, int iNeoNum, int iCurrentTime, int iTimeStart, int iTimeDuration)
{
  if (iCurrentTime >= iTimeStart && iCurrentTime <= (iTimeStart + iTimeDuration))
  {
    //   Serial.println(iCurrentTime);
    if (iTimeDuration != 0)
    {
      double dRedStep = double(double(abs(iRedstop - iRedstart)) / iTimeDuration);
      double dGreenStep = double(double(abs(iGrnStop - iGrnStrt)) / iTimeDuration);
      //Serial.println(dGreenStep);
      double dBlueStep = double(double(abs(iBlueStop - iBlueStart)) / iTimeDuration);
      //Serial.println(dBlueStep);
      int iRed;
      int iGreen;
      int iBlue;
      if (iRedstop >= iRedstart)
      {
        iRed = iRedstart + ((iCurrentTime - iTimeStart) * dRedStep);
        //    Serial.println(iRed);
        if (iRed >= iRedstop) {
          iRed = iRedstop;
        }
      }
      else
      {
        iRed = iRedstart - ((iCurrentTime - iTimeStart) * dRedStep);
        if (iRed <= iRedstop) {
          iRed = iRedstop;
        }
      }
      if (iGrnStop >= iGrnStrt)
      {
        iGreen = iGrnStrt + ((iCurrentTime - iTimeStart) * dGreenStep);
        if (iGreen >= iGrnStop) {
          iGreen = iGrnStop;
        }
      }
      else
      {
        iGreen = iGrnStrt - ((iCurrentTime - iTimeStart) * dGreenStep);
        if (iGreen <= iGrnStop) {
          iGreen = iGrnStop;
        }
      }
      if (iBlueStop >= iBlueStart)
      {
        iBlue = iBlueStart + ((iCurrentTime - iTimeStart) * dBlueStep);
        if (iBlue >= iBlueStop) {
          iBlue = iBlueStop;
        }
      }
      else
      {
        iBlue = iBlueStart - ((iCurrentTime - iTimeStart) * dBlueStep);
        if (iBlue <= iBlueStop) {
          iBlue = iBlueStop;
        }
      }
      Serial.println(iRed);
      Serial.println(iGreen);
      Serial.println(iBlue);
      pixels.setPixelColor(iNeoNum, pixels.Color(iRed, iGreen, iBlue));
    }
    else
    {
      pixels.setPixelColor(iNeoNum, pixels.Color(iRedstop, iGrnStop, iBlueStop));
    }
  }
}
void  Sparkle(int iNeoNum, int iCount, int iTimeStart, int iTimeDuration)
{
  //Redstart,Redstop,GreenStart,GreenStop,BlueStart,BlueStop
  LedFade(0, 255, 0, 255, 0, 255,
          //LedNumber,TimerCount,TimerStart,TimerDuration
          iNeoNum, iCount, iTimeStart + iTimeDuration / 6, iTimeDuration / 6);
  LedFade(0, 0, 0, 0, 0, 255,
          iNeoNum, iCount, iTimeStart + ((2 * iTimeDuration) / 6), iTimeDuration / 6);
  LedFade(0, 0, 0, 0, 0, 0,
          iNeoNum, iCount, iTimeStart + ((3 * iTimeDuration) / 6), iTimeDuration / 6);
  LedFade(0, 255, 0, 255, 0, 255,
          iNeoNum, iCount, iTimeStart + ((4 * iTimeDuration) / 6), iTimeDuration / 6);
  LedFade(0, 0, 0, 0, 0, 255,
          iNeoNum, iCount, iTimeStart + ((5 * iTimeDuration) / 6), iTimeDuration / 6);
  LedFade(0, 0, 0, 0, 0, 0,
          iNeoNum, iCount, iTimeStart + ((6 * iTimeDuration) / 6), iTimeDuration / 6);
}
