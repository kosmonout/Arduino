#include <Adafruit_NeoPixel.h>

#define NUM_PIXELS 22
#define START_RANDOM_PIXEL 0
#define STOP_RANDOM_PIXEL 1
#define DELAY_START 50
#define DELAY_LOOP_STOP 20000
#define DELAY_STOP 200
#define BRIGHT_START 40
#define BRIGHT_STOP 80
#define CANDLE_COLOR 0.67
#define GPIOPIN 0

Adafruit_NeoPixel pixels(NUM_PIXELS, GPIOPIN, NEO_GRB | NEO_KHZ800);
int iRed;
int iGreen;
int iDelay[NUM_PIXELS];
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
  // setting up random delay per pixel
  for (int iNeoNum = 0; iNeoNum < NUM_PIXELS; iNeoNum++)
  {
    switch (iNeoNum)
    {
      case 0:
        iDelay[iNeoNum] = 500;
        bLedSet[iNeoNum] = false;
        break;
      case 1:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 2:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 3:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 4:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 5:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 6:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 7:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 8:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 9:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 10:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 11:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 12:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      // Start House Light**********************************************************
      case 13:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[NUM_PIXELS] = false;
        break;
      case 14:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 15:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 16:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 17:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 18:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 19:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 20:
        iDelay[iNeoNum] = random(DELAY_START, DELAY_STOP);
        bLedSet[iNeoNum] = false;
        break;
      case 21:
        iDelay[iNeoNum] = 500;
        bLedSet[iNeoNum] = false;
        break;
    }
  }
}

void loop()
{
  //Walk through delay loop
  for (int iCount = 0; iCount < DELAY_LOOP_STOP; iCount++)
  {
    Sparkle(0, iCount, 0, 100);
    Sparkle(1, iCount, 250, 100);
    Sparkle(2, iCount, 500, 100);
    Sparkle(3, iCount, 750, 100);
    Sparkle(4, iCount, 1000, 100);
    Sparkle(5, iCount, 1250, 100);
    Sparkle(6, iCount, 1500, 100);
    Sparkle(7, iCount, 1750, 100);
    Sparkle(8, iCount, 2000, 100);
    Sparkle(9, iCount, 2250, 100);
    Sparkle(10, iCount, 2500, 100);
    Sparkle(11, iCount, 2750, 100);
    Sparkle(12, iCount, 3000, 100);
    LedFade(0, 105, 0, 20, 0, 100,
            //LedNumber,TimerCount,TimerStart,TimerDuration
            0, iCount, 4000, 500);
    LedFade(0, 105, 0, 20, 0, 100,
            1, iCount, 5000, 500);
    LedFade(0, 105, 0, 20, 0, 100,
            2, iCount, 6000, 500);
    LedFade(0, 105, 0, 20, 0, 100,
            3, iCount, 7000, 500);
    LedFade(0, 105, 0, 20, 0, 100,
            4, iCount, 80000, 500);
    LedFade(0, 105, 0, 20, 0, 100,
            5, iCount, 9000, 500);
    LedFade(0, 105, 0, 20, 0, 100,
            6, iCount, 10000, 500);
    LedFade(0, 105, 0, 20, 0, 100,
            7, iCount, 9000, 500);
    LedFade(0, 105, 0, 20, 0, 100,
            8, iCount, 8000, 500);
    LedFade(0, 105, 0, 20, 0, 100,
            9, iCount, 7000, 500);
    LedFade(0, 105, 0, 20, 0, 100,
            10, iCount, 6000, 500);
    LedFade(0, 105, 0, 20, 0, 100,
            11, iCount, 5000, 500);
    LedFade(0, 105, 0, 20, 0, 100,
            12, iCount, 4000, 500);

    LedFade(105, 51, 20, 102, 100, 0,
            //LedNumber,TimerCount,TimerStart,TimerDuration
            0, iCount, 17000, 500);
    LedFade(105, 51, 20, 102, 100, 0,
            1, iCount, 16000, 500);
    LedFade(105, 51, 20, 102, 100, 0,
            2, iCount, 15000, 500);
    LedFade(105, 51, 20, 102, 100, 0,
            3, iCount, 14000, 500);
    LedFade(105, 51, 20, 102, 100, 0,
            4, iCount, 13000, 500);
    LedFade(105, 51, 20, 102, 100, 0,
            5, iCount, 12000, 500);
    LedFade(105, 51, 20, 102, 100, 0,
            6, iCount, 11000, 500);
    LedFade(105, 51, 20, 102, 100, 0,
            7, iCount, 12000, 500);
    LedFade(105, 51, 20, 102, 100, 0,
            8, iCount, 13000, 500);
    LedFade(105, 51, 20, 102, 100, 0,
            9, iCount, 14000, 500);
    LedFade(105, 51, 20, 102, 100, 0,
            10, iCount, 15000, 500);
    LedFade(105, 51, 20, 102, 100, 0,
            11, iCount, 16000, 500);
    LedFade(105, 51, 20, 102, 100, 0,
            12, iCount, 17000, 500);
    for (int iNeoNum = 0; iNeoNum < NUM_PIXELS; iNeoNum++)
    {
      switch (iNeoNum)
      {

        // Start House Light First Floor**********************************************************
        case 13:
          if (iCount % iDelay[iNeoNum] == 0)
          {
            iRed = random(BRIGHT_START, BRIGHT_STOP);
            iGreen = iRed * CANDLE_COLOR;
            pixels.setPixelColor(iNeoNum, pixels.Color(iRed, iGreen, 0));
          }
          break;
        case 14:
          if (iCount % iDelay[iNeoNum] == 0)
          {
            iRed = random(BRIGHT_START, BRIGHT_STOP);
            iGreen = iRed * CANDLE_COLOR;
            pixels.setPixelColor(iNeoNum, pixels.Color(iRed, iGreen, 0));
          }
          break;
        case 15:
          if (iCount % iDelay[iNeoNum] == 0)
          {
            iRed = random(BRIGHT_START, BRIGHT_STOP);
            iGreen = iRed * CANDLE_COLOR;
            pixels.setPixelColor(iNeoNum, pixels.Color(iRed, iGreen, 0));
          }
          break;
        // Start House Light Second Floor**********************************************************
        case 16:
          if (iCount % iDelay[iNeoNum] == 0)
          {
            iRed = random(BRIGHT_START, BRIGHT_STOP);
            iGreen = iRed * CANDLE_COLOR;
            pixels.setPixelColor(iNeoNum, pixels.Color(iRed, iGreen, 0));
          }
          break;
        case 17:
          if (iCount % iDelay[iNeoNum] == 0)
          {
            iRed = random(BRIGHT_START, BRIGHT_STOP);
            iGreen = iRed * CANDLE_COLOR;
            pixels.setPixelColor(iNeoNum, pixels.Color(iRed, iGreen, 0));
          }
          break;
        case 18:
          if (iCount % iDelay[iNeoNum] == 0)
          {
            iRed = random(BRIGHT_START, BRIGHT_STOP);
            iGreen = iRed * CANDLE_COLOR;
            pixels.setPixelColor(iNeoNum, pixels.Color(iRed, iGreen, 0));
          }
          break;
        case 19:
          if (iCount % iDelay[iNeoNum] == 0)
          {
            iRed = random(BRIGHT_START, BRIGHT_STOP);
            iGreen = iRed * CANDLE_COLOR;
            pixels.setPixelColor(iNeoNum, pixels.Color(iRed, iGreen, 0));
          }
          break;
        case 20:
          if (iCount % iDelay[iNeoNum] == 0)
          {
            iRed = random(BRIGHT_START, BRIGHT_STOP);
            iGreen = iRed * CANDLE_COLOR;
            pixels.setPixelColor(iNeoNum, pixels.Color(iRed, iGreen, 0));
          }
          break;
        case 21:
          if (iCount % iDelay[iNeoNum] == 0)
          {
            if (bLedSet[iNeoNum] == false)
            {
              pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
              bLedSet[iNeoNum] = true;
            }
            else
            {
              pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
              bLedSet[iNeoNum] = false;
            }
          }
          pixels.show();
          delay(1); //ms
      }
    }
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
