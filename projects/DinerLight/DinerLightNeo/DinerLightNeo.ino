#include <Adafruit_NeoPixel.h>

#define NUM_PIXELS 22
#define START_RANDOM_PIXEL 0
#define STOP_RANDOM_PIXEL 1
#define DELAY_START 50
#define DELAY_LOOP_STOP 10000
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
    for (int iNeoNum = 0; iNeoNum < NUM_PIXELS; iNeoNum++)
    {
      switch (iNeoNum)
      {
        case 0:
          if (iCount == 250)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
            bLedSet[iNeoNum] = true;
          }
          else if (iCount == 500)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
          }
          else if (iCount == 750)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 255));
          }
          else if (iCount == 1000)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 0));
          }
          break;
        case 1:
          if (iCount == 1000)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
          }
          else if (iCount == 1250)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
          }
          else if (iCount == 1500)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 255));
          }
          else if (iCount == 1750)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 0));
          }
          break;
        case 2:
          if (iCount == 1750)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
          }
          else if (iCount == 2000)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
          }
          else if (iCount == 2250)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 255));
          }
          else if (iCount == 2500)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 0));
          }
          break;
        case 3:
          if (iCount == 2500)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
          }
          else if (iCount == 2750)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
          }
          else if (iCount == 3000)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 255));
          }
          else if (iCount == 3250)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 0));
          }
          break;
        case 4:
          if (iCount == 3250)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
          }
          else if (iCount == 3500)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
          }
          else if (iCount == 3750)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 255));
          }
          else if (iCount == 4000)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 0));
          }
          break;
        case 5:
          if (iCount == 4000)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
          }
          else if (iCount == 4250)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
          }
          else if (iCount == 4500)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 255));
          }
          else if (iCount == 4750)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 0));
          }
          break;
        case 6:
          if (iCount == 4750)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
          }
          else if (iCount == 5000)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
          }
          else if (iCount == 5250)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 255));
          }
          else if (iCount == 5500)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 0));
          }
          break;
        case 7:
          if (iCount == 5500)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
          }
          else if (iCount == 5750)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
          }
          else if (iCount == 6000)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 255));
          }
          else if (iCount == 6250)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 0));
          }
          break;
        case 8:
          if (iCount == 6250)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
          }
          else if (iCount == 6500)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
          }
          else if (iCount == 6750)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 255));
          }
          else if (iCount == 7000)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 0));
          }
          break;
        case 9:
          if (iCount == 7000)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
          }
          else if (iCount == 7250)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
          }
          else if (iCount == 7500)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 255));
          }
          else if (iCount == 7750)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 0));
          }
          break;
        case 10:
          if (iCount == 7750)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
          }
          else if (iCount == 8000)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
          }
          else if (iCount == 8250)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 255));
          }
          else if (iCount == 8500)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 0));
          }
          break;
        case 11:
          if (iCount == 8500)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
          }
          else if (iCount == 8750)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
          }
          else if (iCount == 9000)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 255));
          }
          else if (iCount == 9250)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 0));
          }
          break;
        case 12:
          if (iCount == 9250)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(255, 0, 0));
          }
          else if (iCount == 9500)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 255, 0));
          }
          else if (iCount == 9750)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 255));
          }
          else if (iCount == 9999)
          {
            pixels.setPixelColor(iNeoNum, pixels.Color(0, 0, 0));
          }
          break;
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
          break;
      }
    }
    pixels.show();
    delay(1); //ms
  }
}
