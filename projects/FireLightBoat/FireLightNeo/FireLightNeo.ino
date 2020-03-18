#include <Adafruit_NeoPixel.h>

#define NUM_PIXELS 4
#define DELAY_START 50
#define DELAY_STOP 400
#define BRIGHT_START 40
#define BRIGHT_STOP 80
#define CANDLE_COLOR 0.67
#define GPIOPIN 0


Adafruit_NeoPixel pixels(NUM_PIXELS, GPIOPIN, NEO_GRB | NEO_KHZ800);

void setup()
{
  pixels.begin();
  for (int i = 0; i < NUM_PIXELS; i++)
  {
    pixels.setPixelColor(i, 0);
    pixels.show();
  }
}

void loop()
{
  int iRed;
  int iGreen;
  for (int iNeoNum = 0; iNeoNum < NUM_PIXELS; iNeoNum++)
  {
    iRed = random(BRIGHT_START, BRIGHT_STOP);
    iGreen = iRed * CANDLE_COLOR;
    pixels.setPixelColor(iNeoNum, pixels.Color(iRed, iGreen, 0));
  }
  pixels.show();
  delay(random(DELAY_START, DELAY_STOP));
}
