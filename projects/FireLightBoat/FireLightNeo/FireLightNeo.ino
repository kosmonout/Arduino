#include <Adafruit_NeoPixel.h>

#define NUM_PIXELS 4
#define DELAY_START 50
#define DELAY_STOP 400
#define BRIGHT_START 50
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

void setColor(uint32_t color) 
{
  pixels.setPixelColor(0, color);
  pixels.show();
}

void loop() 
{
  int iRed;
  int iGreen;
  iRed = random(BRIGHT_START, BRIGHT_STOP);
  iGreen = iRed * CANDLE_COLOR;
  setColor(pixels.Color(iRed, iGreen, 0));
  delay(random(DELAY_START, DELAY_STOP));
}
