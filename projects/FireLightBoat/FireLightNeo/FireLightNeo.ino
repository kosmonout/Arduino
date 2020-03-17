#include <Adafruit_NeoPixel.h>

#define NUM_PIXELS 4
#define DELAY_START 50
#define DELAY_STOP 400
#define COLOR_START 50
#define COLOR_STOP 80

Adafruit_NeoPixel pixels(NUM_PIXELS, 2, NEO_GRB | NEO_KHZ800);

void setup() {
  pixels.begin();
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, 0);
    pixels.show();
  }
}

void setColor(uint32_t color) {
     pixels.setPixelColor(0, color);
     pixels.show();
}

void loop() {
  int red = random(COLOR_START, COLOR_STOP);
  int green = red*0.67;
  setColor(pixels.Color(red, green, 0));
  delay(random(DELAY_START, DELAY_STOP));
}
