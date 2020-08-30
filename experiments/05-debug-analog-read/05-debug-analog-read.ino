
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#ifndef PSTR
 #define PSTR // Make Arduino Due happy
#endif

const int PULSE_INPUT = A0;

#define NOEPIX_PIN     6
#define NOEPIX_COUNT  60
Adafruit_NeoPixel strip(NOEPIX_COUNT, NOEPIX_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.setBrightness(10);
  strip.show();
}

void drawBar( const Adafruit_NeoPixel* strip, const uint8_t startIndex, const uint8_t endIndex, const byte red, const byte green, const byte blue, const uint16_t value, const uint16_t valueMax ){
  uint8_t barLength = endIndex - startIndex;
  uint8_t amountPerPix = valueMax / barLength;
  uint8_t fullyOnCount = value / amountPerPix;
  uint8_t remainder = value - fullyOnCount*amountPerPix;
  for( int i=startIndex; i<startIndex+fullyOnCount; i+=1 ){
    strip->setPixelColor( i, strip->Color(red, green, blue) );
  }
  strip->setPixelColor(
    startIndex+fullyOnCount,
    strip->Color(
      map( remainder, 0, amountPerPix, 0, red ),
      map( remainder, 0, amountPerPix, 0, green ),
      map( remainder, 0, amountPerPix, 0, blue )
    )
  );
}

byte r = 64;
byte g = 0;
byte b = 0;
byte neopixel_bar_length = 8;
int pulse_sensor_max = 1024;

void loop() {
  delay(20);
  int value = analogRead(PULSE_INPUT);
  strip.clear();
  drawBar( &strip, 0, neopixel_bar_length, r, g, b, value, pulse_sensor_max );
  strip.show();
}
