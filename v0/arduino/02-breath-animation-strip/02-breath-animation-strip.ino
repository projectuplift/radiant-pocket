#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#ifndef PSTR
 #define PSTR // Make Arduino Due happy
#endif

const int PULSE_INPUT = A0;
const int PULSE_BLINK = 13;    // Pin 13 is the on-board LED
const int PULSE_FADE = 5;
const int THRESHOLD = 550;   // Adjust this number to avoid noise when idle

#define NOEPIX_PIN     6
#define NOEPIX_COUNT  60
Adafruit_NeoPixel strip(NOEPIX_COUNT, NOEPIX_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.setBrightness(10);
  strip.show();
}

// inhale, hold, exhale
#define BREATHING_STATE_COUNT 3
int breathingDurations[] = {200,350,400};
int breathingStateIndex = 0;
int breathingFrameIndex = 0;

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
void drawBarFull( const Adafruit_NeoPixel* strip, const uint8_t startIndex, const uint8_t endIndex, const byte red, const byte green, const byte blue, const uint16_t value, const uint16_t valueMax ){
  uint8_t barLength = endIndex - startIndex;
  uint8_t amountPerPix = valueMax / barLength;
  uint8_t fullyOnCount = value / amountPerPix;
  uint8_t remainder = value - fullyOnCount*amountPerPix;
  for( int i=startIndex; i<endIndex; i+=1 ){
    strip->setPixelColor( i, strip->Color(red, green, blue) );
  }
}
void drawBarReverse( const Adafruit_NeoPixel* strip, const uint8_t startIndex, const uint8_t endIndex, const byte red, const byte green, const byte blue, const uint16_t value, const uint16_t valueMax ){
  uint8_t barLength = endIndex - startIndex;
  uint8_t amountPerPix = valueMax / barLength;
  uint8_t fullyOnCount = (valueMax-value) / amountPerPix;
  uint8_t remainder = (valueMax-value) - fullyOnCount*amountPerPix;
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

void breathingUpdate(){
  breathingFrameIndex += 1;
  //Serial.println( breathingFrameIndex, DEC);
  if( breathingFrameIndex > breathingDurations[breathingStateIndex] ){
    breathingFrameIndex = 0;
    breathingStateIndex += 1;
    if( breathingStateIndex >= BREATHING_STATE_COUNT ){
      breathingStateIndex = 0;
    }
  }

  if( breathingStateIndex % BREATHING_STATE_COUNT == 0 ){
    //drawBar( &strip, 2, 8, 0, 0, 255, breathingFrameIndex,  breathingDurations[breathingStateIndex] );
    //drawBreath( &matrix, colors[0], map( breathingFrameIndex, 0, breathingDurations[breathingStateIndex], 128, 255 ) );
    drawBar( &strip, 2, 8, 0, 0, 255, breathingFrameIndex,  breathingDurations[breathingStateIndex] );
  }
  if( breathingStateIndex % BREATHING_STATE_COUNT == 1 ){
    //drawBreath( &matrix, colors[0], 255 );
    drawBarFull( &strip, 2, 8, 0, 0, 255, 1, 1 );
  }
  if( breathingStateIndex % BREATHING_STATE_COUNT == 2 ){
    //drawBreath( &matrix, colors[0], map( breathingFrameIndex, 0, breathingDurations[breathingStateIndex], 255, 128 ) );
    drawBarReverse( &strip, 2, 8, 0, 0, 255, breathingFrameIndex,  breathingDurations[breathingStateIndex] );
  }
}

void loop() {

  delay(20);

  strip.clear();
  breathingUpdate();
  strip.show();
}
