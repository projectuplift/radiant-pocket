#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>

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

PulseSensorPlayground pulseSensor;

#define NOEPIX_PIN     6
#define NOEPIX_COUNT  60
Adafruit_NeoPixel strip(NOEPIX_COUNT, NOEPIX_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  /*
     Use 115200 baud because that's what the Processing Sketch expects to read,
     and because that speed provides about 11 bytes per millisecond.

     If we used a slower baud rate, we'd likely write bytes faster than
     they can be transmitted, which would mess up the timing
     of readSensor() calls, which would make the pulse measurement
     not work properly.
  */
  Serial.begin(115200);

  strip.begin();
  strip.setBrightness(10);
  strip.show();

  // Configure the PulseSensor manager.

  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.blinkOnPulse(PULSE_BLINK);

  // Now that everything is ready, start reading the PulseSensor signal.
  if (!pulseSensor.begin()) {
    /*
       PulseSensor initialization failed,
       likely because our particular Arduino platform interrupts
       aren't supported yet.

       If your Sketch hangs here, try PulseSensor_BPM_Alternative.ino,
       which doesn't use interrupts.
    */
    for(;;) {
      // Flash the led to show things didn't work.
      digitalWrite(PULSE_BLINK, LOW);
      delay(50);
      digitalWrite(PULSE_BLINK, HIGH);
      delay(50);
    }
  }
}

/*
 * Section on Breathing Animation
 */
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

/*
 * Section for visualizing a Heart Beat
 */
long heartbeatTimeOfLastBeat = 0;
long heartbeatTimeSinceLastBeat = 0;

#define HEARTBEAT_BEAT_DURATION_MS 200
#define HEARTBEAT_PENDING_DURATION_MS 3000

void drawHeartBeat( const Adafruit_NeoPixel* strip, const byte red, const byte green, const byte blue, uint16_t value ){
  strip->setPixelColor(
    1,
    map( value, 0, 255, 0, red ),
    map( value, 0, 255, 0, green ),
    map( value, 0, 255, 0, blue )
  );
}

void heartbeatOnBeat(){
  heartbeatTimeOfLastBeat = millis();
}

void heartbeatUpdate(){
  heartbeatTimeSinceLastBeat = millis() - heartbeatTimeOfLastBeat;
  
  if( heartbeatTimeSinceLastBeat < HEARTBEAT_BEAT_DURATION_MS ){
    drawHeartBeat( &strip, 255,0,0, map(heartbeatTimeSinceLastBeat, 0, HEARTBEAT_BEAT_DURATION_MS, 128, 32) );
    Serial.println( map(heartbeatTimeSinceLastBeat, 0, HEARTBEAT_BEAT_DURATION_MS, 128, 32), DEC );
  }else if( heartbeatTimeSinceLastBeat < HEARTBEAT_PENDING_DURATION_MS ){
    drawHeartBeat( &strip, 255,0,0, map(heartbeatTimeSinceLastBeat, 0, HEARTBEAT_PENDING_DURATION_MS, 32, 0) );
    Serial.println( map(heartbeatTimeSinceLastBeat, 0, HEARTBEAT_PENDING_DURATION_MS, 32, 0), DEC );
  }
}

/*
 * Section on Heart Rate Variability (HRV)
 */
int lastIbi = 0;
int nextIbi = 0;
int sampleCount = 0;
int samplesOverThreshold = 0;
int deltaIbiThreshold = 50;
int samplesBeforeOutput = 10;
int hrvSmoothMin = 0;
int hrvSmoothMeasure = 5;
int hrvSmoothMax = 10;

void hrvDraw( Adafruit_NeoPixel* strip, byte rLow, byte gLow, byte bLow, byte rHigh, byte gHigh, byte bHigh, byte blendLowHigh ){
  // blendLowHigh = 0 -> all low, 128 -> half/half, 255 -> all high
  byte r = map( blendLowHigh, 0, 255, rLow, rHigh );
  byte g = map( blendLowHigh, 0, 255, gLow, gHigh );
  byte b = map( blendLowHigh, 0, 255, bLow, bHigh );
  strip->setPixelColor( 0, r, g, b );
}

void hrvOnBeat(){
  lastIbi = nextIbi;
  nextIbi = pulseSensor.getInterBeatIntervalMs();
  int delta = abs(nextIbi - lastIbi);
  if( delta > deltaIbiThreshold ){
    samplesOverThreshold += 1;
  }
  sampleCount += 1;
  if( sampleCount >= samplesBeforeOutput ){
    if( samplesOverThreshold > hrvSmoothMeasure ){
      hrvSmoothMeasure += 1;
    }else{
      hrvSmoothMeasure -= 1;
    }
    if( hrvSmoothMeasure < hrvSmoothMin ){
      hrvSmoothMeasure = hrvSmoothMin;
    }
    if( hrvSmoothMeasure > hrvSmoothMax ){
      hrvSmoothMeasure = hrvSmoothMax;
    }
    hrvDraw( &strip, 0, 0, 255, 255, 0, 0, map(samplesOverThreshold,0,samplesBeforeOutput,0,255) );
    sampleCount = 0;
    samplesOverThreshold = 0;
  }
}

void hrvUpdate(){
  hrvDraw( &strip, 0, 0, 255, 255, 0, 0, map(hrvSmoothMeasure,hrvSmoothMin,hrvSmoothMax,0,255) );
}

void loop() {

  delay(20);

  strip.clear();

  if (pulseSensor.sawStartOfBeat()) {
    heartbeatOnBeat();
    hrvOnBeat();
  }

  breathingUpdate();
  heartbeatUpdate();
  hrvUpdate();
  
  strip.show();
}
