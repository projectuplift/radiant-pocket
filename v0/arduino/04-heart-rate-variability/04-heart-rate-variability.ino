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

int lastIbi = 0;
int nextIbi = 0;
int sampleCount = 0;
int samplesOverThreshold = 0;
int deltaIbiThreshold = 50;
int samplesBeforeOutput = 10;
int hrvSmoothMin = 0;
int hrvSmoothMeasure = 5;
int hrvSmoothMax = 10;

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
    hrvOnBeat();
  }

  hrvUpdate();
  
  strip.show();
}
