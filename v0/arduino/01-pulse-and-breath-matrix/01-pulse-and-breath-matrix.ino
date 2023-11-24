/*
   Code to detect pulses from the PulseSensor,
   using an interrupt service routine.

   Here is a link to the tutorial\
   https://pulsesensor.com/pages/getting-advanced

   Copyright World Famous Electronics LLC - see LICENSE
   Contributors:
     Joel Murphy, https://pulsesensor.com
     Yury Gitman, https://pulsesensor.com
     Bradford Needham, @bneedhamia, https://bluepapertech.com

   Licensed under the MIT License, a copy of which
   should have been included with this software.

   This software is not intended for medical use.
*/

/*
   Every Sketch that uses the PulseSensor Playground must
   define USE_ARDUINO_INTERRUPTS before including PulseSensorPlayground.h.
   Here, #define USE_ARDUINO_INTERRUPTS true tells the library to use
   interrupts to automatically read and process PulseSensor data.

   See ProcessEverySample.ino for an example of not using interrupts.
*/
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>

#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#ifndef PSTR
 #define PSTR // Make Arduino Due happy
#endif

/*
   The format of our output.

   Set this to PROCESSING_VISUALIZER if you're going to run
    the Processing Visualizer Sketch.
    See https://github.com/WorldFamousElectronics/PulseSensor_Amped_Processing_Visualizer

   Set this to SERIAL_PLOTTER if you're going to run
    the Arduino IDE's Serial Plotter.
*/
const int OUTPUT_TYPE = SERIAL_PLOTTER;

/*
   Pinout:
     PULSE_INPUT = Analog Input. Connected to the pulse sensor
      purple (signal) wire.
     PULSE_BLINK = digital Output. Connected to an LED (and 220 ohm resistor)
      that will flash on each detected pulse.
     PULSE_FADE = digital Output. PWM pin onnected to an LED (and resistor)
      that will smoothly fade with each pulse.
      NOTE: PULSE_FADE must be a pin that supports PWM. Do not use
      pin 9 or 10, because those pins' PWM interferes with the sample timer.
*/
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

/*
   All the PulseSensor Playground functions.
*/
PulseSensorPlayground pulseSensor;
#define NEOPIXEL_PIN 6
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(5, 8, NEOPIXEL_PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB            + NEO_KHZ800);

const uint16_t colors[] = {
  matrix.Color(0, 0, 255),
  matrix.Color(30, 0, 230),
  matrix.Color(55, 0, 205),
  matrix.Color(80, 0, 180),
  matrix.Color(105, 0, 155),
  matrix.Color(130, 0, 130),
  matrix.Color(155, 0, 105),
  matrix.Color(180, 0, 80),
  matrix.Color(205, 0, 55),
  matrix.Color(230, 0, 30),
  matrix.Color(255, 0, 0)
};

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

  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(10);
  matrix.fillScreen(0);
  matrix.show();

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

// inhale, hold, exhale
#define BREATHING_STATE_COUNT 3
int breathingDurations[] = {200,350,400};
int breathingStateIndex = 0;
int breathingFrameIndex = 0;

#define COLOR( r, g, b ) (((r&0b11111000)<<8)|((g&0b11111100)<<5)|((b&0b11111000)>>3))

void drawBreath( const Adafruit_NeoMatrix* matrix, uint16_t color, uint8_t amount ){
  // adjust color
  //Serial.println( amount, DEC );
  matrix->drawPixel( 1, 1, matrix->Color(0,0,amount) );
}

void drawHeart( const Adafruit_NeoMatrix* matrix, uint16_t color, uint8_t amount ){
  matrix->drawPixel( 0, 0, matrix->Color(amount,0,0) );
}

void drawHRV( const Adafruit_NeoMatrix* matrix, uint16_t* colors, uint8_t amount ){
  matrix->drawPixel( 2, 2, colors[amount] );
}

void loop() {

  delay(20);

  //matrix.fillScreen(0);

  breathingFrameIndex += 1;
  //Serial.println( breathingFrameIndex, DEC);
  if( breathingFrameIndex > breathingDurations[breathingStateIndex] ){
    breathingFrameIndex = 0;
    breathingStateIndex += 1;
    if( breathingStateIndex >= BREATHING_STATE_COUNT ){
      breathingStateIndex = 0;
    }
  }

  //drawBreath( matrix, colors[0], breathingFrameIndex );

  if( breathingStateIndex % BREATHING_STATE_COUNT == 0 ){
    drawBreath( &matrix, colors[0], map( breathingFrameIndex, 0, breathingDurations[breathingStateIndex], 128, 255 ) );
  }
  if( breathingStateIndex % BREATHING_STATE_COUNT == 1 ){
    drawBreath( &matrix, colors[0], 255 );
  }
  if( breathingStateIndex % BREATHING_STATE_COUNT == 2 ){
    drawBreath( &matrix, colors[0], map( breathingFrameIndex, 0, breathingDurations[breathingStateIndex], 255, 128 ) );
  }

  if (pulseSensor.sawStartOfBeat()) {
    drawHeart( &matrix, colors[10], 64 );
    //drawHeart( &matrix, colors[10], map( pulseSensor.getPulseAmplitude(0), 0, 1023, 0, 255 ) );
    // TODO: Beat
    lastIbi = nextIbi;
    nextIbi = pulseSensor.getInterBeatIntervalMs();
    int delta = abs(nextIbi - lastIbi);
    if( delta > deltaIbiThreshold ){
      samplesOverThreshold += 1;
    }
    sampleCount += 1;
    if( sampleCount >= samplesBeforeOutput ){
      sampleCount = 0;
      drawHRV( &matrix, colors, samplesOverThreshold );
      // Serial.println( samplesOverThreshold, DEC );
      //matrix.fillScreen( colors[samplesOverThreshold] );
      //matrix.show();
      samplesOverThreshold = 0;
    }
  }else{
    drawHeart( &matrix, colors[10], 128 );
  }

  matrix.show();
}
