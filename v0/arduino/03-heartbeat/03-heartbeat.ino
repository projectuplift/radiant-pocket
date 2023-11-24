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

PulseSensorPlayground pulseSensor;

#define NOEPIX_PIN     6
#define NOEPIX_COUNT  60
Adafruit_NeoPixel strip(NOEPIX_COUNT, NOEPIX_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
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

void loop() {

  delay(20);

  strip.clear();

  if (pulseSensor.sawStartOfBeat()) {
    heartbeatOnBeat();
  }

  heartbeatUpdate();
  
  strip.show();
}
