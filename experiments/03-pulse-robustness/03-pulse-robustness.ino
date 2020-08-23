#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>

#define SENSOR_PIN 0
#define LED_PIN 13

PulseSensorPlayground pulseSensor;

void setup() {
  pinMode(LED_PIN,OUTPUT);
  Serial.begin(9600);

  pulseSensor.analogInput(SENSOR_PIN);

  // begin and blink if failure
  if (!pulseSensor.begin()) {
    for(;;) {
      // Flash the led to show things didn't work.
      digitalWrite(LED_PIN, LOW);
      delay(50);
      digitalWrite(LED_PIN, HIGH);
      delay(50);
    }
  }
  
}

/*
 * We keep track of the maximum and minimum sensor values for 
 * 50 samples and set the threshold to 75% of the way to the
 * max from the min. So if the max is 1100 and the min is 100
 * we set the threshold to 850.
 */
#define SENSOR_RANGE_TIME_TO_LIVE 50
int _pulse_sensor_threshold = 0;
int _pulse_sensor_last_value = 0;
int _pulse_sensor_min_value = 1024; // sensor is 10-bit
int _pulse_sensor_max_value = 0;
int _pulse_sensor_min_ttl = 0;
int _pulse_sensor_max_ttl = 0;

void robust_threshold_update(void){
  int _pulse_sensor_range_change = 0;
  _pulse_sensor_last_value = pulseSensor.getLatestSample();

  _pulse_sensor_min_ttl -= 1;
  _pulse_sensor_max_ttl -= 1;

  if( _pulse_sensor_last_value < _pulse_sensor_min_value || _pulse_sensor_min_ttl <= 0 ){
    _pulse_sensor_min_value = _pulse_sensor_last_value;
    _pulse_sensor_min_ttl = SENSOR_RANGE_TIME_TO_LIVE;
    _pulse_sensor_range_change = 1;
  }
  if( _pulse_sensor_last_value > _pulse_sensor_max_value || _pulse_sensor_max_ttl <= 0 ){
    _pulse_sensor_max_value = _pulse_sensor_last_value;
    _pulse_sensor_max_ttl = SENSOR_RANGE_TIME_TO_LIVE;
    _pulse_sensor_range_change = 1;
  }

  if( _pulse_sensor_range_change ){
    _pulse_sensor_threshold = map( 75, 0, 100, _pulse_sensor_min_value, _pulse_sensor_max_value );
    pulse_sensor_update_threshold( _pulse_sensor_threshold );
  }
}
void pulse_sensor_update_threshold( int threshold ){
  pulseSensor.setThreshold( threshold );
}

void loop() {

  robust_threshold_update();

  // pulseSensor.setThreshold(THRESHOLD);

  Serial.print(_pulse_sensor_last_value, DEC);
  Serial.print(",");
  Serial.print(_pulse_sensor_min_value, DEC);
  Serial.print(",");
  Serial.print(_pulse_sensor_max_value, DEC);
  Serial.print(",");
  Serial.print(_pulse_sensor_threshold, DEC);
  Serial.println();

  delay(10);
}
