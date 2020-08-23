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

/**********************************************************************
 * Robust Adjustable Threshold for Heatbeat Detection                 *
 **********************************************************************
 * 
 * We keep track of the maximum and minimum sensor values for 
 * `robust_threshold_time_to_live` samples and set the threshold
 * to `robost_threshold_percent` (default: 75%) of the way to the
 * max from the min. So if the max is 1100 and the min is 100
 * we set the threshold to 850.
 * Adjusting the threshold provides more accurate beat detection
 * than using a static threshold. However, there are cases were the
 * max/min are just below/above their previous values so they
 * "timeout" and reset to a more average value which is easily
 * surpased by the sensor. So instead of adjusting the max/min
 * when there is a strictly a new max/min we allow some deviation
 * and re-set the max/min if the new max is "close enough" to the
 * old (specifically if it's within robust_threshold_allowed_deviation
 * of the old max/min).
 * The other variables are for keeping track of data and should
 * not be adjusted.
 */
int robust_threshold_time_to_live = 50;
int robost_threshold_percent = 75;
int robust_threshold_allowed_deviation = 20;
int robust_threshold_has_finger_min_range = 50;
int _robust_threshold_value = 0;
int _robust_threshold_sensor_value = 0;
int _robust_threshold_min_sensor_value = 1024; // sensor is 10-bit
int _robust_threshold_max_sensor_value = 0;
int _robust_threshold_min_ttl = 0;
int _robust_threshold_max_ttl = 0;
int _robust_threshold_has_finger = 0;

void robust_threshold_update(void){
  int range_change = 0;
  _robust_threshold_sensor_value = pulseSensor.getLatestSample();

  _robust_threshold_min_ttl -= 1;
  _robust_threshold_max_ttl -= 1;

  if( _robust_threshold_sensor_value <= _robust_threshold_min_sensor_value+robust_threshold_allowed_deviation || _robust_threshold_min_ttl <= 0 ){
    _robust_threshold_min_sensor_value = _robust_threshold_sensor_value;
    _robust_threshold_min_ttl = robust_threshold_time_to_live;
    range_change = 1;
  }
  if( _robust_threshold_sensor_value >= _robust_threshold_max_sensor_value-robust_threshold_allowed_deviation || _robust_threshold_max_ttl <= 0 ){
    _robust_threshold_max_sensor_value = _robust_threshold_sensor_value;
    _robust_threshold_max_ttl = robust_threshold_time_to_live;
    range_change = 1;
  }

  if( range_change ){
    _robust_threshold_value = map( robost_threshold_percent, 0, 100, _robust_threshold_min_sensor_value, _robust_threshold_max_sensor_value );
    pulseSensor.setThreshold( _robust_threshold_value );
    
    int min_max_range = _robust_threshold_max_sensor_value - _robust_threshold_min_sensor_value;
    if( min_max_range > robust_threshold_has_finger_min_range ){
      _robust_threshold_has_finger = 1;
    }else{
      _robust_threshold_has_finger = 0;
    }
  }
}
int robust_threshold_has_finger( void ){
  return _robust_threshold_has_finger;
}

int debug_beat_marker = 0;
int debug_has_finger_marker = 0;
void loop() {

  robust_threshold_update();
  
  if( pulseSensor.sawStartOfBeat() ){
    debug_beat_marker = 1024;
  }
  if( robust_threshold_has_finger() ){
    debug_has_finger_marker = 1024;
  }else{
    debug_has_finger_marker = 0;
  }

  Serial.print(_robust_threshold_sensor_value, DEC);
  Serial.print(",");
  Serial.print(_robust_threshold_min_sensor_value, DEC);
  Serial.print(",");
  Serial.print(_robust_threshold_max_sensor_value, DEC);
  Serial.print(",");
  Serial.print(_robust_threshold_value, DEC);
  Serial.print(",");
  Serial.print(debug_beat_marker, DEC);
  Serial.print(",");
  Serial.print(debug_has_finger_marker, DEC);
  Serial.println();

  debug_beat_marker = 0;

  delay(10);
}
