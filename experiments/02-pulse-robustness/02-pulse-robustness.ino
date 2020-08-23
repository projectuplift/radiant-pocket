#define SENSOR_PIN 0
#define LED_PIN 13

int sensor_heartbeat_threshold = 550;

void setup() {
  pinMode(LED_PIN,OUTPUT);
  Serial.begin(9600);
}

#define SAMPLES_PER_WINDOW 10
int _pulse_sensor_index = 0;
int _pulse_sensor_sum = 0;
int _pulse_sensor_last_value = 0;
int _pulse_sensor_average = 0;

void pulse_sensor_update(void) {
  _pulse_sensor_last_value = analogRead(SENSOR_PIN);
  _pulse_sensor_sum += _pulse_sensor_last_value;
  _pulse_sensor_index += 1;

  if( _pulse_sensor_index > SAMPLES_PER_WINDOW ){
    _pulse_sensor_average = _pulse_sensor_sum / SAMPLES_PER_WINDOW;
    _pulse_sensor_sum = 0;
    _pulse_sensor_index = 0;
  }
}
int pulse_sensor_get_value(void){
  return _pulse_sensor_last_value;
}
int pulse_sensor_get_average(void){
  return _pulse_sensor_average;
}

void loop() {

  pulse_sensor_update();

  // pulseSensor.setThreshold(THRESHOLD);

  Serial.print(pulse_sensor_get_value(), DEC);
  Serial.print(",");
  Serial.print(pulse_sensor_get_average(), DEC);
  Serial.println();

  delay(10);
}
