#define SENSOR_PIN 0
#define LED_PIN 13

int sensor_value;
int sensor_heartbeat_threshold = 550;

void setup() {
  pinMode(LED_PIN,OUTPUT);
  Serial.begin(9600);
}

void loop() {

  sensor_value = analogRead(SENSOR_PIN);
  Serial.println(sensor_value);
  if(sensor_value > sensor_heartbeat_threshold ){
   digitalWrite(LED_PIN,HIGH);
  } else {
   digitalWrite(LED_PIN,LOW);
  }

  delay(10);
}
