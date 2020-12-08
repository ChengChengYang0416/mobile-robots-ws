#define photo_sensor_pin A0

int val = 0;

void setup(){
  pinMode(photo_sensor_pin, INPUT);
  Serial.begin(9600);
}

void loop(){
  val = analogRead(photo_sensor_pin);
  Serial.println(val);
}
