#define touch_pin_L 12  // connected to mid pin of touch sensor
#define touch_pin_R 13  // connected to mid pin of touch sensor
#define touch_pin_M A0  // connected to mid pin of touch sensor

void setup(){
    pinMode(touch_pin_L, INPUT);
    pinMode(touch_pin_R, INPUT);
    pinMode(touch_pin_M, INPUT);
    Serial.begin(9600);
}

void loop(){
    if (digitalRead(touch_pin_L) == HIGH){
        Serial.println("left touched");
        delay(200);
    }
    if (digitalRead(touch_pin_R) == HIGH){
        Serial.println("right touched");
        delay(200);
    }
    if (digitalRead(touch_pin_M) == HIGH){
        Serial.println("middle touched");
        delay(200);
    }
}
