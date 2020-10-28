#define IN1 8                     // control pin for left motor
#define IN2 9                     // control pin for left motor
#define IN3 10                    // control pin for right motor
#define IN4 11                    // control pin for right motor
#define ENA 5                     // analog pin for motor PWM
#define ENB 6                     // analog pin for motor PWN

void setup(void) {
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);  
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);  
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 100);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 100);
}

