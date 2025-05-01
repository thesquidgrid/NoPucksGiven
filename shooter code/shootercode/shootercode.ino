// insert correct pin numbers

#define IN1 21
#define IN2 20
#define PWMA 37


void setup() {
  // tells teensy to output to these pins and not read from them
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

}

void loop() {
  // Slowly ramp up PWM from 0 to 800 (adjust as needed)
  analogWrite(PWMA, 1000);
  delay(300);
  analogWrite(PWMA, 0);

  delay(3000);  // Run at full speed for 2 seconds

}


void shoot() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWMA, 1000);  // max power

  // Stop motor 
  
}

void turnOff(){
  analogWrite(PWMA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
