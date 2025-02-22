#define NUM_SENSORS 7 // Change this value if we need to delete one sensor

// Sensor values array
int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6}; // A0 - A6 refer to pins 14 - 19 on the teensy

void setup() {
  Serial.begin(9600); // Serial Monitor

  // Set sensor pins as INPUT
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.println("Sensor Setup Complete!");
}

void loop() {
  Serial.print("Sensor Values: ");
    
  for (int i = 0; i < NUM_SENSORS; i++) {
    int sensorValue = analogRead(sensorPins[i]); // Reads each sensors value and stores it as sensorValue
    Serial.printf("Sensor %d: %d", i, sensorValue);

    if (i < NUM_SENSORS - 1) {
        Serial.print(", ");
    }
  }
  Serial.println(); // Moves to next line for formatting
  delay(200); //200ms between loops(Highly Subject To)
}
