// Line Sensor Pins
#define LEFT_SENSOR   9  // Left sensor pin
#define CENTER_SENSOR 10 // Center sensor pin
#define RIGHT_SENSOR  11 // Right sensor pin

// Pushbutton Pin
#define BUTTON_PIN  12 // Pushbutton pin

bool isRunning = false; // Used in our printing logic

void setup() {
    Serial.begin(9600); // Start serial communication

    // Set sensor pins as inputs
    pinMode(LEFT_SENSOR, INPUT);
    pinMode(CENTER_SENSOR, INPUT);
    pinMode(RIGHT_SENSOR, INPUT);

    // Set button pin as input with pull-up resistor
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    Serial.println("Press the button to start displaying sensor values...");
}

void loop() {
    // Wait for button press to start
    if (digitalRead(BUTTON_PIN) == LOW) {  
        isRunning = !isRunning; // Toggle the state
        delay(500); // Debounce delay
    }

    // If not running, stop printing
    if (!isRunning) {
        Serial.println("Waiting for button press...");
        delay(500);
        return; // Skip the rest of the loop
    }

    // Read sensor values
    int left = digitalRead(LEFT_SENSOR);
    int center = digitalRead(CENTER_SENSOR);
    int right = digitalRead(RIGHT_SENSOR);

    // Print sensor values
    Serial.print("Left: ");
    Serial.print(left);

    Serial.print(" | Center: "); 
    Serial.print(center);

    Serial.print(" | Right: ");
    Serial.println(right);

    delay(500); // Delay next reading
