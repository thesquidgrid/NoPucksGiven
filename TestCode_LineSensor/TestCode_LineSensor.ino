#include "Adafruit_VL53L0X.h"
#include <Wire.h>  // Include the Wire library for custom I2C pins

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }
 
  Wire.begin(); //<-- optional petrameters to change address    // Begin I2C communication on Wire1 with the custom SDA/SCL pins

  if (!lox.begin(VL53L0X_I2C_ADDR, 0, &Wire)) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  
  lox.setAddress(0x29);
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }

  delay(1000);
}
