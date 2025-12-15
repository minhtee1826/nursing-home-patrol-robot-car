#include <Wire.h>
#include <QMC5883LCompass.h>
#include <math.h>

QMC5883LCompass compass;

void setup() {
  Serial.begin(9600); // Initialize Serial communication with the ESP32
  Wire.begin();       // Initialize I2C for the sensor
  compass.init();
  // ... (Optional: add calibration and declination settings here)
}

void loop() {
  int x, y, z;
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  // Send data in a consistent format (e.g., X,Y,Z\n)
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(z); // println adds a newline character to signal end of data

  delay(200);
}
