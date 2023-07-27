// Include necessary libraries and header file for DHT11
#include <Arduino.h>
#include "DHT11.h"

int D2 = 2; // Replace 2 with the actual board pin number where the DHT11 data pin is connected

// Create a DHT11 object with the data pin connected to D2 (replace D2 with the actual pin number)
DHT11 dht11(D2);

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);
  Serial.println("DHT11 Temperature and Humidity Sensor Reading");
}

void loop() {
  // Read data from the DHT11 sensor
  dht11_result result = dht11.read();
  
  if (result.result == DHT_OK) {
    // If data reading is successful, print temperature and humidity
    Serial.print("Temperature: ");
    Serial.print(result.T);
    Serial.print("°C");
    Serial.print("    Humidity: ");
    Serial.print(result.RH);
    Serial.println("%");
  } else {
    // If there is an error reading the sensor, print the error code
    Serial.print("Error reading DHT11 sensor. Error code: ");
    Serial.println(result.result);
  }

  // Wait for 2 seconds before reading again
  delay(2000);
}
