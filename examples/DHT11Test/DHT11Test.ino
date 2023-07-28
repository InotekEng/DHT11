/*
 * File: DHT11Test.ino
 * Project: DHT11Test
 * Author: Rabiaa Jebali,  rabiajebali5@gmail.com
 * -----
 * Modified By: Anis Messaoud
 * -----
 * 
 * Copyright (c) 2023 Inotek Engineering SARL
 * 
 * THIS SOFTWARE IS PROVIDED BY Inotek Engineering SARL "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Inotek Engineering SARL OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <Arduino.h>
#include "DHT11.h"

#define DHT_PIN D1 // Replace with the actual board pin number where the DHT11 data pin is connected

// Create a DHT11 object with the data pin connected to D2 (replace D2 with the actual pin number)
DHT11 dht11(DHT_PIN);

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
