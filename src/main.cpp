/*
 * Example for Bosch BME280_driver wrapper class
 * Author: Wolfgang Kracht
 * Created: 27.02.2021
 * 
 * NOTE: Running in ESP.deepSleep loop requires D0 (GPIO16) to be connected to RST of the ESP8266.
 */

#include "BME280wrapper.h"

#define CONSOLE Serial
//#define ESP_DEEP_SLEEP

BME280wrapper bme280;

// some BME280 parameters
struct {
  int      I2C_Addr = 0x76;   // i2c address of BME280 device
  uint32_t TimeSecs = 15;     // measuring interval time in [s]
  double   Altitude = 60.0;   // altitude of device in [m]
  int      PowerPin = -15;    // positive means D8 (GPIO15) used as 3.3V Vcc source for device
} BME;


void setup() {
  Serial.begin(115200);
  CONSOLE.println("\nbooting...");

  if (BME.PowerPin >= 0) {
    pinMode(BME.PowerPin, OUTPUT);
    digitalWrite(BME.PowerPin, HIGH);
  }
  
  if (!bme280.setup(BME.I2C_Addr)) {
    CONSOLE.println("BME280 setup failed!");
  }
}


void loop() {
  static uint32_t timestamp = BME.TimeSecs * 1000;
  uint32_t time_ms = millis();

  if (time_ms - timestamp >= BME.TimeSecs * 1000) {
    timestamp = time_ms;
  
    if (bme280.forceMeasure()) {
      CONSOLE.print("Time="); CONSOLE.println(time_ms);
      CONSOLE.print("Temp="); CONSOLE.println(bme280.getTemperature());
      CONSOLE.print("Humi="); CONSOLE.println(bme280.getHumidity());
      CONSOLE.print("Pres="); CONSOLE.println(bme280.getPressure());
      CONSOLE.print("PSea="); CONSOLE.println(bme280.getPressureAtSeaLevel(BME.Altitude));
      CONSOLE.print("DewP="); CONSOLE.println(bme280.getDewPoint());
      CONSOLE.println();
    }
    else {
      CONSOLE.println("BME280 measurement failed!");
    }
   
#ifdef ESP_DEEP_SLEEP
    CONSOLE.print("execution time: "); CONSOLE.print(micros());
    CONSOLE.println("us\ngoing to deep sleep...");
    if (BME.PowerPin >= 0) digitalWrite(BME.PowerPin, LOW);
    ESP.deepSleep(BME.TimeSecs * 1000000 - micros());
#endif
  }
}
