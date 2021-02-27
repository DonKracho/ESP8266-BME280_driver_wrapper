#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "bme280.h"

class BME280wrapper
{
public:
  BME280wrapper() = default;
  virtual ~BME280wrapper() = default;

  bool   setup(int i2c_addr);
  bool   forceMeasure();
  bool   startMeasure();
  bool   readSensorData();
  double getTemperature() { return comp_data.temperature; };
  double getHumidity() { return comp_data.humidity; };
  double getPressure() { return comp_data.pressure / 100.0; };
  double getPressureAtSeaLevel(double altitude = 0.0) { return getPressureAtSeaLevel(getPressure(), getTemperature(), altitude); }
  double getPressureAtSeaLevel(double pressure, double temperature, double altitude = 0.0);
  double getDewPoint() { return getDewPoint(getTemperature(), getHumidity()); };
  double getDewPoint(double temperature, double humidity);

private:
  bool   begin(int i2c_addr);

  uint8_t dev_addr;
  uint32_t measure_delay;
  struct bme280_dev dev;
  struct bme280_data comp_data;
};
