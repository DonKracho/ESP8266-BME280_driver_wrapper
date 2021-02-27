/*
 * Bosch BME280_driver wrapper class
 * Author: Wolfgang Kracht
 * Created: 27.02.2021
 */

#include "BME280wrapper.h"
#include <math.h>

void user_delay_us(uint32_t period, void *intf_ptr) {
  delayMicroseconds(period);
}

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

  /*
   * The parameter intf_ptr can be used as a variable to store the I2C address of the device
   */

  /*
   * Data on the bus should be like
   * |------------+---------------------|
   * | I2C action | Data                |
   * |------------+---------------------|
   * | Start      | -                   |
   * | Write      | (reg_addr)          |
   * | Stop       | -                   |
   * | Start      | -                   |
   * | Read       | (reg_data[0])       |
   * | Read       | (....)              |
   * | Read       | (reg_data[len - 1]) |
   * | Stop       | -                   |
   * |------------+---------------------|
   */

  Wire.beginTransmission(*(int*)intf_ptr);        // burst read
  Wire.write(reg_addr);                           // reg_addr is start address of the data
  rslt = Wire.endTransmission();
  if (rslt == 0) {
    Wire.requestFrom(*(int*)intf_ptr, len);       // request "length" bytes of data
    uint32_t byteCount;
    while ((byteCount = Wire.available()) == 0) delay(1);
    if (byteCount == len) {                       // correct answer
      for (int i = 0; i < (int) byteCount; i++) { // read bytes to array
        reg_data[i] = Wire.read();
      }
    }
  }
  return rslt;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

  /*
    * The parameter intf_ptr can be used as a variable to store the I2C address of the device
    */

  /*
    * Data on the bus should be like
    * |------------+---------------------|
    * | I2C action | Data                |
    * |------------+---------------------|
    * | Start      | -                   |
    * | Write      | (reg_addr)          |
    * | Write      | (reg_data[0])       |
    * | Write      | (....)              |
    * | Write      | (reg_data[len - 1]) |
    * | Stop       | -                   |
    * |------------+---------------------|
    */

  Wire.beginTransmission(*(int*)intf_ptr);
  Wire.write(reg_addr);
  for (int i = 0; i < (int)len; i++) { // read bytes to array
    Wire.write(reg_data[i]);
  }
  rslt = Wire.endTransmission();
  return rslt;
}

double BME280wrapper::getPressureAtSeaLevel(double pressure, double temperature, double altitude) {
  double alti = 0.0065 * altitude;
  return pressure * pow(1 - (alti / (temperature + alti + 273.15)), -5.257);
}

double BME280wrapper::getDewPoint(double temperature, double humidity) {
  // a and b are coefficients. For Sonntag90 constant set, a = 17.62°C and b = 243.12°C;
  double a = 17.62;
  double b = 243.12;
  double alpha = log(humidity / 100) + a * temperature / (temperature + b);
  return b * alpha / (a - alpha); 
}

bool BME280wrapper::readSensorData() {
  return bme280_get_sensor_data(BME280_ALL, &comp_data, &dev) == 0;
}

bool BME280wrapper::startMeasure() {
  uint8_t stat = 0;
  int8_t rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
  do {
    dev.delay_us(measure_delay, &dev.intf_ptr);   // wait for the measurement to complete
    rslt = bme280_get_regs(BME280_STATUS_REG_ADDR, &stat, 1, &dev);
  } while (rslt == 0 && (stat & 0x08) != 0);
  return rslt == 0;
}

bool BME280wrapper::forceMeasure() {
  uint8_t stat = 0;
  int8_t rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
  do {
    dev.delay_us(measure_delay, &dev.intf_ptr);   // wait for the measurement to complete
    rslt = bme280_get_regs(BME280_STATUS_REG_ADDR, &stat, 1, &dev);
  } while (rslt == 0 && (stat & 0x08) != 0);
  if (rslt == 0) {
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
  };
  return rslt == 0 && comp_data.pressure > 90000.0;
}

bool BME280wrapper::begin(int addr) {
  dev_addr = addr;

  dev.intf_ptr = &dev_addr;
  dev.intf = BME280_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_us = user_delay_us;

  dev.intf_rslt = bme280_init(&dev);
  return dev.intf_rslt == 0;
}

bool BME280wrapper::setup(int addr) {
  Wire.begin();
  bool ret = begin(addr);

  if (ret) {
    /* Recommended mode of operation: Indoor navigation */
    dev.settings.osr_h = BME280_OVERSAMPLING_16X;
    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev.settings.osr_t = BME280_OVERSAMPLING_16X;
    dev.settings.filter = BME280_FILTER_COEFF_16;

    uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    bme280_set_sensor_settings(settings_sel, &dev);
    // Calculate the minimum delay required between consecutive measurement based upon the sensor enabled  and the oversampling configuration.
    measure_delay = bme280_cal_meas_delay(&dev.settings);
  }
  return ret;
}
