# ESP8266-BME280_driver_wrapper

There are several approaches for a BME280 interface class, this is one of them. I had issues with high noise.
Imho, triggering the ESP8266 watchdog reset when the I2C setup of the BME280 device fails is a bad design, but copied often.

The data sheet of the BME280 strongly recommends to use the original driver from Bosch. Therfore I gave it a try.
Refer to: https://github.com/BoschSensortec/BME280_driver The original source code is somewhat old fashioned c language, but VS-Code/platform.io compiles it with no issues.

This BME280wrapper class is just a c++ wrapper class for the original Bosch source located in the lib folder.
The structure of this repository complies to a pattform.io project.

Using an Arduino IDE you may need to move the source files of the lib folder to the main folder. I'm not using the Arduino IDE anymore. The VS-Code/platform.io IDE is much more confortable and I recommend to use it instead.

To change the oversampling and filter coefficients of the BME280 device, please edit the parameters in the BME280wrapper::setup function directly. My approach was to achieve the higest precision available against lowest power consumption possible.

To minimize the power consumption when battery powered you may supply the BME280 by a switchable GPIO output of the ESP8266.
This means a hard reset for the BME280 in every deep sleep loop and may result in slightly higher noise.

If you wannt to run the ESP8266 in deep sleep loop just uncomment the line "#define ESP_DEEP_SLEEP" in main.cpp.
Please note that a functional deep sleep loop requires D8 (GPIO16) to be connectet to RST pin of the ESP8266.
