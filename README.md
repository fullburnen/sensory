Sensory
---

A simple Arduino sensor for Home Assistant that reports over MQTT. Supports Home Assistant's MQTT Discovery.

Hardware requirements:
* Adafruit Feather M0 WiFi with ATWINC1500
* Temperature sensor
  * Adafruit BMP280 via Stemma QT or I2C

Arduino libraries required:
* Adafruit SAMD
* WiFi101
* ArduinoMqttClient
* Adafruit BMP280

Sensor data supported:
* Temperature
