Sensory
---

A simple Arduino sensor for Home Assistant that reports over MQTT. Supports Home Assistant's MQTT Discovery.

Hardware requirements:
* Board:
  * Adafruit Feather M0 WiFi with ATWINC1500 (PID: 3010/3061/3044/2598)
* Sensor (pick one):
  * Adafruit BMP280 via Stemma QT / I2C (PID: 2651)
  * Adafruit SHT31-D (PID: 2857) or SHT30-D (PID: 4099)

Arduino libraries required:
* Board:
  * Adafruit SAMD
  * WiFi101
  * ArduinoMqttClient
  * ArduinoJson
  * RTCZero
* Sensor:
  * Adafruit BMP280
  * Adafruit SHT31

Sensor data supported:
* Temperature
* Voltage
* Humidity
