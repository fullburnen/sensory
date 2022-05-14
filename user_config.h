//SSID connection info
#define SECRET_SSID "ssid"
#define SECRET_PASS "password"

//Device name
#define SENSOR_NAME "sensor1"
#define SENSOR_NAME_NICE "Sensor 1"

//MQTT server information
#define MQTT_SERVER "localhost"
#define MQTT_PORT 1883
#define MQTT_USERNAME SENSOR_NAME
#define MQTT_PASSWORD SENSOR_NAME

//Sensor hardware
//#define USE_BMP280
#define USE_SHT31
//#define USE_ALT_ADDRESS

//Read sensors and publish to server every x seconds
#define REPORT_INTERVAL_S 300

#define USE_METRIC

//Which measurements to report. Unsupported measurements are disabled at compile
#define REPORT_VOLTAGE
#define VOLTAGE_PIN A7
#define REPORT_TEMPERATURE
#define REPORT_HUMIDITY
