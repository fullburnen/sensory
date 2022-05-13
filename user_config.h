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
#define MQTT_TOPIC_CONFIG_TEMPERATURE "homeassistant/sensor/" SENSOR_NAME "_T/config"
#define MQTT_TOPIC_CONFIG_VOLTAGE "homeassistant/sensor/" SENSOR_NAME "_V/config"
#define MQTT_TOPIC_CONFIG_HUMIDITY "homeassistant/sensor/" SENSOR_NAME "_H/config"
#define MQTT_TOPIC_STATE "homeassistant/sensor/" SENSOR_NAME "/state"

//Sensor hardware
//#define USE_BMP280
#define USE_SHT31
//#define USE_ALT_ADDRESS

//Read sensors and publish to server every x seconds
#define REPORT_INTERVAL_S 300

#define USE_METRIC
