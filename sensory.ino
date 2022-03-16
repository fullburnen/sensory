#include <ArduinoMqttClient.h>
#include <WiFi101.h>
#include <ArduinoJson.h>

#include "user_config.h"

#ifdef USE_BMP280
#include <Adafruit_BMP280.h>
#define REPORT_TEMPERATURE
#elif defined( USE_SHT31 )
#include <Adafruit_SHT31.h>
#define REPORT_TEMPERATURE
#endif

#define REPORT_VOLTAGE
#define VBAT_PIN A7

#define LED_STATUS_PIN 13

//=============================================================================
//Globals
//=============================================================================

//Wifi globals
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int wifi_status = WL_IDLE_STATUS;

//Sensor globals
#ifdef USE_BMP280
Adafruit_BMP280 sensor;
#elif defined( USE_SHT31 )
Adafruit_SHT31 sensor;
#endif

bool alt_address = false;
#ifdef USE_ALT_ADDRESS
alt_address = true;
#endif

//MQTT globals
char mqtt_server[] = MQTT_SERVER;
int mqtt_port = MQTT_PORT;
char mqtt_username[] = MQTT_USERNAME;
char mqtt_password[] = MQTT_PASSWORD;
#ifdef REPORT_TEMPERATURE
char mqtt_topic_config_temperature[] = MQTT_TOPIC_CONFIG_TEMPERATURE;
#endif
#ifdef REPORT_VOLTAGE
char mqtt_topic_config_voltage[] = MQTT_TOPIC_CONFIG_VOLTAGE;
#endif
char mqtt_topic_state[] = MQTT_TOPIC_STATE;
char sensor_name[] = SENSOR_NAME;
char sensor_name_nice[] = SENSOR_NAME_NICE;

WiFiClient wifi_client;
MqttClient mqtt_client( wifi_client );

//=============================================================================
//Main block
//=============================================================================

void setup() {
    setup_serial( false );
    Serial.println( "Booting" );

#ifdef USE_BMP280
    if ( !sensor.begin( alt_address ? BMP280_ADDRESS_ALT : BMP280_ADDRESS ) ) {
        Serial.println( "Could not find a valid BMP280 sensor" );
        delay( 1000 );
    }
    Serial.println( "Set up BMP280 sensor" );
    sensor.setSampling( Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500 );
#elif defined( USE_SHT31 )
    if ( !sensor.begin( alt_address ? 0x45 : SHT31_DEFAULT_ADDR ) ) {
        Serial.println( "Could not find a valid SHT31 sensor" );
        delay( 1000 );
    }
    Serial.println( "Set up SHT31 sensor" );
    sensor.heater( false );
#endif

    //For Adafruit ATWINC1500
    WiFi.setPins( 8, 7, 4, 2 );

    if ( WiFi.status() == WL_NO_SHIELD ) {
        while ( true ) {
            delay( 30000 );
            Serial.println( "No wifi detected" );
        }
    }

    //Set LED pin to indicate activity
    pinMode( LED_STATUS_PIN, OUTPUT );

    setup_wifi();
    setup_mqtt();
    send_config();

    Serial.println( "Boot done" );
}

void loop() {
#ifdef REPORT_TEMPERATURE
    float temperature = NAN;
#endif
#ifdef REPORT_VOLTAGE
    float voltage = NAN;
#endif
    StaticJsonDocument<512> json_doc;

    digitalWrite( LED_STATUS_PIN, HIGH );
    delay( 1000 );

    if ( !Serial ) {
        setup_serial( false );
    }

    wifi_status = WiFi.status();
    if ( wifi_status != WL_CONNECTED ) {
        Serial.print( "Connection lost" );
        mqtt_client.stop();
        WiFi.end();
        wifi_status = WiFi.status();
        setup_wifi();
    }
    if ( !mqtt_client.connected() ) {
        setup_mqtt();
        send_config();
    }

#ifdef REPORT_TEMPERATURE
    read_temperature( &temperature );
    if ( !isnan( temperature ) ) {
        round_float( &temperature, 2 );
        json_doc["temperature"] = temperature;
    }
#endif

#ifdef REPORT_VOLTAGE
    read_voltage( &voltage );
    if ( !isnan( voltage ) ) {
        round_float( &voltage, 2 );
        json_doc["voltage"] = voltage;
    }
#endif

    if ( !json_doc.isNull() ) {
        send_state( &json_doc );
    }

    digitalWrite( LED_STATUS_PIN, LOW );

    delay( REPORT_INTERVAL );
}

//=============================================================================
//Setup functions
//=============================================================================

void setup_wifi() {
    while ( wifi_status != WL_CONNECTED ) {
        Serial.print( "Connecting to SSID: " );
        Serial.println( ssid );
        wifi_status = WiFi.begin( ssid, pass );
        delay( 10000 );
    }
    Serial.println( "Connected" );
    WiFi.maxLowPowerMode();
}

void setup_mqtt() {
    Serial.print( "Connecting to broker: " );
    Serial.println( mqtt_server );

    mqtt_client.setId( sensor_name );
    mqtt_client.setUsernamePassword( mqtt_username, mqtt_password );

    while ( !mqtt_client.connect( mqtt_server, mqtt_port ) ) {
        Serial.print( "Error: " );
        Serial.println( mqtt_client.connectError() );
        delay( 10000 );
    }
}

void setup_serial( bool blocking ) {
    Serial.begin( 115200 );

    if ( blocking ) {
        while ( !Serial ) {
            ;
        }
    }
}

//=============================================================================
//MQTT functions
//=============================================================================

bool mqtt_publish( MqttClient *client, char *topic, char *buf, bool retain = false, uint8_t qos = 0, bool dup = false ) {
    int ret = 0;
    ret = client->beginMessage( topic, strlen( buf ), retain, qos, dup );
    if ( ret == 1 ) {
        client->print( buf );
        ret = client->endMessage();
    }

    return ( ret == 1 );
}

void mqtt_publish_config( MqttClient *client, char *topic, char *buf ) {
    bool ret = mqtt_publish( client, topic, buf, true, 0, false );

    if ( !ret ) {
        Serial.print( "Failed to publish config: " );
        Serial.println( buf );
    }
    else {
        Serial.print( "Published config: " );
        Serial.println( buf );
    }
}

void send_config() {
    char buf[512];

#ifdef REPORT_TEMPERATURE
    build_config_temperature( buf, sizeof( buf ) );
    mqtt_publish_config( &mqtt_client, mqtt_topic_config_temperature, buf );
#endif

#ifdef REPORT_VOLTAGE
    build_config_voltage( buf, sizeof( buf ) );
    mqtt_publish_config( &mqtt_client, mqtt_topic_config_voltage, buf );
#endif
}

void send_state( JsonDocument* document ) {
    int buf_size = 512;
    char buf[buf_size];

    serializeJson( *document, buf, buf_size );

    if ( !mqtt_publish( &mqtt_client, mqtt_topic_state, buf, false, 0, false ) ) {
        Serial.print( "Failed to publish state: " );
        Serial.println( buf );
    }
    else {
        Serial.print( "Published: " );
        Serial.println( buf );
    }
}

//=============================================================================
//Utility functions
//=============================================================================

void round_float( float* number, int decimals ) {
    int factor = 10 ^ decimals;
    if ( *number > 0 ) {
        *number = floor( *number * factor + 0.5 ) / factor;
    }
    else {
        *number = ceil( *number * factor - 0.5 ) / factor;
    }
}

//=============================================================================
//Sensor functions
//=============================================================================

#ifdef REPORT_TEMPERATURE
void build_config_temperature( char* buf, int buf_size ) {
    snprintf( buf, buf_size, "{\"unique_id\":\"%s_temp\",\"state_topic\":\"homeassistant/sensor/%s/state\",\"name\":\"%s Temperature\",\"device_class\":\"temperature\",\"unit_of_measurement\":\"°F\",\"value_template\":\"{{ value_json.temperature }}\",\"device\":{\"name\":\"%s\",\"identifiers\":\"%s\"}}", sensor_name, sensor_name, sensor_name_nice, sensor_name_nice, sensor_name );
    buf[buf_size - 1] = '\0';
}

void read_temperature( float* temperature_f ) {
    float temperature_c = NAN;
    *temperature_f = NAN;
#ifdef USE_BMP280
    if ( sensor.takeForcedMeasurement() ) {
        temperature_c = sensor.readTemperature();
    }
#elif defined( USE_SHT31 )
    temperature_c = sensor.readTemperature();
#endif
    if ( !isnan( temperature_c ) ) {
        *temperature_f = temperature_c * 9 / 5 + 32;
    }
    if ( isnan( *temperature_f ) ) {
        Serial.println( "Unable to take measurement" );
    }
}
#endif

#ifdef REPORT_VOLTAGE
void build_config_voltage( char* buf, int buf_size ) {
    snprintf( buf, buf_size, "{\"unique_id\":\"%s_voltage\",\"state_topic\":\"homeassistant/sensor/%s/state\",\"name\":\"%s Voltage\",\"device_class\":\"voltage\",\"unit_of_measurement\":\"V\",\"value_template\":\"{{ value_json.voltage }}\",\"device\":{\"name\":\"%s\",\"identifiers\":\"%s\"}}", sensor_name, sensor_name, sensor_name_nice, sensor_name_nice, sensor_name );
    buf[buf_size - 1] = '\0';
}

void read_voltage( float* voltage_v ) {
    *voltage_v = analogRead( VBAT_PIN );
    *voltage_v = *voltage_v * 2 * 3.3 / 1024;
}
#endif

