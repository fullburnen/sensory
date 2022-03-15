#include <ArduinoMqttClient.h>
#include <WiFi101.h>

#include "user_config.h"

#ifdef USE_BMP280
#include <Adafruit_BMP280.h>
#define REPORT_TEMPERATURE
#elif defined( USE_SHT31 )
#include <Adafruit_SHT31.h>
#define REPORT_TEMPERATURE
#endif

#define LED_STATUS_PIN 13

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
char mqtt_topic_config[] = MQTT_TOPIC_CONFIG;
char mqtt_topic_temperature[] = MQTT_TOPIC_TEMPERATURE;
char sensor_name[] = SENSOR_NAME;
char sensor_name_nice[] = SENSOR_NAME_NICE;

WiFiClient wifi_client;
MqttClient mqtt_client( wifi_client );

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
    WiFi.lowPowerMode();

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
        send_temperature( &temperature );
    }
#endif

    digitalWrite( LED_STATUS_PIN, LOW );

    delay( REPORT_INTERVAL );
}

#ifdef REPORT_TEMPERATURE
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

bool mqtt_publish( MqttClient *client, char *topic, char *buf, bool retain = false, uint8_t qos = 0, bool dup = false ) {
    int ret = 0;
    ret = client->beginMessage( topic, strlen( buf ), retain, qos, dup );
    if ( ret == 1 ) {
        client->print( buf );
        ret = client->endMessage();
    }

    return ( ret == 1 );
}

void send_config() {
    char buf[512];
    snprintf( buf, sizeof( buf ), "{\"unique_id\":\"%s_temp\",\"state_topic\":\"homeassistant/sensor/%s/state\",\"name\":\"%s Temperature\",\"device_class\":\"temperature\",\"unit_of_measurement\":\"°F\",\"value_template\":\"{{ value_json.temperature }}\",\"device\":{\"name\":\"%s\",\"identifiers\":\"%s\"}}", sensor_name, sensor_name, sensor_name_nice, sensor_name_nice, sensor_name );
    buf[sizeof( buf ) - 1] = '\0';

    if ( !mqtt_publish( &mqtt_client, mqtt_topic_config, buf, true, 0, false ) ) {
        Serial.print( "Failed to publish config: " );
        Serial.println( buf );
    }
    else {
        Serial.print( "Published config: " );
        Serial.println( buf );
    }
}

#ifdef REPORT_TEMPERATURE
void send_temperature( float* temperature_f ) {
    float temperature = 0.00;
    char buf[23]; // {"temperature":-xxx}

    //stupid check for temperature digits
    if ( isnan( *temperature_f ) ) {
        return;
    }
    else if ( *temperature_f > 0 ) {
        temperature = floor( *temperature_f * 100 + 0.5 ) / 100;
    }
    else {
        temperature = ceil( *temperature_f *100 - 0.5 ) / 100;
    }

    snprintf( buf, sizeof( buf ), "{\"temperature\":%.2f}", temperature );
    buf[sizeof( buf ) - 1] = '\0';

    if ( !mqtt_publish( &mqtt_client, mqtt_topic_temperature, buf, false, 0, false ) ) {
        Serial.print( "Failed to publish temperature: " );
        Serial.println( buf );
    }
    else {
        Serial.print( "Published: " );
        Serial.println( buf );
    }
}
#endif
