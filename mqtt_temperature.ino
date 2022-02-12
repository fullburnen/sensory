#include <Adafruit_BMP280.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <WiFi101.h>

#include "arduino_secrets.h"

#define LOOP_INTERVAL 60000

//Wifi globals
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int wifi_status = WL_IDLE_STATUS;

//Sensor globals
Adafruit_BMP280 sensor;

//MQTT globals
char mqtt_server[] = MQTT_SERVER;
int mqtt_port = MQTT_PORT;
char mqtt_username[] = MQTT_USERNAME;
char mqtt_password[] = MQTT_PASSWORD;
char mqtt_topic_config[] = MQTT_TOPIC_CONFIG;
char mqtt_topic_temperature[] = MQTT_TOPIC_TEMPERATURE;
char sensor_name[] = SENSOR_NAME;
char sensor_name_nice[] = SENSOR_NAME_NICE;

WiFiClient client;
Adafruit_MQTT_Client mqtt( &client, mqtt_server, mqtt_port, mqtt_username, mqtt_password ); 
Adafruit_MQTT_Publish mqtt_config = Adafruit_MQTT_Publish( &mqtt, mqtt_topic_config );
Adafruit_MQTT_Publish mqtt_temperature = Adafruit_MQTT_Publish( &mqtt, mqtt_topic_temperature );

void setup() {
    setup_serial( false );

    if ( !sensor.begin() ) {
        Serial.println( "Could not find a valid sensor" );
        delay( 1000 );
    }

    sensor.setSampling( Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500 );

    //For Adafruit ATWINC1500
    WiFi.setPins( 8, 7, 4, 2 );

    if ( WiFi.status() == WL_NO_SHIELD ) {
        while ( true ) {
            delay( 30000 );
            Serial.println( "No wifi detected" );
        }
    }

    pinMode( 13, OUTPUT );

    setup_wifi();
    setup_mqtt();
}

void loop() {
    float temperature = -999.00;

    digitalWrite( 13, HIGH );
    delay( 1000 );

    if ( !Serial ) {
        setup_serial( false );
    }

    wifi_status = WiFi.status();
    if ( wifi_status != WL_CONNECTED ) {
        Serial.print( "Connection lost" );
        mqtt.disconnect();
        WiFi.end();
        wifi_status = WiFi.status();
        setup_wifi();
    }
    if ( !mqtt.connected() ) {
        setup_mqtt();
    }

    read_temperature( &temperature );
    if ( temperature != -999 ) {
        send_temperature( &temperature );
    }

    digitalWrite( 13, LOW );

    delay( LOOP_INTERVAL );
}

void read_temperature( float* temperature_f ) {
    if ( sensor.takeForcedMeasurement() ) {
        *temperature_f = sensor.readTemperature() * 9 / 5 + 32;
        /*
        Serial.print( "Temperature is: " );
        Serial.print( *temperature_f );
        Serial.println( " F" );
        */
    }
    else {
        Serial.println( "Unable to take measurement" );
        *temperature_f = -999.00;
    }
}

void setup_wifi() {
    while ( wifi_status != WL_CONNECTED ) {
        Serial.print( "Connecting to " );
        Serial.println( ssid );
        wifi_status = WiFi.begin( ssid, pass );
        delay( 10000 );
    }
    Serial.println( "Connected" );
    WiFi.maxLowPowerMode();
}

void setup_mqtt() {
    int8_t ret = -1;

    while ( ret != 0 ) {
        ret = mqtt.connect();
        delay( 10000 );
    }

    //send_config();
}

void send_config() {
    char buf[512];
    snprintf( buf, sizeof( buf ), "{\"unique_id\":\"%s_temp\",\"state_topic\":\"homeassistant/sensor/%s/state\",\"name\":\"%s Temperature\",\"device_class\":\"temperature\",\"unit_of_measurement\":\"°F\",\"value_template\":\"{{ value_json.temperature }}\",\"device\":{\"name\":\"%s\",\"identifiers\":\"%s\"}}", sensor_name, sensor_name, sensor_name_nice, sensor_name_nice, sensor_name );
    buf[sizeof( buf ) - 1] = '\0';

    if ( !mqtt_config.publish( buf ) ) {
        Serial.print( "Failed to publish config: " );
        Serial.println( buf );
    }
    else {
        Serial.print( "Published config: " );
        Serial.println( buf );
    }
}

void send_temperature( float* temperature_f ) {
    float temperature = 0.00;
    char buf[23]; // {"temperature":-xxx}

    //stupid check for temperature digits
    if ( *temperature_f > 0 ) {
        temperature = floor( *temperature_f * 100 + 0.5 ) / 100;
    }
    else {
        temperature = ceil( *temperature_f *100 - 0.5 ) / 100;
    }
    if ( temperature > 999.99 || temperature < -99.99 ) {
        return;
    }

    snprintf( buf, sizeof( buf ), "{\"temperature\":%.2f}", temperature );
    buf[sizeof( buf ) - 1] = '\0';

    if ( !mqtt_temperature.publish( buf ) ) {
        Serial.print( "Failed to publish temperature: " );
        Serial.println( buf );
    }
    else {
        Serial.print( "Published: " );
        Serial.println( buf );
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
