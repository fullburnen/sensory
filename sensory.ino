#include "user_config.h"

#include <ArduinoMqttClient.h>
#include <WiFi101.h>
#include <ArduinoJson.h>

//#define USE_RTC
#ifdef USE_RTC
#include <RTCZero.h>
#endif

#define REPORT_INTERVAL_MS REPORT_INTERVAL_S * 1000

#ifdef USE_BMP280
#include <Adafruit_BMP280.h>
#undef REPORT_HUMIDITY
#elif defined( USE_SHT31 )
#include <Adafruit_SHT31.h>
#undef REPORT_PRESSURE
#endif

#ifdef REPORT_VOLTAGE
#ifndef VOLTAGE_PIN
#define VOLTAGE_PIN A7
#endif
#endif

#define LED_STATUS_PIN 13

#ifdef REPORT_TEMPERATURE
#define MQTT_TOPIC_CONFIG_TEMPERATURE "homeassistant/sensor/" SENSOR_NAME "_T/config"
#endif
#ifdef REPORT_VOLTAGE
#define MQTT_TOPIC_CONFIG_VOLTAGE "homeassistant/sensor/" SENSOR_NAME "_V/config"
#endif
#ifdef REPORT_HUMIDITY
#define MQTT_TOPIC_CONFIG_HUMIDITY "homeassistant/sensor/" SENSOR_NAME "_H/config"
#endif
#ifdef REPORT_PRESSURE
#define MQTT_TOPIC_CONFIG_PRESSURE "homeassistant/sensor/" SENSOR_NAME "_P/config"
#endif

#define MQTT_TOPIC_STATE "homeassistant/sensor/" SENSOR_NAME "/state"

//=============================================================================
//Globals
//=============================================================================

#ifdef USE_RTC
RTCZero rtc;
#endif

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

bool use_metric = true;

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
#ifdef REPORT_HUMIDITY
char mqtt_topic_config_humidity[] = MQTT_TOPIC_CONFIG_HUMIDITY;
#endif
#ifdef REPORT_PRESSURE
char mqtt_topic_config_pressure[] = MQTT_TOPIC_CONFIG_PRESSURE;
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
    delay( 3000 );
    setup_serial( false );
    Serial.println( "Booting" );

#ifdef USE_METRIC
    use_metric = true;
#else
    use_metric = false;
#endif

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

    byte mac_addr[6];
    WiFi.macAddress( mac_addr );
    char mac_addr_str[18];
    snprintf( mac_addr_str, sizeof( mac_addr_str ), "%x:%x:%x:%x:%x:%x", mac_addr[5], mac_addr[4], mac_addr[3], mac_addr[2], mac_addr[1], mac_addr[0] );
    mac_addr_str[sizeof( mac_addr_str ) - 1] = '\0';
    Serial.print( "Wifi MAC: " );
    Serial.println( mac_addr_str );

    //Set LED pin to indicate activity
    pinMode( LED_STATUS_PIN, OUTPUT );

#ifdef USE_RTC
    rtc.begin();
    rtc.setEpoch( 0 );
#endif

    setup_wifi();
    setup_mqtt();
    send_config();

    Serial.println( "Boot done" );
}

void loop() {
#ifdef REPORT_TEMPERATURE
    double temperature = NAN;
#endif
#ifdef REPORT_VOLTAGE
    double voltage = NAN;
#endif
#ifdef REPORT_HUMIDITY
    double humidity = NAN;
#endif
#ifdef REPORT_PRESSURE
    double pressure = NAN;
#endif
    StaticJsonDocument<512> json_doc;

    digitalWrite( LED_STATUS_PIN, HIGH );
    delay( 1000 );

#ifdef USE_RTC
    rtc.disableAlarm();
#endif

    if ( !Serial ) {
        setup_serial( false );
    }

    wifi_status = WiFi.status();
    if ( wifi_status != WL_CONNECTED ) {
        Serial.println( "Connection lost" );
        mqtt_client.stop();
        WiFi.end();
        wifi_status = WL_IDLE_STATUS;
        setup_wifi();
    }
    if ( !mqtt_client.connected() ) {
        setup_mqtt();
        send_config();
    }

#ifdef REPORT_TEMPERATURE
    read_temperature( &temperature );
    if ( !isnan( temperature ) ) {
        temperature = round_double( temperature, 2 );
        json_doc["temperature"] = temperature;
    }
#endif

#ifdef REPORT_VOLTAGE
    read_voltage( &voltage );
    if ( !isnan( voltage ) ) {
        voltage = round_double( voltage, 1 );
        json_doc["voltage"] = voltage;
    }
#endif

#ifdef REPORT_HUMIDITY
    read_humidity( &humidity );
    if ( !isnan( humidity ) ) {
        humidity = round_double( humidity, 2 );
        json_doc["humidity"] = humidity;
    }
#endif

#ifdef REPORT_PRESSURE
    read_pressure( &pressure );
    if ( !isnan( pressure ) ) {
        pressure = round_double( pressure, 2 );
        json_doc["pressure"] = pressure;
    }
#endif

    if ( !json_doc.isNull() ) {
        send_json( mqtt_topic_state, &json_doc );
    }

    digitalWrite( LED_STATUS_PIN, LOW );

#ifdef USE_RTC
    rtc.setAlarmEpoch( rtc.getEpoch() + REPORT_INTERVAL_S );
    rtc.enableAlarm( rtc.MATCH_YYMMDDHHMMSS );
    rtc.standbyMode();
#else
    delay( REPORT_INTERVAL_MS );
#endif
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

    IPAddress addr = WiFi.localIP();
    char buf[16];
    IPAddress_to_cstr( addr, buf, sizeof( buf ) );
    Serial.print( "Wifi IP: " );
    Serial.println( buf );

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
    StaticJsonDocument<512> json_doc;

#ifdef REPORT_TEMPERATURE
    build_config_temperature( &json_doc );
    send_json( mqtt_topic_config_temperature, &json_doc );
    json_doc.clear();
#endif

#ifdef REPORT_VOLTAGE
    build_config_voltage( &json_doc );
    send_json( mqtt_topic_config_voltage, &json_doc );
    json_doc.clear();
#endif

#ifdef REPORT_HUMIDITY
    build_config_humidity( &json_doc );
    send_json( mqtt_topic_config_humidity, &json_doc );
    json_doc.clear();
#endif

#ifdef REPORT_PRESSURE
    build_config_pressure( &json_doc );
    send_json( mqtt_topic_config_pressure, &json_doc );
    json_doc.clear();
#endif
}

void send_json( char *topic, JsonDocument *document ) {
    int buf_size = 512;
    char buf[buf_size];

    if ( topic == NULL || strlen( topic ) == 0 || document == NULL ) {
        Serial.println( "Failed to publish: null" );
        return;
    }

    serializeJson( *document, buf, buf_size );

    if ( !mqtt_publish( &mqtt_client, topic, buf, false, 0, false ) ) {
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

double round_double( double number, int decimals ) {
    double factor = pow( 10, decimals );
    return round( number * factor ) / factor;
}

bool IPAddress_to_cstr( IPAddress addr, char* buf, int buf_size ) {
    if ( buf_size < 16 ) {
        return false;
    }

    uint8_t octet[4];
    for ( int i = 0; i < 4; i++ ) {
        octet[i] = addr >> ( i * 8 );
    }
    snprintf( buf, buf_size, "%d.%d.%d.%d", octet[0], octet[1], octet[2], octet[3] );
    return true;
}

//=============================================================================
//Sensor functions
//=============================================================================

#ifdef REPORT_TEMPERATURE
void build_config_temperature( JsonDocument *document ) {
    (*document)["unique_id"] = String( sensor_name ) + "_temp";
    (*document)["state_topic"] = "homeassistant/sensor/" + String( sensor_name ) + "/state";
    (*document)["name"] = String( sensor_name_nice ) + " Temperature";
    (*document)["device_class"] = "temperature";
    if ( use_metric ) {
        (*document)["unit_of_measurement"] = "°C";
    }
    else {
        (*document)["unit_of_measurement"] = "°F";
    }
    (*document)["value_template"] = "{{ value_json.temperature }}";
    (*document)["device"]["name"] = sensor_name_nice;
    (*document)["device"]["identifiers"] = sensor_name;
}

void read_temperature( double* temperature ) {
    double temperature_c = NAN;
    *temperature = NAN;
#ifdef USE_BMP280
    if ( sensor.takeForcedMeasurement() ) {
        temperature_c = ( double )sensor.readTemperature();
    }
#elif defined( USE_SHT31 )
    temperature_c = sensor.readTemperature();
#endif
    if ( !isnan( temperature_c ) ) {
        if ( use_metric ) {
            *temperature = temperature_c;
        }
        else {
            *temperature = temperature_c * 9 / 5 + 32;
        }
    }
    if ( isnan( *temperature ) ) {
        Serial.println( "Unable to take temperature measurement" );
    }
}
#endif

#ifdef REPORT_VOLTAGE
void build_config_voltage( JsonDocument *document ) {
    (*document)["unique_id"] = String( sensor_name ) + "_voltage";
    (*document)["state_topic"] = "homeassistant/sensor/" + String( sensor_name ) + "/state";
    (*document)["name"] = String( sensor_name_nice ) + " Voltage";
    (*document)["device_class"] = "voltage";
    (*document)["unit_of_measurement"] = "V";
    (*document)["value_template"] = "{{ value_json.voltage }}";
    (*document)["device"]["name"] = sensor_name_nice;
    (*document)["device"]["identifiers"] = sensor_name;
}

void read_voltage( double* voltage_v ) {
    *voltage_v = ( double )analogRead( VOLTAGE_PIN );
    *voltage_v = *voltage_v * 2 * 3.3 / 1024;
}
#endif

#ifdef REPORT_HUMIDITY
void build_config_humidity( JsonDocument *document ) {
    (*document)["unique_id"] = String( sensor_name ) + "_humidity";
    (*document)["state_topic"] = "homeassistant/sensor/" + String( sensor_name ) + "/state";
    (*document)["name"] = String( sensor_name_nice ) + " Humidity";
    (*document)["device_class"] = "humidity";
    (*document)["unit_of_measurement"] = "%";
    (*document)["value_template"] = "{{ value_json.humidity }}";
    (*document)["device"]["name"] = sensor_name_nice;
    (*document)["device"]["identifiers"] = sensor_name;
}

void read_humidity( double* humidity ) {
    *humidity = NAN;
#ifdef USE_SHT31
    *humidity = sensor.readHumidity();
#endif
    if ( isnan( *humidity ) ) {
        Serial.println( "Unable to take humidity measurement" );
    }
}
#endif

#ifdef REPORT_PRESSURE
void build_config_pressure( JsonDocument *document ) {
    (*document)["unique_id"] = String( sensor_name ) + "_pressure";
    (*document)["state_topic"] = "homeassistant/sensor/" + String( sensor_name ) + "/state";
    (*document)["name"] = String( sensor_name_nice ) + " Pressure";
    (*document)["device_class"] = "pressure";
    (*document)["unit_of_measurement"] = "Pa";
    (*document)["value_template"] = "{{ value_json.pressure }}";
    (*document)["device"]["name"] = sensor_name_nice;
    (*document)["device"]["identifiers"] = sensor_name;
}

void read_pressure( double* pressure ) {
    *pressure = NAN;
#ifdef USE_BMP280
    *pressure = sensor.readPressure();
#endif
    if ( isnan( *pressure ) ) {
        Serial.println( "Unable to take pressure measurement" );
    }
}
#endif
