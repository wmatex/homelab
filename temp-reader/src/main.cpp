// Include the required libraries
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <OneWire.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <cstdio>
#include <cstdlib>

const char *ssid = "Rychly Internety";
const char *password = "smrtmlecnebilkovine";

IPAddress staticIP(192, 168, 0, 22);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

const char *mqtt_server = "192.168.0.2";
const int mqtt_port = 1883;
const char *mqtt_client_id = "NodeMCU_Temperature_Sensor1";

const char *TOPIC = "sensor/temperature/ambient";

// Define the GPIO pin where the DS18B20 data line is connected
#define ONE_WIRE_BUS D4 // This is GPIO2

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to the Dallas Temperature sensor library
DallasTemperature sensors(&oneWire);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

const float THRESHOLD = 0.001;
float g_temp = 0;

void publishRegisterValue(float value) {
    if (abs(value - g_temp) > THRESHOLD) {
        g_temp = value;
        Serial.printf("[MQTT] Publishing %f to topic %s\n", value, TOPIC);

        char msg_buffer[10];
        sprintf(msg_buffer, "%0.2f", value);
        mqttClient.publish(TOPIC, msg_buffer, true);
    }
}

void MQTTreconnect() {
    while (!mqttClient.connected()) {
        Serial.print("[MQTT] Attempting connection...");
        if (mqttClient.connect(mqtt_client_id)) {
            Serial.println("connected");
        } else {
            Serial.printf("[MQTT] failed, rc=%d. Trying again in 5 seconds.\n",
                          mqttClient.state());
            delay(5000);
        }
    }
}

void setup(void) {
    Serial.begin(9600);
    Serial.println("DS18B20 Temperature Sensor Test");

    Serial.println("\n[SYSTEM] Starting MQTT Temperature sensor...");

    WiFi.mode(WIFI_STA);
    WiFi.config(staticIP, gateway, subnet);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("\n[Wi-Fi] Connected.");

    mqttClient.setServer(mqtt_server, mqtt_port);

    sensors.begin();
}

void loop(void) {
    // Handle WiFi, MQTT, and OTA updates
    if (!mqttClient.connected()) {
        MQTTreconnect();
    }
    mqttClient.loop();

    // Request temperature from the sensor
    Serial.print("Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    Serial.println(" DONE");

    // Get the temperature in Celsius from the first sensor on the bus
    float tempC = sensors.getTempCByIndex(0);

    // Check if the sensor returned a valid reading
    // -127 is a common error code for a disconnected sensor
    if (tempC == DEVICE_DISCONNECTED_C) {
        Serial.println("Error: Could not read temperature data!");
        return; // Stop here if there was an error
    }
    //

    // Print the temperatures to the Serial Monitor 🌡️
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print(" °C  |  \n");

    publishRegisterValue(tempC);

    // Wait 2 seconds before the next reading
    delay(2000);
}
