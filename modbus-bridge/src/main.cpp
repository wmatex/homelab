#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ModbusMaster.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <WiFiUdp.h>
#include <cstdio>

const char *ssid = "Rychly Internety";
const char *password = "smrtmlecnebilkovine";

IPAddress staticIP(192, 168, 0, 21);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

const char *mqtt_server = "192.168.0.2";
const int mqtt_port = 1883;
const char *mqtt_client_id = "NodeMCU_Sofar_Bridge";

const char *TOPIC_PREFIX = "sofar/";

#define RS485_TX_PIN D4
#define HEAT_PUMP_SLAVE_ID 1

// Use a struct to hold data for each register
// Register data types for different value interpretations
enum class RegisterType { temp, number, bits };

struct ModbusRegister {
    const uint16_t address;
    const char *topic;
    const bool write;
    const RegisterType type;
    uint16_t last_value;
};

// Define all the registers we want to handle
ModbusRegister registers[] = {
    // Address, Topic, Write, Type, Last Value
    {0x0, "status/mode", false, RegisterType::number, 0},
    {0xC, "status/power", false, RegisterType::number, 0},
    {0x16, "status/total_production", false, RegisterType::number, 0},
    {0x19, "status/today_production", false, RegisterType::number, 0},
};
const int numRegisters = sizeof(registers) / sizeof(registers[0]);

WiFiClient espClient;
PubSubClient mqttClient(espClient);
ModbusMaster node;
SoftwareSerial rs485(D6, D7); // RX, TX

void preTransmission() { digitalWrite(RS485_TX_PIN, HIGH); }
void postTransmission() { digitalWrite(RS485_TX_PIN, LOW); }

// --- Helper Functions ---
void publishRegisterValue(int index, uint16_t value) {
    if (value != registers[index].last_value) {
        registers[index].last_value = value;
        Serial.printf("[MQTT] Publishing %d to topic %s\n", value,
                      registers[index].topic);

        char msg_buffer[10];
        // For temperatures, divide by 10 to get one decimal place
        if (registers[index].type == RegisterType::temp) {
            float temp = (int16_t)value / 10.0f;
            dtostrf(temp, 1, 1, msg_buffer);
        } else {
            itoa(value, msg_buffer, 10);
        }
        char topic_with_prefix[255];
        sprintf(topic_with_prefix, "%s%s", TOPIC_PREFIX,
                registers[index].topic);
        mqttClient.publish(topic_with_prefix, msg_buffer, true);
    }
}

void MQTTreconnect() {
    while (!mqttClient.connected()) {
        Serial.print("[MQTT] Attempting connection...");
        if (mqttClient.connect(mqtt_client_id)) {
            Serial.println("connected");
            // Subscribe to all command topics
            for (int i = 0; i < numRegisters; i++) {
                if (registers[i].write) {
                    char topic[255];
                    sprintf(topic, "%s%s/set", TOPIC_PREFIX,
                            registers[i].topic);
                    mqttClient.subscribe(topic);
                    Serial.printf("[MQTT] Subscribed to: %s\n",
                                  registers[i].topic);
                }
            }
        } else {
            Serial.printf("[MQTT] failed, rc=%d. Trying again in 5 seconds.\n",
                          mqttClient.state());
            delay(5000);
        }
    }
}

void setupOTA() {
    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);

    // Hostname defaults to esp8266-[ChipID]
    // ArduinoOTA.setHostname("modbus-bridge-esp8266");

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_FS
            type = "filesystem";
        }

        // NOTE: if updating FS this would be the place to unmount FS using
        // FS.end()
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
        }
    });
    ArduinoOTA.begin();
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n[SYSTEM] Starting MQTT Modbus Bridge...");

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

    pinMode(RS485_TX_PIN, OUTPUT);
    digitalWrite(RS485_TX_PIN, LOW);
    rs485.begin(9600);
    node.begin(HEAT_PUMP_SLAVE_ID, rs485);

    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

    setupOTA();
}

void loop() {
    // Handle WiFi, MQTT, and OTA updates
    if (!mqttClient.connected()) {
        MQTTreconnect();
    }
    mqttClient.loop();
    ArduinoOTA.handle();

    Serial.println("--------------------");
    Serial.println("[LOOP] Starting Modbus poll cycle...");

    // Loop through each register defined in the array
    for (int i = 0; i < numRegisters; i++) {
        uint16_t address = registers[i].address;
        Serial.printf("[RTU] Reading register at address 0x%04X (%d)...\n",
                      address, address);

        // Request one holding register at the specified address
        uint8_t result = node.readHoldingRegisters(address, 1);

        // Check the result of the request
        if (result == node.ku8MBSuccess) {
            Serial.println("[RTU] ...Read successful.");
            uint16_t value = node.getResponseBuffer(
                0); // Get the first (and only) register value
            publishRegisterValue(i, value);
        } else {
            Serial.printf("[RTU] ...FAILED to read. Error: 0x%02X\n", result);
        }

        // A small delay between each Modbus request is good practice
        delay(250);
    }

    Serial.println("[LOOP] Poll cycle finished.");
    delay(5000); // Wait 5 seconds before starting the next poll cycle
}

// void loop() {
//     // This part of the loop runs constantly, keeping the MQTT connection
//     alive. if (!mqttClient.connected()) {
//         MQTTreconnect();
//     }
//     mqttClient.loop(); // This MUST be called frequently.
//     ArduinoOTA.handle();
//
//     delay(1000);
//
//     Serial.println("--------------------");
//     Serial.println("[LOOP] Starting Modbus poll cycle...");
//
//     int frame = 0;
//     int lastId = registers[0].address;
//     for (int i = 0; i <= numRegisters; i++) {
//         if (i >= numRegisters || registers[i].address - lastId > 1) {
//             Serial.printf("[RTU] Reading registers (address %d - %d) = %d, "
//                           "index = %d...\n",
//                           lastId - frame + 1, lastId, frame, i);
//             uint8_t result1 =
//                 node.readHoldingRegisters(lastId - frame + 1, frame);
//
//             if (result1 == node.ku8MBSuccess) {
//                 Serial.println("[RTU] ...Read successful.");
//
//                 for (int j = 0; j < frame; j++) {
//                     publishRegisterValue(i - frame + j,
//                                          node.getResponseBuffer(j));
//                 }
//             } else {
//                 Serial.printf("[RTU] ...FAILED to read.  Error: 0x%02X\n",
//                               result1);
//             }
//             frame = 1;
//             delay(250);
//         } else {
//             frame++;
//         }
//
//         if (i < numRegisters) {
//             lastId = registers[i].address;
//         }
//     }
//
//     Serial.println("[LOOP] Poll cycle finished.");
// }
