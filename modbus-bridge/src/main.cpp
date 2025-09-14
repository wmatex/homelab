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

IPAddress staticIP(192, 168, 0, 20);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

const char *mqtt_server = "192.168.0.2";
const int mqtt_port = 1883;
const char *mqtt_client_id = "NodeMCU_HeatPump_Bridge";

const char *TOPIC_PREFIX = "heatpump/";

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
    // === Primary Control Registers (Read/Write) ===
    // Address, Topic, Write, Type, Last Value
    {1011, "status/power", true, RegisterType::number,
     0}, // Power On/Off (0=OFF, 1=ON) [cite: 1509]
    {1012, "status/mode", true, RegisterType::number,
     0}, // Mode (e.g., 0=Hot water, 1=Heating, etc.) [cite: 1509]
    {1045, "config/forced_switching_time", true, RegisterType::number,
     0}, // H32: Forced switching time for heating water [cite: 1509]
    {1157, "status/dhw_target_temp", true, RegisterType::temp,
     0}, // R01: Domestic hot water target temperature [cite: 1509]
    {1158, "status/heating_target_temp", true, RegisterType::temp,
     0}, // R02: Heating target temperature [cite: 1509]
    {1159, "status/cooling_target_temp", true, RegisterType::temp,
     0}, // R03: Cooling target temperature [cite: 1509]

    // === Configuration Registers (Read/Write) ===
    {1160, "config/heating_startup_diff", true, RegisterType::temp,
     0}, // R04: Heating start-up return difference [cite: 1509]
    {1161, "config/heating_shutdown_diff", true, RegisterType::temp,
     0}, // R05: Heating shutdown temperature difference [cite: 1509]
    {1193, "config/main_pump_op_temp", true, RegisterType::temp,
     0}, // R40: Operating ambient temp of main circulating pump [cite: 1509]
    {1194, "config/ac_pump_op_temp", true, RegisterType::temp,
     0}, // R41: Operating ambient temp of air conditioning pump [cite: 1509]
    {1195, "config/dhw_startup_diff", true, RegisterType::temp,
     0}, // R16: Hot water start-up return difference [cite: 1509]
    {1196, "config/dhw_shutdown_diff", true, RegisterType::temp,
     0}, // R17: Hot water shutdown temperature difference [cite: 1509]
    {1197, "config/circ_pump_mode", true, RegisterType::number,
     0}, // P01: Operating mode of circulating water pump [cite: 1509]
    {1198, "config/circ_pump_interval", true, RegisterType::number,
     0}, // P02: Circulating water pump operation time interval [cite: 1511]
    {1199, "config/circ_pump_duration", true, RegisterType::number,
     0}, // P03: Circulating water pump operation duration [cite: 1511]
    {1201, "config/dhw_pump_mode", true, RegisterType::number,
     0}, // P05: Operating mode of domestic hot water pump [cite: 1511]
    {1202, "config/manual_pump_control", true, RegisterType::number,
     0}, // P06: Manually controlled water pump (Manual typo, likely P06) [cite:
         // 1511]
    {1238, "config/max_outlet_temp", true, RegisterType::temp,
     0}, // R62: Max water outlet temperature of heat pump [cite: 1511]

    // === Read-Only Status Registers ===
    // Address, Topic, Write, Type, Last Value
    {2013, "status/temp_value_after_limit", false, RegisterType::temp,
     0}, // Current temperature value (after limit) [cite: 1511]
    {2014, "status/temp_value_after_compensation", false, RegisterType::temp,
     0}, // Current temperature value (after compensation) [cite: 1511]
    {2019, "status/output_bits", true, RegisterType::bits,
     0}, // Bitfield of system outputs (compressor, fan, etc.) [cite: 1511]
    {2034, "status/input_bits", true, RegisterType::bits,
     0}, // Bitfield of system inputs (switches, etc.) [cite: 1512]
    {2045, "status/inlet_water_temp", false, RegisterType::temp,
     0}, // T01: Water inlet temperature [cite: 1512]
    {2046, "status/outlet_water_temp", false, RegisterType::temp,
     0}, // T02: Water outlet temperature [cite: 1512]
    {2047, "status/dhw_tank_temp", false, RegisterType::temp,
     0}, // T08: Water tank temperature [cite: 1512]
    {2048, "status/ambient_temp", false, RegisterType::temp,
     0}, // T04: Ambient temperature [cite: 1512]
    {2085, "status/error_bits_1", false, RegisterType::bits,
     0}, // First block of error status bits [cite: 1518]
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

// --- Main MQTT and Modbus Functions ---
void callback(char *topic, byte *payload, unsigned int length) {
    Serial.printf("[MQTT] Message arrived on topic: %s\n", topic);

    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    Serial.printf("[MQTT] Message payload: %s\n", message);

    // Find which register this command is for
    for (int i = 0; i < numRegisters; i++) {
        char topic_cmp[255];
        sprintf(topic_cmp, "%s%s/set", TOPIC_PREFIX, registers[i].topic);
        if (registers[i].write && strcmp(topic, topic_cmp) == 0) {
            uint16_t value_to_write = atoi(message);

            // For temperatures, multiply by 10
            if (registers[i].type == RegisterType::temp) {
                value_to_write = (uint16_t)(atof(message) * 10);
            }

            Serial.printf("[RTU] Writing %d to register %d...\n",
                          value_to_write, registers[i].address);
            uint8_t result =
                node.writeSingleRegister(registers[i].address, value_to_write);

            if (result != node.ku8MBSuccess) {
                Serial.printf("[RTU] Write failed with error code: 0x%02X\n",
                              result);
            } else {
                Serial.println("[RTU] Write successful.");
            }
            return; // Exit after handling the command
        }
    }
}

void reconnect() {
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

void setup() {

    Serial.begin(115200);
    Serial.println("\n[SYSTEM] Starting MQTT Modbus Bridge...");

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
    Serial.println("\nNO FUCKING WAY.");

    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(callback);

    pinMode(RS485_TX_PIN, OUTPUT);
    digitalWrite(RS485_TX_PIN, LOW);
    rs485.begin(9600);
    node.begin(HEAT_PUMP_SLAVE_ID, rs485);

    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

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

void loop() {
    // This part of the loop runs constantly, keeping the MQTT connection alive.
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop(); // This MUST be called frequently.
    ArduinoOTA.handle();

    delay(1000);

    Serial.println("--------------------");
    Serial.println("[LOOP] Starting Modbus poll cycle...");

    int frame = 0;
    int lastId = registers[0].address;
    for (int i = 0; i <= numRegisters; i++) {
        if (i >= numRegisters || registers[i].address - lastId > 1) {
            Serial.printf("[RTU] Reading registers (address %d - %d) = %d, "
                          "index = %d...\n",
                          lastId - frame + 1, lastId, frame, i);
            uint8_t result1 =
                node.readHoldingRegisters(lastId - frame + 1, frame);

            if (result1 == node.ku8MBSuccess) {
                Serial.println("[RTU] ...Read successful.");

                for (int j = 0; j < frame; j++) {
                    publishRegisterValue(i - frame + j,
                                         node.getResponseBuffer(j));
                }
            } else {
                Serial.printf("[RTU] ...FAILED to read.  Error: 0x%02X\n",
                              result1);
            }
            frame = 1;
            delay(250);
        } else {
            frame++;
        }

        if (i < numRegisters) {
            lastId = registers[i].address;
        }
    }

    Serial.println("[LOOP] Poll cycle finished.");
}
