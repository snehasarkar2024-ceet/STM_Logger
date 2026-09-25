

#include <WiFi.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>

// ================= WIFI CONFIG =================
const char* ssid     = "4G-UFI-9578";
const char* password = "1234567890";
// const char* ssid     = "Connect@itel";
// const char* password = "WeLoveIndia";

// ================= SERVER / MQTT CONFIG =================
const char* mqtt_server = "121.242.232.220";
const int   mqtt_port   = 5010;
const char* device_id   = "EX1";

String mqtt_pub_topic = "stm/EX1/data";
String mqtt_sub_topic = "stm/EX1/cmd";

// ================= UART CONFIG =================
#define UART_BAUD 115200

// ================= LED CONFIG =================
#define STATUS_LED 3

// ================= GLOBALS =================
HardwareSerial STMSerial(0);

WiFiClient espClient;
PubSubClient mqttClient(espClient);
bool mqttWasConnected = false;

unsigned long lastBlinkTime = 0;
const unsigned long blinkInterval = 500;
bool ledState = LOW;

// ================= STM RX BUFFER =================
// Increased to 512 bytes to safely read fast $CAN frame dumps from STM32
#define STM_RX_BUF_SIZE 512

char stmRxBuf[STM_RX_BUF_SIZE];
uint8_t stmRxIndex = 0;
bool stmLineReady = false;

// =================================================
// SEND DATA TO STM32 VIA UART
// =================================================
void sendToSTM(String type, float value) {
  String message = "TYPE=" + type + ";VALUE=" + String(value, 2);
  Serial.println("[UART] Sending to STM32:");
  Serial.println(message);
  STMSerial.println(message);
}

// =================================================
// MQTT PUBLISH HELPER
// =================================================
void publishMQTT(const char* json)
{
  if (mqttClient.connected())
  {
    mqttClient.publish(mqtt_pub_topic.c_str(), json);
    Serial.print("[MQTT TX] ");
    Serial.println(json);
  }
}

// =================================================
// MQTT CALLBACK (Server → ESP)
// =================================================
void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  String message = "";
  for (unsigned int i = 0; i < length; i++)
    message += (char)payload[i];

  Serial.print("[MQTT RX] ");
  Serial.println(message);

  DynamicJsonDocument doc(2048);
  DeserializationError err = deserializeJson(doc, message);
  
  if (err) 
  {
    Serial.println("❌ JSON parse failed");
    return;
  }

  // Handle nested JSON cmd struct: {"cmd": {"type": "...", "value": ...}}
  JsonObject cmd = doc["cmd"];
  if (!cmd)
  {
    // Fallback: check top-level keys if {"type": "...", "value": ...}
    if (!doc["type"].isNull())
    {
      String type = doc["type"];
      float value = doc["value"] | 0.0f;
      sendToSTM(type, value);
    }
    return;
  }

  String cmdType = cmd["type"];

  // RPM Profile streaming array over UART
  if (cmdType == "rpm_profile") {
    JsonArray rpmArray = cmd["value"].as<JsonArray>();
    if (!rpmArray.isNull()) {
      int size = rpmArray.size();
      for (int i = 0; i < size; i++) {
        int rpm = rpmArray[i];
        STMSerial.println(rpm);
        delay(5);
      }
    }
  }
  else if (!cmd["value"].isNull()) {
    float value = cmd["value"];
    sendToSTM(cmdType, value);
  }
}

// =================================================
// MQTT CONNECT & RECONNECT
// =================================================
void connectMQTT()
{
  while (!mqttClient.connected())
  {
    Serial.print("[MQTT] Connecting...");
    String clientId = "ESP32-" + String(device_id);

    if (mqttClient.connect(clientId.c_str()))
    {
      Serial.println("✅ Connected");
      mqttClient.subscribe(mqtt_sub_topic.c_str());
      mqttWasConnected = true;
    }
    else
    {
      Serial.print("❌ Failed rc=");
      Serial.print(mqttClient.state());
      delay(2000);
    }
  }
}

// =================================================
// LED HANDLER
// =================================================
void updateStatusLED() {
  if (mqttClient.connected()) {
    digitalWrite(STATUS_LED, HIGH);
  }
  else {
    if (millis() - lastBlinkTime >= blinkInterval) {
      lastBlinkTime = millis();
      ledState = !ledState;
      digitalWrite(STATUS_LED, ledState);
    }
  }
}

// =================================================
// STM UART RECEIVE HANDLER
// =================================================
void handleSTM32UART()
{
  while (STMSerial.available())
  {
    char c = STMSerial.read();

    if (c == '\n')
    {
      stmRxBuf[stmRxIndex] = '\0';
      if (!stmLineReady)
        stmLineReady = true;
      stmRxIndex = 0;
    }
    else if (c != '\r')
    {
      if (stmRxIndex < STM_RX_BUF_SIZE - 1)
        stmRxBuf[stmRxIndex++] = c;
      else
        stmRxIndex = 0;
    }
  }
}

// =================================================
// PROCESS STM32 MESSAGES → MQTT
// =================================================
void processSTM32Message()
{
  if (!stmLineReady) return;

  stmLineReady = false;

  Serial.print("[STM32 → ESP] ");
  Serial.println(stmRxBuf);

  // =================================================
  // 1. RAM CAN LOGS FROM STM32 VIA UART7 ($CAN Format)
  // Format: $CAN,<BusID>,<Timestamp>,<CAN_ID>,<DLC>,<HexDataString>
  // Example: $CAN,2,123456,0x07811020,8,1122334455667788
  // =================================================
  if (strncmp(stmRxBuf, "$CAN,", 5) == 0)
  {
    uint32_t busId = 0;
    unsigned long timestamp = 0;
    uint32_t canId = 0;
    uint32_t dlc = 0;
    uint8_t data[8] = {0};

    int ret = sscanf(stmRxBuf,
                     "$CAN,%u,%lu,0x%X,%u,%2hhx%2hhx%2hhx%2hhx%2hhx%2hhx%2hhx%2hhx",
                     &busId,
                     &timestamp,
                     &canId,
                     &dlc,
                     &data[0], &data[1], &data[2], &data[3],
                     &data[4], &data[5], &data[6], &data[7]);

    if (ret >= 4)
    {
      StaticJsonDocument<384> doc;

      doc["type"]      = "ram_can_log";
      doc["device"]    = device_id;
      doc["bus"]       = busId;
      doc["timestamp"] = timestamp;

      char canIdStr[11];
      snprintf(canIdStr, sizeof(canIdStr), "0x%08X", (unsigned int)canId);
      doc["can_id"]    = canIdStr;
      doc["dlc"]       = dlc;

      JsonArray arr = doc.createNestedArray("data");
      for (uint32_t i = 0; i < dlc && i < 8; i++)
      {
        arr.add(data[i]);
      }

      char jsonOut[384];
      serializeJson(doc, jsonOut);
      publishMQTT(jsonOut);
    }
    else
    {
      Serial.println("[ESP] Invalid $CAN RAM format from STM32");
    }

    memset(stmRxBuf, 0, STM_RX_BUF_SIZE);
    return;
  }

  // ================= 2. ACK / NACK MESSAGES =================
  if (strncmp(stmRxBuf, "ACK:", 4) == 0 || strncmp(stmRxBuf, "NACK:", 5) == 0)
  {
    char *typePtr = strtok(stmRxBuf, ":");
    char *dataPtr = strtok(NULL, ":");

    if (typePtr != NULL && dataPtr != NULL)
    {
      char *cmdPtr = strtok(dataPtr, "=");
      char *valPtr = strtok(NULL, "=");

      if (cmdPtr != NULL && valPtr != NULL)
      {
        StaticJsonDocument<256> doc;
        doc["type"]    = typePtr;
        doc["device"]  = device_id;
        doc["command"] = cmdPtr;
        doc["value"]   = atof(valPtr);

        char jsonOut[256];
        serializeJson(doc, jsonOut);
        publishMQTT(jsonOut);
      }
    }
    memset(stmRxBuf, 0, STM_RX_BUF_SIZE);
    return;
  }

  // ================= 3. STREAMING CAN MESSAGES =================
  if (strncmp(stmRxBuf, "CAN", 3) == 0)
  {
    int frameId = 0;
    uint8_t data[8];

    int ret = sscanf(stmRxBuf,
                     "CAN%d:%2hhx%2hhx%2hhx%2hhx%2hhx%2hhx%2hhx%2hhx",
                     &frameId,
                     &data[0], &data[1], &data[2], &data[3],
                     &data[4], &data[5], &data[6], &data[7]);

    if (ret == 9)
    {
      StaticJsonDocument<256> doc;
      doc["type"]   = "can";
      doc["device"] = device_id;
      doc["frame"]  = frameId;

      switch(frameId)
      {
        case 1: doc["source"] = "tmcm_1"; break;
        case 2: doc["source"] = "tmcm_2"; break;
        case 3: doc["source"] = "dcdc_12v"; break;
        case 4: doc["source"] = "dcdc_48v"; break;
        default: doc["source"] = "unknown"; break;
      }

      JsonArray arr = doc.createNestedArray("data");
      for (int i = 0; i < 8; i++)
      {
        arr.add(data[i]);
      }

      char jsonOut[256];
      serializeJson(doc, jsonOut);
      publishMQTT(jsonOut);
    }
    memset(stmRxBuf, 0, STM_RX_BUF_SIZE);
    return;
  }

  // ================= 4. COUNT DATA =================
  if (strncmp(stmRxBuf, "TYPE=COUNT", 10) == 0)
  {
    char type[20] = {0};
    float value = 0;

    int ret = sscanf(stmRxBuf, "TYPE=%19[^;];VALUE=%f", type, &value);
    if (ret == 2)
    {
      StaticJsonDocument<256> doc;
      doc["type"]   = "count";
      doc["device"] = device_id;
      doc["value"]  = (uint32_t)value;

      char jsonOut[256];
      serializeJson(doc, jsonOut);
      publishMQTT(jsonOut);
    }
    memset(stmRxBuf, 0, STM_RX_BUF_SIZE);
    return;
  }

  // ================= 5. TEMP DATA =================
  if (strstr(stmRxBuf, "\"TYPE\":\"TEMP\"") != NULL)
  {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, stmRxBuf);
    if (!err)
    {
      float t1 = doc["T1"];
      float t2 = doc["T2"];

      StaticJsonDocument<256> out;
      out["type"]   = "temp";
      out["device"] = device_id;
      out["t1"]     = t1;
      out["t2"]     = t2;

      char jsonOut[256];
      serializeJson(out, jsonOut);
      publishMQTT(jsonOut);
    }
    memset(stmRxBuf, 0, STM_RX_BUF_SIZE);
    return;
  }

  // ================= UNKNOWN MESSAGE =================
  Serial.println("[ESP] Unknown message type");
  memset(stmRxBuf, 0, STM_RX_BUF_SIZE);
}

// =================================================
// SETUP
// =================================================
void setup()
{
  Serial.begin(115200);
  delay(300);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  STMSerial.begin(UART_BAUD);
  Serial.println("✅ UART Initialized");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println("✅ WiFi Connected");

  // MQTT
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);

  // OTA
  ArduinoOTA.setHostname("esp32c6-EX1");
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
}

// =================================================
// LOOP
// =================================================
void loop()
{
  if (!mqttClient.connected())
    connectMQTT();

  mqttClient.loop();

  // ================= MQTT DISCONNECT SAFETY =================
  if (mqttWasConnected && !mqttClient.connected())
  {
    Serial.println("[MQTT] ❌ Disconnected");
    mqttWasConnected = false;

    Serial.println("[SAFETY] MQTT lost → STOP to STM32");
    sendToSTM("stop", 1.0);
  }

  // ================= STM UART =================
  handleSTM32UART();
  processSTM32Message();

  updateStatusLED();

  ArduinoOTA.handle();
  delay(10);
}