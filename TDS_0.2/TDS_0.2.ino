#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>      // ไลบรารีสำหรับจัดการข้อมูล JSON
#include <OneWire.h>
#include <DallasTemperature.h>

#define TdsSensorPin 33
#define VREF 5.0
#define SCOUNT 5
#define ONE_WIRE_BUS 18 //กำหนดว่าขาของเซนเซอร์ 18B20 ต่อกับขา 2

// Sensor objects
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// WiFi and MQTT configuration
const char* WIFI_SSID = "Gt Neo";
const char* WIFI_PASSWORD = "chatpeth12345678";

// *ตั้งค่าการเชื่อมต่อกับ ThingsBoard MQTT*
const char* mqttServer = "demo.thingsboard.io";  // ที่อยู่เซิร์ฟเวอร์ MQTT ของ ThingsBoard
const int mqttPort = 1883;                       // พอร์ต MQTT
const char* mqttUser = "XpBbpDPlEXssvTJH7nc5";   // Access Token ของอุปกรณ์ใน ThingsBoard
const char* mqttPassword = "";                   // ปกติไม่ต้องใส่รหัสผ่าน (ปล่อยว่าง)


const int relayPin1 = 4; 
const int relayPin2 = 5;
const float TDS_THRESHOLD = 50.0;
const unsigned long RELAY_DURATION = 15000;
const unsigned long STABILIZATION_TIME = 10000; // เวลา 10 วินาทีให้เซ็นเซอร์ stabilize

WiFiClient espClient;
PubSubClient client(espClient);

int analogBuffer[SCOUNT];
int analogBufferIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;

bool tdsEnabled = true;
unsigned long relayStartTime = 0;
bool relayActive = false;
unsigned long enableTime = 0; // เวลาเมื่อเปิดการทำงาน TDS
bool firstReadingAfterEnable = true; // ตรวจสอบว่าเป็นค่าอ่านแรกหลังจากเปิดหรือไม่



int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];
  
  for (int j = 0; j < iFilterLen - 1; j++) {
    for (int i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        int bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  
  return (iFilterLen & 1) ? bTab[(iFilterLen - 1) / 2] : (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
}



void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("connected");
      client.subscribe("v1/devices/me/rpc/request/+");
    } else {
      Serial.printf("failed, rc= %d. Try again in 5 seconds\n", client.state());
      delay(5000);
    }
  }
}


// *ฟังก์ชัน callback สำหรับรับคำสั่ง MQTT*
void callback(char* topic, byte* payload, unsigned int length) {
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  
  Serial.print("Received [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.println("JSON parse error: " + String(error.c_str()));
    return;
  }

  String method = doc["method"].as<String>();
  method.trim();

  // ตรวจสอบว่าคำสั่งที่ได้รับเกี่ยวกับปั๊มตัวไหน
  bool pumpState = false;
  int relayPin = -1;

  if (method == "pump1") {
    relayPin = relayPin1;
  } else if (method == "pump2") {
    relayPin = relayPin2;
  } else {
    Serial.println("Unknown command");
    return;
  }

  if (doc["params"].is<bool>()) {
    pumpState = doc["params"];
  } else if (doc["params"].is<const char*>()) {
    String param = doc["params"].as<String>();
    if (param.equalsIgnoreCase("ON") || param == "1" || param.equalsIgnoreCase("true")) {
      pumpState = true;
    }
  } else if (doc["params"].isNull()) {
    pumpState = !digitalRead(relayPin);  // Toggle สถานะ
  }

  digitalWrite(relayPin, pumpState ? LOW : HIGH);  // LOW = เปิด (Active Low)
  Serial.printf("Relay %d set to: %s\n", relayPin, pumpState ? "ON" : "OFF");

}

void setup() {
  
  Serial.begin(115200);
  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);

  digitalWrite(relayPin1, LOW);
  digitalWrite(relayPin2, HIGH);
  pinMode(TdsSensorPin, INPUT);
  sensors.begin();
  Serial.println("System Initialized");

    // Initialize WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  
  // Initialize MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  client.publish("v1/devices/me/attributes", "{\"led\":true}");

  enableTime = millis(); // ตั้งค่าเวลาเริ่มต้นเมื่อเปิดระบบ
}

void loop() {
   if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static unsigned long analogSampleTimepoint = millis();
  static unsigned long thingsboardTimepoint = millis();
    sensors.requestTemperatures();
    float temperatureC = sensors.getTempCByIndex(0);

  delay(1000);

  if (tdsEnabled && millis() - analogSampleTimepoint > 500) {
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex = (analogBufferIndex + 1) % SCOUNT;
    analogSampleTimepoint = millis();
  }
  
  static unsigned long printTimepoint = millis();
  if (tdsEnabled && millis() - printTimepoint > 2000) {
    int analogBufferTemp[SCOUNT];
    for (int i = 0; i < SCOUNT; i++) analogBufferTemp[i] = analogBuffer[i];
    
    int median = getMedianNum(analogBufferTemp, SCOUNT);
    averageVoltage = median * (float)VREF / 4096.0;
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;
    tdsValue = (133.42 * compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage) * 0.5;
    
    if (tdsValue > 0) {
    Serial.print("Temperature: ");
    Serial.print(temperatureC);
    Serial.println(" °C");
      Serial.print("Voltage: ");
      Serial.print(averageVoltage, 2);
      Serial.print("V, TDS Value: ");
      Serial.print(tdsValue, 0);
      Serial.println(" ppm");
      sendToThingsBoard(tdsValue,temperatureC);
      
      // ตรวจสอบว่าเวลาผ่านไป 10 วินาทีแล้วหรือยัง
      if (millis() - enableTime > STABILIZATION_TIME) {
        if (tdsValue < TDS_THRESHOLD) {
          Serial.println("TDS below threshold - Taking action");
          tdsEnabled = false;
          digitalWrite(relayPin2, LOW); // Activate relay 2
          relayStartTime = millis();
          relayActive = true;
        }
      } else {
        Serial.println("(Stabilizing - waiting for 10 seconds)");
      }
    }
    
    printTimepoint = millis();
  }
  
  if (relayActive && (millis() - relayStartTime >= RELAY_DURATION)) {
    digitalWrite(relayPin2, HIGH);
    relayActive = false;
    tdsValue = 0;
    tdsEnabled = true;
    enableTime = millis(); // รีเซ็ทเวลาเมื่อเปิด TDS ใหม่
    firstReadingAfterEnable = true;
    Serial.println("Action completed - TDS reset and re-enabled");
    sendToThingsBoard(tdsValue,temperatureC);
  }
        

}


void sendToThingsBoard(float tds, float temp) {
  StaticJsonDocument<200> doc;
  doc["TDS"] = tds;
  doc["Temperature"] = temp;
  doc["pump1"] = digitalRead(relayPin1) == LOW;  // แปลงเป็น TRUE ถ้า LOW (Active Low)
  doc["pump2"] = digitalRead(relayPin2) == LOW;

  char buffer[200];
  serializeJson(doc, buffer);
  Serial.println("Sending payload: " + String(buffer));
  client.publish("v1/devices/me/telemetry", buffer);
}