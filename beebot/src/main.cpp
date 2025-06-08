#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const int redLED = 15;   // G15
const int blueLED = 2;   // G2
const int greenLED = 0;  // G0


// Wi-Fi credentials
const char* ssid = "ZTE Blade V50 Design";
const char* password = "12345678malith";

// MQTT broker settings
const char* mqtt_server = "broker.mqttdashboard.com";
const int mqtt_port = 1883; // Use port 1883 for standard MQTT

WiFiClient espClient;
PubSubClient client(espClient);

// Robot ID
String myID = "1";

// PID control variables
float Kp = 1.5, Ki = 0.0, Kd = 0.5;
float lastError = 0, integral = 0;
float setAngle = 0;

float startAngle = 0, endAngle = 0, travelDis = 0;
volatile bool newData = false;

// Motor pins
const int pwmA = 32;
const int in1 = 33;
const int in2 = 25;
const int pwmB = 13;
const int in3 = 27;
const int in4 = 14;
const int stby = 26; // Standby pin

// Declare motor control functions
void enableMotors();
void disableMotors();

// Wi-Fi connection
void setup_wifi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// MQTT callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.println("Failed to parse JSON");
    return;
  }

  String receivedId = doc["id"].as<String>();
  if (receivedId != myID) return;

  startAngle = doc["x"];
  travelDis = doc["y"];
  endAngle = doc["angle"];
  newData = true;

  Serial.println("Received MQTT command:");
  Serial.println(" - ID: " + receivedId);
  Serial.println(" - x (startAngle): " + String(startAngle));
  Serial.println(" - y (travelDis): " + String(travelDis));
  Serial.println(" - angle (endAngle): " + String(endAngle));
}

// MQTT reconnect
void mqttReconnect() {
  while (!client.connected()) {
    String clientId = "esp32_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.print("Attempting MQTT connection... ");
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      String topic = "swarm/esp32/" + clientId + "/command";
      client.subscribe(topic.c_str());
      Serial.println("Subscribed to topic: " + topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 1 second");
      delay(1000);
    }
  }
}

// Enable and disable motors
void enableMotors() {
  digitalWrite(stby, HIGH);
}

void disableMotors() {
  digitalWrite(stby, LOW);
}

// Motor setup
void setupMotors() {
  pinMode(pwmA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(stby, OUTPUT);
  enableMotors();
}

// Motor movement
void moveForward(int speed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(pwmA, speed);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(pwmB, speed);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(pwmA, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(pwmB, 0);
}

// Simulated angle reading
float readAngle() {
  return setAngle; // Simulated for now
}

void updateAngle() {
  float currentAngle = readAngle();
  float error = setAngle - currentAngle;
  integral += error;
  float derivative = error - lastError;
  float output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int speed = constrain((int)output, -100, 100);
  if (speed > 0) {
    moveForward(speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmA, -speed);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(pwmB, -speed);
  } else {
    stopMotors();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("BeeBot Starting...");

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  setupMotors();
  Serial.println("Motors initialized");
  digitalWrite(redLED,HIGH);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost. Reconnecting...");
    setup_wifi();
  }

  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();

  if (newData) {
    Serial.println("Executing new movement command...");

    // Turn to start angle
    Serial.println("Turning to start angle: " + String(startAngle));
    setAngle = startAngle;
    while (abs(readAngle() - setAngle) > 0.05) {
      updateAngle();
    }
    stopMotors();
    delay(500);

    // Move forward
    Serial.println("Moving forward: " + String(travelDis));
    moveForward(100);
    delay((int)(travelDis * 100)); // Simulated travel delay
    stopMotors();
    delay(500);

    // Turn to end angle
    Serial.println("Turning to end angle: " + String(endAngle));
    setAngle = endAngle;
    while (abs(readAngle() - setAngle) > 0.05) {
      updateAngle();
    }
    stopMotors();

    newData = false;
    Serial.println("Movement command completed");

    // Send completion status
    String clientId = "esp32_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    String statusTopic = "swarm/esp32/" + clientId + "/status";
    JsonDocument doc;
    doc["status"] = "completed";
    doc["id"] = myID;
    String statusMessage;
    serializeJson(doc, statusMessage);
    client.publish(statusTopic.c_str(), statusMessage.c_str());

    Serial.println("Published status: " + statusMessage);
  }
}
