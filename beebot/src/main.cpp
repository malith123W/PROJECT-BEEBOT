#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <Wire.h>

const int redLED = 15;
const int blueLED = 2;
const int greenLED = 0;

// Motor pins
const int pwmA = 32;
const int in1 = 33;
const int in2 = 25;
const int pwmB = 13;
const int in3 = 27;
const int in4 = 14;
const int stby = 26;

// Wi-Fi credentials
const char* ssid = "ZTE Blade V50 Design";
const char* password = "12345678malith";

// MQTT broker settings
const char* mqtt_server = "broker.mqttdashboard.com";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// Robot ID
String myID = "1";

const float turningThresh = 0.15;
const double distThresh = 20;
const int MPU = 0x68;
float GyroX, GyroY, GyroZ;
float angle;
float GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

// JSON decoded variables
double startAngle = 0, endAngle = 0, travelDis = 0;

// PID variables
double Setpoint, Input, Output;
bool newData = false;

// PID configuration
PID myPID(&Input, &Output, &Setpoint, 16, 0, 0.23, DIRECT);

void MoL(int val) {
  val = constrain(val, -255, 255);
  if (val > 0) {
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
    analogWrite(pwmA, val);
  } else {
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
    analogWrite(pwmA, abs(val));
  }
}

void MoR(int val) {
  val = constrain(val, -255, 255);
  if (val > 0) {
    digitalWrite(in4, HIGH);
    digitalWrite(in3, LOW);
    analogWrite(pwmB, val);
  } else {
    digitalWrite(in4, LOW);
    digitalWrite(in3, HIGH);
    analogWrite(pwmB, abs(val));
  }
}

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

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.println("Failed to parse JSON");
    return;
  }
  String receivedId = doc["id"].as<String>();
  if (receivedId != myID) return;

  startAngle = doc["startAngle"];
  travelDis = doc["distance"];
  endAngle = doc["endAngle"];
  newData = true;

  Serial.println("Received MQTT command:");
  Serial.println(" - ID: " + receivedId);
  Serial.println(" - startAngle: " + String(startAngle));
  Serial.println(" - distance: " + String(travelDis));
  Serial.println(" - endAngle: " + String(endAngle));
}

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

void enableMotors() {
  digitalWrite(stby, HIGH);
}

void disableMotors() {
  digitalWrite(stby, LOW);
}

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

void updateGyro() {
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 32.75;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 32.75;
  GyroZ = GyroZ - GyroErrorZ;
  angle = angle + GyroZ * elapsedTime;
}

double radToDegree(double rads) {
  return (double)(rads * 180.0 / PI);
}

void calculate_IMU_error() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission(true);
  delay(20);

  float sumErrorZ = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    Wire.read(); Wire.read(); // skip X
    Wire.read(); Wire.read(); // skip Y
    int16_t gz = (Wire.read() << 8 | Wire.read());
    sumErrorZ += (gz / 32.75);
  }
  GyroErrorZ = sumErrorZ / 200.0;
}

void LED(byte color) {
  digitalWrite(redLED, color & 1);
  digitalWrite(greenLED, (color >> 1) & 1);
  digitalWrite(blueLED, (color >> 2) & 1);
}

void intShow() {
  LED(1); // blue
  delay(400);
  LED(2); // green
  delay(400);
  LED(4); // red
  delay(200);
  LED(0);
  delay(1000);
}

void pulse(int pulsetime, int time) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  for (int i = 0; i < time; i++) {
    digitalWrite(pwmA, HIGH);
    digitalWrite(pwmB, HIGH);
    delayMicroseconds(pulsetime / 10);
    digitalWrite(pwmA, LOW);
    digitalWrite(pwmB, LOW);
    delayMicroseconds(pulsetime * 9 / 10);
  }
}

void dataDecoder(String jsonStr) {
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, jsonStr);
  if (error) {
    Serial.println("Failed to parse serial JSON");
    return;
  }
  String receivedId = doc["id"].as<String>();
  if (receivedId != myID) return;

  startAngle = doc["startAngle"];
  travelDis = doc["distance"];
  endAngle = doc["endAngle"];
  newData = true;
}

void turn() {
  bool turningDone = false;
  angle = 0;
  Setpoint = startAngle; // now in degrees

  while (!turningDone) {
    LED(2); // green
    updateGyro();
    Input = (double)angle;
    myPID.Compute();
    MoL(-Output);
    MoR(Output);
    if (abs(startAngle - angle) < turningThresh) {
      turningDone = true;
    }
    LED(0); // off
  }
  angle = 0;
  MoL(0);
  MoR(0);
}

void moveForward(int speed) {
  MoL(speed);
  MoR(speed);
}

void stopMotors() {
  MoL(0);
  MoR(0);
}

void setup() {
  Serial.begin(115200);
  Serial.println("BeeBot Starting...");
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  setupMotors();
  Serial.println("Motors initialized");

  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(20);
  myPID.SetMode(AUTOMATIC);
  Setpoint = 0;
  calculate_IMU_error();
  delay(20);
  intShow();
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

  // Serial input for JSON commands
  if (Serial.available() > 0) {
    String jsonStr = Serial.readStringUntil('\n');
    dataDecoder(jsonStr);
    LED(0);
  }

  if (newData) {
    Serial.println("Executing new movement command...");

    // Turn to start angle
    Serial.println("Turning to start angle: " + String(startAngle));
    turn();
    stopMotors();
    delay(500);

    // Move forward
    Serial.println("Moving forward: " + String(travelDis));
    moveForward(100);
    delay((int)(travelDis * 10)); // Simulated travel delay
    stopMotors();
    delay(500);

    // Turn to end angle
    Serial.println("Turning to end angle: " + String(endAngle));
    startAngle = endAngle;
    turn();
    stopMotors();

    newData = false;
    Serial.println("Movement command completed");

    // Send completion status
    String clientId = "esp32_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    String statusTopic = "swarm/esp32/" + clientId + "/status";
    StaticJsonDocument<128> doc;
    doc["status"] = "completed";
    doc["id"] = myID;
    String statusMessage;
    serializeJson(doc, statusMessage);
    client.publish(statusTopic.c_str(), statusMessage.c_str());
    Serial.println("Published status: " + statusMessage);
  }
}
