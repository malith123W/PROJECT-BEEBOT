#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <Wire.h>
#include <esp_task_wdt.h>

// Constants
const int WIFI_TIMEOUT_MS = 10000;
const int MQTT_TIMEOUT_MS = 5000;
const int WATCHDOG_TIMEOUT_S = 10;
const int MAX_MOTOR_SPEED = 100;
const int MIN_MOTOR_SPEED = -100;
const int JSON_BUFFER_SIZE = 256;

// Pin definitions
const int redLED = 4;
const int blueLED = 16;
const int greenLED = 17;

// Motor pins
const int pwmA = 32;
const int in1 = 33;
const int in2 = 25;
const int pwmB = 13;
const int in3 = 27;
const int in4 = 14;
const int stby = 26;

// Wi-Fi credentials - should be moved to a separate config file
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
//const int MPU = 0x68;
float GyroX, GyroY, GyroZ;
float angle;
//float GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

const uint8_t MPU_ADDR = 0x68;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_DATA = 0x43;
const float GYRO_SCALE_1000DPS = 32.8f; // Accurate scale factor
float GyroErrorZ = 0.0f; // Renamed for clarity




String id = "";
bool idflag = false;
bool good = false;
int idx = 0;
double arr[3];  // Temporary storage for values


bool turningDone = false; // flag true if tuning is done
bool movingDone = false;  // flag true if robot at the destination
double prvstartAngle = 0; // vaiable used to track start angle changes
double spd = 125;         // speed of the movements: [-255, 255]

int tcount = 0;
double dirCorrection = -1;
double prevDist = 0;



// JSON decoded variables
double startAngle = 0, endAngle = 0, travelDis = 0;

// PID variables
double Setpoint, Input, Output;
bool newData = false;

// variables to hold temp data
String reciveStr = "";

// PID configuration
PID myPID(&Input, &Output, &Setpoint, 16, 0, 0.23, DIRECT);

void dataDecoder(char c);
void MoL(int val);
void MoR(int val);
bool updateGyro();
void turn();
void move();
void stopMotors();
bool setup_wifi();
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool mqttReconnect();
void enableMotors();
void disableMotors();
void setupMotors();
void LED(byte color);
bool calculate_IMU_error(); 
void pulse(int pulsetime, int time);
void intShow();
void emergencyStop();

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22, 400000);
  Serial.println("BeeBot Starting...");
  
  // Initialize watchdog timer
  //esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
  //esp_task_wdt_add(NULL);
  
  if (!setup_wifi()) {
    Serial.println("Failed to connect to WiFi. Restarting...");
    ESP.restart();
  }
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  
  setupMotors();
  Serial.println("Motors initialized");

  myPID.SetOutputLimits(MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  myPID.SetSampleTime(20);
  myPID.SetMode(AUTOMATIC);
  Setpoint = 0;
  
  if (!calculate_IMU_error()) {
    Serial.println("IMU calibration failed. Restarting...");
    ESP.restart();
  }
  
  delay(20);
  intShow();
}

void loop() {
  // Feed the watchdog
  //esp_task_wdt_reset();
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost. Reconnecting...");
    if (!setup_wifi()) {
      Serial.println("Failed to reconnect to WiFi. Restarting...");
      ESP.restart();
    }
  }
  
  if (!client.connected()) {
    if (!mqttReconnect()) {
      Serial.println("Failed to reconnect to MQTT. Restarting...");
      ESP.restart();
    }
  }
  
  client.loop();


  // LED status indication (non-blocking)
  static unsigned long lastLEDUpdate = 0;
  if(millis() - lastLEDUpdate > 1000) {
    LED((client.connected() ? 2 : 4)); // Green=connected, Red=disconnected
    lastLEDUpdate = millis();
  }
  

// Serial input for JSON commands
// Serial input for JSON commands


  if (newData) {
    Serial.println("Executing new movement command...");

    // Turn to start angle
      // start turning process if the start angle is above the "turningThresh"
      if ((-turningThresh > startAngle) || (turningThresh < startAngle))
      {

        Serial.println(" start turning process");
        turn();
        LED(0);
      }
      else{
        turningDone = true;
        Serial.println("  turning done");
      }

      // set the movingDone flag if the robo is at the destination
      if (travelDis < distThresh)
      {
        Serial.println(" set the movingDone");
        movingDone = true;
      }
      else
      {

        movingDone = false;
        Serial.println(" set the moving false");
      }

      if ((tcount < 40) && turningDone && newData && !movingDone) //run motors with PID if conditions are satisfied
      {
        Serial.println(" update gyro");
        Setpoint = 0; // set the gyro setpoint to 0
        if (!updateGyro()) {
          Serial.println("Error: Failed to update gyro");
          return;
        }
        Input = (double)angle;
        myPID.Compute();

        Serial.println(String(Output) );
        MoL(-50);
        MoR(-50);
      }
      else
      {
        LED(0);
        newData = false;
        MoL(0);
        MoR(0);
      }

      tcount++;
      delay(5);

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




void MoL(int val) {
  if (val < MIN_MOTOR_SPEED || val > MAX_MOTOR_SPEED) {
    Serial.println("Error: Invalid motor speed value");
    return;
  }
  val = constrain(val, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  digitalWrite(in1, val > 0 ? HIGH : LOW);
  digitalWrite(in2, val > 0 ? LOW : HIGH);
  analogWrite(pwmB, abs(val));
}





void MoR(int val) {
  if (val < MIN_MOTOR_SPEED || val > MAX_MOTOR_SPEED) {
    Serial.println("Error: Invalid motor speed value");
    return;
  }
  val = constrain(val, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  digitalWrite(in4, val > 0 ? HIGH : LOW);
  digitalWrite(in3, val > 0 ? LOW : HIGH);
  analogWrite(pwmA, abs(val));
}





bool setup_wifi() {
  unsigned long startAttemptTime = millis();
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startAttemptTime > WIFI_TIMEOUT_MS) {
      Serial.println("\nWiFi connection timeout");
      return false;
    }
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}





void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (length > JSON_BUFFER_SIZE) {
    Serial.println("Error: Message too large");
    return;
  }

  StaticJsonDocument<JSON_BUFFER_SIZE> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (!doc.containsKey("id") || !doc["id"].is<String>()) {
    Serial.println("Error: Missing or invalid 'id' field");
    return;
  }

  if (!doc.containsKey("startAngle") || !doc["startAngle"].is<double>()) {
    Serial.println("Error: Missing or invalid 'startAngle' field");
    return;
  }

  const char* receivedId = doc["id"];
  startAngle = doc["startAngle"].as<double>();
  travelDis = doc["distance"];
  endAngle = doc["endAngle"];
  newData = true;

  // Build a command string matching your serial format
  String command = String(receivedId) + "," 
                 + String(startAngle, 2) + "," 
                 + String(travelDis, 2) + "," 
                 + String(endAngle, 2) + "\\n";

  // Feed each character to dataDecoder() like serial input
Serial.println(command);
for (int i = 0; i < command.length(); i++) {
    dataDecoder(command[i]);
    
  }

Serial.printf("Start: %.2f, End: %.2f, Distance: %.2f\n", startAngle, endAngle, travelDis);


}





bool mqttReconnect() {
  unsigned long startAttemptTime = millis();
  
  while (!client.connected()) {
    if (millis() - startAttemptTime > MQTT_TIMEOUT_MS) {
      Serial.println("MQTT connection timeout");
      return false;
    }

    String clientId = "esp32_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.print("Attempting MQTT connection... ");
    
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      String topic = "swarm/esp32/" + clientId + "/command";
      client.subscribe(topic.c_str());
      Serial.println("Subscribed to topic: " + topic);
      return true;
    }
    
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" trying again in 1 second");
    delay(1000);
  }
  return true;
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




bool updateGyro() {
  static unsigned long previousTime = 0;
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0f;
  previousTime = currentTime;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_DATA);
  if (Wire.endTransmission(false) != 0) {
    return false; // I2C error
  }

 if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6, (bool)true) == 6){
    // Read and process all three axes
    int16_t rawX = Wire.read() << 8 | Wire.read();
    int16_t rawY = Wire.read() << 8 | Wire.read();
    int16_t rawZ = Wire.read() << 8 | Wire.read();

    // Convert to degrees/sec and apply calibration
    GyroX = rawX / GYRO_SCALE_1000DPS;
    GyroY = rawY / GYRO_SCALE_1000DPS;
    GyroZ = (rawZ / GYRO_SCALE_1000DPS) - GyroErrorZ;

    // Integrate to get angle
    angle += GyroZ * elapsedTime;
    
    return true;
  }
  return false;
}

bool calculate_IMU_error() {
  // Wake up MPU and configure gyro
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Set gyro range to ±1000dps
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x10);
  Wire.endTransmission(true);
  
  delay(100); // Stabilization time

  // Calibrate Z-axis
  int samples = 200;
  float sumErrorZ = 0.0f;
  
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_DATA);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6, (bool)true) == 6) {
      Wire.read(); Wire.read(); // Skip X
      Wire.read(); Wire.read(); // Skip Y
      int16_t rawZ = Wire.read() << 8 | Wire.read();
      sumErrorZ += rawZ / GYRO_SCALE_1000DPS;
      delay(5);
    }
  }
  
  GyroErrorZ = sumErrorZ / samples;
  return true;
}

double radToDegree(double rads) {
  return rads * 180.0 / PI;
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


int count = 0; //temp




// function to decode
void dataDecoder(char c)
{
  LED(4);        //red
  if (c == '\n') // if the endline char
  {
    idflag = true; // start to read the id
    good = false;  // id is not good
    idx = 0;     // reset the index
  }
  else
  {
    if (c == ',') // if comma found
    {
      if (good) // if id is good
      {

        arr[idx] = id.toDouble(); // update the arr
        if (idx == 2)
        {
          newData = true; // set the newdata flag
          tcount = 0;     //when tcount < delay_constant the motor PID will start

//          Serial.println("data recieved");
          startAngle = arr[0]; // do what you want
          endAngle = arr[2];
          travelDis = arr[1];
//          Serial.println("data:" + String(st  artAngle) + " , " + String(endAngle) + " , " + String(travelDis) + ", " + String(angle));
        }
        idx = (idx + 1) % 3; // increment the index
      }
      if (idflag) // if id is getting
      {
        if (id == myID){
          LED(1); //blue
          good = true; // id is good
          delay(20);
          LED(0); //blue
        }
      }
      id = "";        // reset the id
      idflag = false; // id reading done`
    }
    else
      id += c; // append char to the id
  }

  

}





void turn()
{ 
  
  turningDone = false;
  angle = 0;                               //set the current angle to zer0
  //Setpoint = -1 * radToDegree(startAngle); // set the setpoint as the startAngle
  Setpoint =startAngle;

  prvstartAngle = startAngle; // update the prvstartAngle
//  Serial.println("started turning PID " + String(startAngle));

  while (!turningDone)
  {

    client.loop();
    LED(2); //green

    if (prvstartAngle != startAngle) // if there any changes in startAngle, set the current angle to zero and set the set point
    {
      //Setpoint = -1 * radToDegree(startAngle);
      Setpoint =  startAngle;
      angle = 0;
      prvstartAngle = startAngle;
    }
    if (!updateGyro()) {
      Serial.println("Error: Failed to update gyro");
      return;
    }
    Input = (double)angle;
    myPID.Compute();

    Serial.println(String(startAngle) + ", " + String(Setpoint) + "," + String(Input) + ", " + String(Output) + ", ");

    MoL(-Output);
    MoR(Output);

    if(abs(startAngle) < turningThresh) // exit form the loop if the startAngle is bounded in threshold
    {
      Serial.println("turning done");
      turningDone = true;
    }
    LED(0); //off
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




void emergencyStop() {
  MoL(0);
  MoR(0);
  disableMotors();
  LED(4); // Red LED to indicate emergency stop
}

