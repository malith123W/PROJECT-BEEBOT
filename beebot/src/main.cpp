#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <Wire.h>

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

bool idflag = false;
String id = "";
double arr[3]{};   // arr to hold startAngle, travelDis, endAngle
int idx = 0;     // index to track the arr index
bool good = false; // bool to check the correct i


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

void MoL(int val) {
  val = constrain(val, -255, 255);
  if (val > 0) {
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
    analogWrite(pwmB, val);
  } else {
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
    analogWrite(pwmB, abs(val));
  }
}

void MoR(int val) {
  val = constrain(val, -255, 255);
  if (val > 0) {
    digitalWrite(in4, HIGH);
    digitalWrite(in3, LOW);
    analogWrite(pwmA, val);
  } else {
    digitalWrite(in4, LOW);
    digitalWrite(in3, HIGH);
    analogWrite(pwmA, abs(val));
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
  previousTime = currentTime;         // Previous time is stored before the actual time read
  currentTime = millis();             // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000.0;            // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43);               // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU, (size_t)6,true);            // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 32.75;// For a 1000deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 32.75;
  GyroZ = GyroZ - GyroErrorZ;

  angle = angle + GyroZ * elapsedTime; // deg/s * s = deg
  // Serial.println(GyroX);

}

double radToDegree(double rads) {
  return (double)(rads * 180.0 / PI);
}

// function to calculate the gyro error
void calculate_IMU_error() 
{
  Wire.begin();           // Initialize comunication
  Wire.beginTransmission(MPU);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B); // Talk to 0 register 6B
  Wire.write(0x00);  // reset
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1B); // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);// Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);

  float sumErrorZ = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU, (size_t)6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorZ = GyroErrorZ + (GyroZ / 32.75);
  }
  GyroErrorZ = sumErrorZ / 200.0;
    // Print the error values on the Serial Monitor
  //  Serial.print("GyroErrorZ: ");
  //  Serial.println(GyroErrorX);
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
  if (Serial.available() > 0)
  {
    dataDecoder(Serial.read());
  }
}

void turn()
{ 
  
  turningDone = false;
  angle = 0;                               //set the current angle to zer0
  Setpoint = -1 * radToDegree(startAngle); // set the setpoint as the startAngle

  prvstartAngle = startAngle; // update the prvstartAngle
//  Serial.println("started turning PID " + String(startAngle));

  while (!turningDone)
  {
    LED(2); //green
    if (Serial.available() > 0)
    {
      // parsing the json string
      dataDecoder(Serial.read());
    }

    if (prvstartAngle != startAngle) // if there any changes in startAngle, set the current angle to zero and set the set point
    {
      Setpoint = -1 * radToDegree(startAngle);
      angle = 0;
      prvstartAngle = startAngle;
    }
    updateGyro();
    Input = (double)angle;
    myPID.Compute();

    // Serial.println(String(startAngle) + ", " + String(Setpoint) + ", " + String(Input) + ", " + String(Output) + ", ");

    MoL(-Output);
    MoR(Output);
    if ((-turningThresh < startAngle) && (turningThresh > startAngle)) // exit form the loop if the startAngle is bounded in threshold
    {
//      Serial.println("turning done");
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



  while(true){
    for(int i =1; i<8; i++){
      LED(i);
      delay(100);
      LED(0);
      delay(100);
      LED(i);
      delay(100);
      LED(0);
      delay(1000);
      }  
    }
  

  // Serial input for JSON commands
  if (Serial.available() > 0) {
    String jsonStr = Serial.readStringUntil('\n');
    dataDecoder(Serial.read()); // parsing the json string
    LED(0);
  }

  if (newData) {
    Serial.println("Executing new movement command...");

    // Turn to start angle
      // start turning process if the start angle is above the "turningThresh"
      if ((-turningThresh > startAngle) || (turningThresh < startAngle))
      {

        turn();
        LED(0);
      }

      // set the movingDone flag if the robo is at the destination
      if (travelDis < distThresh)
      {

        movingDone = true;
      }
      else
      {

        movingDone = false;
      }

      if ((tcount < 40) && turningDone && newData && !movingDone) //run motors with PID if conditions are satisfied
      {
        Setpoint = 0; // set the gyro setpoint to 0
        updateGyro();
        Input = (double)angle;
        myPID.Compute();
        MoL(spd - Output);
        MoR(spd + Output);
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
