#include <WiFi.h>
#include <PID_v1.h>
#include <Wire.h>


// Constants
const int WIFI_TIMEOUT_MS = 10000;

const int MAX_MOTOR_SPEED = 150;
const int MIN_MOTOR_SPEED = -150;
double spd = -120;         // speed of the movements: [-255, 255]


const String RESPONSE_OK = "OK\r\n";
const String RESPONSE_ERROR = "ERROR\r\n";

// Pin definitions
const int redLED = 16;
const int blueLED = 17;
const int greenLED = 4;

// Motor pins
const int pwmA = 32;
const int in1 = 33;
const int in2 = 25;
const int pwmB = 13;
const int in3 = 27;
const int in4 = 14;
const int stby = 26;

// Wi-Fi credentials - should be moved to a separate config file
const char* ssid = "Dialog 4G 433";
const char* password = "Fe9110EF";
const int port = 8080; // Choose a port (e.g., 8080, 1234, etc.)








// Robot ID
String myID = "1";



const float turningThresh = 0.3;
const double distThresh = 60;
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
PID myPID(&Input, &Output, &Setpoint, 0.25, 0.003, 0.001, DIRECT);

void dataDecoder(char c);
void MoL(int val);
void MoR(int val);
bool updateGyro();
void turn();
void move();
void stopMotors();
bool setup_wifi();
void enableMotors();
void disableMotors();
void setupMotors();
void LED(byte color);
bool calculate_IMU_error(); 
void pulse(int pulsetime, int time);
void intShow();
void emergencyStop();
void processCompleteMessage(String message);

WiFiServer server(port);
WiFiClient client; // <-- Add this global client

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22, 400000);
  Serial.println("BeeBot Starting...");

 // Serial.println("TCP Server started on port " + String(port));
 // Serial.println("Waiting for client connection...");
  
  if (!setup_wifi()) {
    Serial.println("Failed to connect to WiFi. Restarting...");
    ESP.restart();
  }

  server.begin(); // <-- Now called after WiFi is connected

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
   // Serial.println("WiFi lost. Reconnecting...");
    if (!setup_wifi()) {
      //Serial.println("Failed to reconnect to WiFi. Restarting...");
      ESP.restart();
    }
  }

  // Accept new client if none is connected
  if (!client || !client.connected()) {
      client = server.available();
      if (client) {
        //Serial.println("New client connected!");
        client.setTimeout(100); // Set a reasonable timeout
      }
  }
    // LED status indication (non-blocking)
  static unsigned long lastLEDUpdate = 0;
  if(millis() - lastLEDUpdate > 1000) {
    LED((client.connected() ? 2 : 1)); // Green=connected, Red=disconnected
    lastLEDUpdate = millis();
  }




  // If client is connected, read data
  if (client.available() > 0) {
      char c = client.read();
      dataDecoder(c);
    
  }

  // start turning process if the start angle is above the "turningThresh"
    if (newData) {
    Serial.println("Executing new movement command...");
    LED(3);

    // Turn to start angle
      // start turning process if the start angle is above the "turningThresh"
      if ((-turningThresh > startAngle) || (turningThresh < startAngle))
      {

        //Serial.println(" start turning process");
        turn();
        LED(0);
      }
      else{
        turningDone = true;
        //Serial.println("  turning done");
      }

      // set the movingDone flag if the robo is at the destination
      if (travelDis < distThresh)
      {
        //Serial.println(" set the movingDone");
        movingDone = true;
        

      }
      else
      {

        movingDone = false;
        //Serial.println(" set the moving false");
      }




      if ((tcount < 50) && turningDone && newData && !movingDone) //run motors with PID if conditions are satisfied
      {
        //Serial.println(" update gyro");
        Setpoint = 0; // set the gyro setpoint to 0
        if (!updateGyro()) {
          //Serial.println("Error: Failed to update gyro");
          return;
        }
        Input = (double)angle;
        //Serial.println(String(Input) );
        myPID.Compute();

        //Serial.println(String(Output) );
        MoL(spd-Output);
        MoR(spd+Output);
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

   // Serial.println("Movement command completed");
}
}




void MoL(int val) {
  if (val < MIN_MOTOR_SPEED || val > MAX_MOTOR_SPEED) {
    //Serial.println("Error: Invalid motor speed value");
    return;
  }
  val = constrain(val, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  digitalWrite(in1, val > 0 ? HIGH : LOW);
  digitalWrite(in2, val > 0 ? LOW : HIGH);
  analogWrite(pwmA, abs(val));
}





void MoR(int val) {
  if (val < MIN_MOTOR_SPEED || val > MAX_MOTOR_SPEED) {
  //  Serial.println("Error: Invalid motor speed value");
    return;
  }
  val = constrain(val, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  digitalWrite(in4, val > 0 ? HIGH : LOW);
  digitalWrite(in3, val > 0 ? LOW : HIGH);
  analogWrite(pwmB, abs(val));
}





bool setup_wifi() {
  unsigned long startAttemptTime = millis();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  
 // Serial.println("Connected to WiFi!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP()); // ← This is the IP you need
  
//  Serial.println("TCP Server started on port " + String(port));
  return true;
}







void enableMotors() {
  digitalWrite(stby, HIGH);
}





/*void disableMotors() {
  digitalWrite(stby, LOW);
}*/




void setupMotors() {
  pinMode(pwmA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(stby, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);

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
    //Serial.println(angle);

    
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
  //printf("Gyro error Z: %.2f\n", GyroErrorZ);
  return true;
}

double radToDegree(double rads) {
  return rads * 180.0 / PI;
}





void LED(byte color) {
  digitalWrite(blueLED, color & 1);
  digitalWrite(greenLED, (color >> 1) & 1);
  digitalWrite(redLED, (color >> 2) & 1);
}





void intShow() {
  LED(1); // blue
  delay(400);
  LED(2); // green
  delay(400);
  LED(4); // red
  delay(400);
  LED(0);
  delay(1000);
}





/*void pulse(int pulsetime, int time) {
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
*/

int count = 0; //temp




// function to decode
void dataDecoder(char c) {
  static String currentMessage = "";
  
  if (c == '\n') {  // End of message
    // Process complete message
    if (currentMessage.length() > 0) {
      processCompleteMessage(currentMessage);
      currentMessage = "";
    }
  } else {
    currentMessage += c;
  }
}

void processCompleteMessage(String message) {
  // Reset flags
  idflag = true;
  good = false;
  idx = 0;
  
  int commaPos1 = message.indexOf(',');
  if (commaPos1 == -1) {
    client.print(RESPONSE_ERROR);
    return;
  }

  String receivedId = message.substring(0, commaPos1);
  String remainingData = message.substring(commaPos1 + 1);

  // Split remaining data by commas
  int commaPos2 = remainingData.indexOf(',');
  int commaPos3 = remainingData.indexOf(',', commaPos2 + 1);

  if (commaPos2 == -1 || commaPos3 == -1) {
    client.print(RESPONSE_ERROR);
    return;
  }

  // Check ID match
  if (receivedId == myID) {
    good = true;

    if(good){
    
        // Convert values to doubles
        arr[0] = remainingData.substring(0, commaPos2).toDouble();
        arr[1] = remainingData.substring(commaPos2 + 1, commaPos3).toDouble();
        arr[2] = remainingData.substring(commaPos3 + 1).toDouble();

        // Update global variables
        startAngle = arr[0];
        travelDis = arr[1];
        endAngle = arr[2];
        
        // Send acknowledgment
        client.print(RESPONSE_OK);
        //Serial.printf("Device %s received: %.2f,%.2f,%.2f\n", myID.c_str(), startAngle, travelDis, endAngle);
        
        // Set flags for movement processing
        newData = true;
        tcount = 0;
        turningDone = false;
        movingDone = false;
        }
  }  else {
    // Message not for this device - send error response
    client.print(RESPONSE_ERROR);
   // Serial.printf("Message for ID %s received by device %s\n", 
             //    receivedId.c_str(), myID.c_str());
  }
}



void turn()
{ 
  turningDone = false;
  angle = 0;                               //set the current angle to zer0
  Setpoint = -1 * radToDegree(startAngle); // set the setpoint as the startAngle
  //Setpoint =startAngle;

  prvstartAngle = startAngle; // update the prvstartAngle
  //Serial.println("started turning PID " + String(startAngle));

  while (!turningDone)
  {

  LED(1); //red
  if (client.available() > 0) {
      char c = client.read();
      dataDecoder(c);
    
  }



    if (prvstartAngle != startAngle) // if there any changes in startAngle, set the current angle to zero and set the set point
    {
      Setpoint = -1 * radToDegree(startAngle);
      //Setpoint = startAngle;
      angle = 0;
      prvstartAngle = startAngle;
    }
    if (!updateGyro()) {
      //Serial.println("Error: Failed to update gyro");
      return;
    }
    Input = (double)angle;
    myPID.Compute();

    //Serial.println(String(star<tAngle) + ", " + String(Setpoint) + "," + String(Input) + ", " + String(Output) + ", ");
   if (abs(Output)<15 && (Output >0))
    {
         Output =Output +10;
    }
    else if (abs(Output)<10 && (Output <0))
    {
          Output =Output -10;
    }

    MoL(-Output);
    MoR(Output);

    if((-turningThresh < startAngle) && (turningThresh > startAngle)) // exit form the loop if the startAngle is bounded in threshold
    {
      //Serial.println("turning done");
      turningDone = true;
    }
    LED(2); //off
  }
  angle = 0;
  MoL(0);
  MoR(0);

}


/*
void emergencyStop() {
  MoL(0);
  MoR(0);
  disableMotors();
  LED(4); // Red LED to indicate emergency stop
}

void moveForward(int speed) {
  MoL(speed);
  MoR(speed);
}


*/
void stopMotors() {
  MoL(0);
  MoR(0);
}
