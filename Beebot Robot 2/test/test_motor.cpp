#include <Arduino.h>

// Motor A pins
#define PWMA 32
#define AIN1 25
#define AIN2 33
// Motor B pins
#define PWMB 13
#define BIN1 27
#define BIN2 14
// Standby pin
#define STBY 26

// PWM settings
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1

void setup() {
  Serial.begin(115200);
  
  // Configure motor control pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  // Configure PWM channels
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CHANNEL_A);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMB, PWM_CHANNEL_B);
  
  // Activate motor driver (disable standby)
  digitalWrite(STBY, HIGH);
  
  Serial.println("Motor test started");
}

// Motor control functions
void motorA_forward(int speed) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWM_CHANNEL_A, speed);
}

void motorA_backward(int speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(PWM_CHANNEL_A, speed);
}

void motorA_stop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
}

void motorB_forward(int speed) {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void motorB_backward(int speed) {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void motorB_stop() {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWM_CHANNEL_B, 0);
}

void loop() {
  Serial.println("Testing Motor A Forward");
  motorA_forward(200);  // Medium speed (200/255)
  delay(2000);
  motorA_stop();
  delay(1000);

  Serial.println("Testing Motor A Backward");
  motorA_backward(200);
  delay(2000);
  motorA_stop();
  delay(1000);

  Serial.println("Testing Motor B Forward");
  motorB_forward(200);
  delay(2000);
  motorB_stop();
  delay(1000);

  Serial.println("Testing Motor B Backward");
  motorB_backward(200);
  delay(2000);
  motorB_stop();
  delay(1000);

  Serial.println("Testing Both Motors Forward");
  motorA_forward(200);
  motorB_forward(200);
  delay(2000);
  
  Serial.println("Stopping Both Motors");
  motorA_stop();
  motorB_stop();
  delay(3000);
}
