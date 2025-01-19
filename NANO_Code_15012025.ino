#include <Wire.h> //I2c bus communication library 
#define MOTOR_PIN 9    // Motor digital pin 9
#define ENCODER_PIN 2  // Encoder pin 2

// Encoder variables
volatile int encoderTicks = 0;
int motorSpeed = 0; // Motor speed received from the Mega

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Initialize I2C communication as Slave
  Wire.begin(8); // Address 8 for Nano
  Wire.onReceive(receiveCommand); // Register receive handler
  Wire.onRequest(sendEncoderData); // Register request handler
  
  pinMode(MOTOR_PIN, OUTPUT); // Motor control pin setup
  pinMode(ENCODER_PIN, INPUT_PULLUP);  // Encoder pin setup
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
  Serial.println("Nano initialized and ready."); //serial monitor to help with deugging
}

void loop() {
  // Drive the motor with the received speed
  analogWrite(MOTOR_PIN, motorSpeed); // Control motor speed based on Mega's command
  delay(50); // Small delay to stabilize motor
}

// ----------------- I2C Communication -----------------
void receiveCommand(int numBytes) {
  // Receive motor speed from the Mega
  if (Wire.available()) {
    motorSpeed = Wire.read(); // Read the speed value
    Serial.print("Received motor speed: ");
    Serial.println(motorSpeed); // Debug: Print received motor speed
  }
}

void sendEncoderData() {
  // Send encoder tick count to Mega
  Wire.write(lowByte(encoderTicks));
  Wire.write(highByte(encoderTicks));

  // Debug message
  Serial.print("Sent Encoder Data -> Ticks: ");
  Serial.println(encoderTicks); // Debug: Print sent encoder data
}

// ----------------- Encoder Interrupt -----------------
void encoderISR() {
  encoderTicks++; // Increment encoder tick count
}

