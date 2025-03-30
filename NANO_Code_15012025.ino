#include <Wire.h> //I2C library bus communication with Mega
#define Motor_pin 9 //define dc motor pin =9
#define Encoder_pin 2 //define encoder pin =2
// Encoder value changes rapifly, hence a volatile function must be implemented
volatile unsigned int encoderTicks = 0;  //0-65535 range or 2^16-1
int motor_speed = 0; //Motor speed value index

void setup() { //Set up the variables.....................................  
  Serial.begin(115200); //Serial Monitor for debugging
  pinMode(Motor_pin, OUTPUT); //Set motor pin as output
  pinMode(Encoder_pin, INPUT_PULLUP); //Set up encoder sesor
  // Encoder interrupt setup..............................................
  attachInterrupt(digitalPinToInterrupt(Encoder_pin), encoderISR, RISING);
  Wire.begin(8); // I2C setup as slave at address 8        
  Wire.onReceive(receiveCommand); //Receive command from Mega
  Wire.onRequest(send_encoder_data); //Send encoder data to Mega
  Serial.println("Nano ready"); //Print 'Nano ready' 
}

void loop() { // This piece of code will run continiously..........................
  analogWrite(Motor_pin, motor_speed); //Write the speed of the motor based on PWM received
  delay(10); // Reduced delay for better responsiveness
}
// Optimised ISR, no floating points
void encoderISR() { //the program will allways read the encoders last value
  encoderTicks++;  // Will finish at 65535 as set on the volatile int
  
}
// Motor command handler...................................................
void receiveCommand(int numBytes) {
  if(Wire.available()) { //if a byte is sent by mega then:
    motor_speed = Wire.read(); //read motor speed sent by Mega
    motor_speed = constrain(motor_speed, 0, 255); //speed constrain
    Serial.print("Speed: "); //print 'Speed' on the Serial Monitor
    Serial.println(motor_speed); //print the value 
  }
}
// Encoder send the data, 16-bit split into 2 times 8 bytes
void send_encoder_data() {
  Wire.write(lowByte(encoderTicks));// Send LSB first 
  Wire.write(highByte(encoderTicks)); //Send MSB second
  static unsigned int lastSent; //Comands for debugging
  Serial.print("Delta Ticks:"); //Print this text
  Serial.println(encoderTicks - lastSent); //Print the change in the encoder ticks
  lastSent = encoderTicks;
}

