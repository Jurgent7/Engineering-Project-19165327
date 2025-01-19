//19165327 Jutgent Gjeloshaj 2025
#include <Servo.h>  // Import servo motor library
#include <Wire.h>   // I2C bus library, allows data transfer with Nano
#include <math.h>   // Math library allows math operations
#include <SPI.h>  // SPI communication library 
#include <SD.h> // SD card set up  Library

int chipSelect = 4;   //set up chipselect 4
File WaypointData; //variable for working with file object
#define ServoMotor_pin 8  // Servo motor pin 10
Servo steeringServo; // set up variable steering Servo 

#define No_Waypoints 84  // Define all 84 waypoints
float waypoints[No_Waypoints][2] = {
  {8.42, 5.63}, {9.25, 6.76}, {10.00, 7.93}, {10.16, 9.27}, {9.59, 10.58},  
  {8.41, 11.20}, {7.23, 10.70}, {6.15, 11.47}, {5.20, 12.50}, {4.33, 13.60},  
  {3.56, 14.77}, {2.86, 15.96}, {1.85, 16.86}, {0.56, 16.99}, {-0.57, 16.23},  
  {-1.38, 15.08}, {-2.08, 13.87}, {-2.72, 12.62}, {-3.35, 11.37}, {-3.98, 10.12},  
  {-4.61, 8.87}, {-5.22, 7.61}, {-5.83, 6.35}, {-6.41, 5.08}, {-7.05, 3.83},  
  {-7.91, 2.73}, {-8.98, 1.89}, {-9.89, 0.88}, {-9.92, -0.41}, {-9.25, -1.63},  
  {-9.14, -2.93}, {-9.52, -4.27}, {-9.43, -5.59}, {-8.97, -6.92}, {-8.58, -8.24},  
  {-8.41, -9.63}, {-8.32, -11.02}, {-8.18, -12.41}, {-7.99, -13.80}, {-7.59, -15.15},  
  {-6.95, -16.38}, {-5.92, -17.17}, {-4.53, -17.35}, {-3.13, -17.24}, {-1.74, -17.07},  
  {-0.35, -16.90}, {1.04, -16.75}, {2.41, -16.47}, {3.69, -15.96}, {4.84, -15.22},  
  {5.81, -14.21}, {6.51, -12.99}, {7.10, -11.72}, {6.87, -10.45}, {5.71, -10.21},  
  {5.08, -11.29}, {4.78, -12.61}, {3.59, -13.00}, {2.38, -12.44}, {1.27, -11.60},  
  {0.22, -10.66}, {-0.82, -9.73}, {-1.88, -8.81}, {-2.93, -7.89}, {-4.01, -6.99},  
  {-5.10, -6.12}, {-6.16, -5.21}, {-6.94, -4.09}, {-7.11, -2.72}, {-6.71, -1.36},  
  {-5.80, -0.72}, {-5.23, -1.95}, {-4.50, -2.96}, {-3.27, -2.39}, {-2.28, -1.40},  
  {-1.16, -0.56}, {0.10, -0.09}, {1.46, -0.01}, {2.85, -0.09}, {4.12, 0.19},  
  {5.15, 1.13}, {5.98, 2.26}, {6.78, 3.41}, {7.61, 4.54}

};
// Current position and heading
float x_pos = 8.42, y_pos = 5.63;
float theta = 0;  // Heading in radians
// Current waypoint index
int currentWaypointIndex = 0;
// Wheel encoder ticks received from Nano
volatile int encoderTicks = 0;

// Constants
#define Wheel_diameter 10.0                          // car's wheel diameter in cm
#define wheel_circumference (3.14 * Wheel_diameter)  // car's wheel circumference in cm 
#define ticks_rotation 20                            // Number of encoder ticks per full rotation
#define waypoint_threshold 1.0                       // Threshold distance to waypoint in meters

// Timer for non-blocking delays
unsigned long previousMillis = 0;
const long interval = 100;  // Update interval (100ms)

void setup() {
  Wire.begin();
  Serial.begin(9600); //serial monitor 
  
  pinMode(10, OUTPUT);
  SD.begin(chipSelect);
  while (!Serial) {
    delay(10);
  }

  // Initialize Servo
  steeringServo.attach(ServoMotor_pin);
  steeringServo.write(90);  // Center the steering
  Serial.println("Mega initialized and ready.");
}

// navigation -----------------
void navigateToClosestWaypoint() {
  // Define the search range for the next 5 waypoints
  int searchRangeStart = currentWaypointIndex;
  int searchRangeEnd = min(currentWaypointIndex + 5, No_Waypoints);  // Make sure array size does not exceed 5,
  // Only next 5 waypoints will be considered.
  Serial.print(currentWaypointIndex);

  int closestWaypointIndex = -1;
  float closestDistance = 1000;  // Start with a very large number to compare the distance
  //The program will compare first waypoint distance value to this number and then to the next waypioint distance

  // Find the closest waypoint within the next 5
  for (int i = searchRangeStart; i < searchRangeEnd; i++) {
    float targetX = waypoints[i][0];
    float targetY = waypoints[i][1];

    // Compute distance to this waypoint using Equation 1
    float distanceToTarget = sqrt(pow(targetX - x_pos, 2) + pow(targetY - y_pos, 2));

    // Check if this is the closest waypoint so far
    if (distanceToTarget < closestDistance) {
      closestDistance = distanceToTarget;
      closestWaypointIndex = i;
    }
  }

  // If a closest waypoint is found, move towards it
  if (closestWaypointIndex != -1) {
    currentWaypointIndex = closestWaypointIndex;  // Set this waypoint as the target
    float targetX = waypoints[currentWaypointIndex][0];
    float targetY = waypoints[currentWaypointIndex][1];

    // Compute distance and angle to target, using Equations 1 and 2 from report
    float distanceToTarget = sqrt(pow(targetX - x_pos, 2) + pow(targetY - y_pos, 2));
    float angleToTarget = atan2(targetY - y_pos, targetX - x_pos);
    float angleDiff = angleToTarget - theta;
    angleDiff = normalizeAngle(angleDiff);

    // If the waypoint is reached, print a message and update
    if (distanceToTarget < waypoint_threshold) {
      Serial.print("Waypoint ");
      Serial.print(currentWaypointIndex + 1);
      Serial.println(" reached!");
      return;  // Exit function to wait for next update
    }

    // Map angle difference to the servo's range (60° to 120°)
    int steeringAngle = 90 + (angleDiff * 30);  // Adjust the factor as needed
    steeringAngle = constrain(steeringAngle, 60, 120);

    // Dynamically adjust motor speed based on distance to target
    int motorSpeed = map(distanceToTarget, 0, 50, 50, 255);  // max speed (255) and min speed (50)
    // Control Servo for Steering
    steeringServo.write(steeringAngle);
    // Send motor speed to Nano
    sendMotorSpeedToNano(motorSpeed);

    // Display target waypoint, distamce, angle and other features on serial monitor
    Serial.print("Target Waypoint: ");
    Serial.print(targetX, 2);
    Serial.print(", ");
    Serial.print(targetY, 2);
    Serial.print(" / Distance: ");
    Serial.print(distanceToTarget, 2);
    Serial.print(" / Angle to Target: ");
    Serial.print(angleToTarget, 2);
    Serial.print(" / Steering Angle: ");
    Serial.print(steeringAngle);
    Serial.print(" / Motor Speed: ");
    Serial.println(motorSpeed);
  } else {
    Serial.println("No waypoint found in the range!");
  }
}

// Encoder Data -----------------
void updatePositionFromEncoders() {
  Wire.requestFrom(8, 2);  // Request 2 bytes (1 integer: ticks)
  if (Wire.available() >= 2) {
    encoderTicks = Wire.read() | (Wire.read() << 8);
  } else {
    // Error handling: No data received from Nano
    Serial.println("Error: Failed to read encoder data from Nano.");
    return;
  }

  // Compute distance traveled
  float distance = (encoderTicks / (float)ticks_rotation) * wheel_circumference / 100.0;

  // Update position based on distance and heading
  x_pos += distance * cos(theta);
  y_pos += distance * sin(theta);

  // Update the heading (theta) based on encoder ticks (adjust depending on your system)
  float angleTurned = (encoderTicks / (float)ticks_rotation) * 2 * PI;  // Based on the number of encoder ticks
  theta += angleTurned;
  theta = normalizeAngle(theta);  // Normalize angle to range -PI to PI

  // Debug output
  Serial.print("Position Updated -> X: ");
  Serial.print(x_pos, 2);
  Serial.print(", Y: ");
  Serial.print(y_pos, 2);
  Serial.print(" - Heading: ");
  Serial.println(theta, 2);
}

// Keeping steering under managable values
float normalizeAngle(float angle) {  // make sure the angle does not exceed -pi and + pi
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

void sendMotorSpeedToNano(int speed) {
  Wire.beginTransmission(8);  // Address of the Nano
  Wire.write(speed);          // Send dc motor  speed
  Wire.endTransmission();     //close transmission
}
void loop() {

  WaypointData = SD.open("Car.txt", FILE_WRITE);  //open data and write data in it
  // Only update at regular intervals (non-blocking)

  if (WaypointData) {
    WaypointData.print("Target Waypoint: ");
    WaypointData.print(x_pos, 2);
    WaypointData.print(", ");
    WaypointData.print(y_pos, 2);
    WaypointData.print(" / Distance: ");
    WaypointData.print(currentWaypointIndex, 2);
    WaypointData.print(" / Heading: ");
    WaypointData.println(theta, 2);
    WaypointData.close();  // Always close the file after writing
  } 
  else {
    Serial.println("Error: Unable to open SD file.");
  }


   unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Update position from encoders and navigate to waypoint
    updatePositionFromEncoders();
    navigateToClosestWaypoint();
  }
}
