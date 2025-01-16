#include <Servo.h>
#include <Wire.h>
#include <math.h>

// Servo Motor Pin
#define SERVO_PIN 10
Servo steeringServo;

// Waypoints
#define NUM_WAYPOINTS 84
float waypoints[NUM_WAYPOINTS][2] = {
  {21.06, 14.08}, {23.12, 16.90}, {25.01, 19.84}, {25.40, 23.17},
  {23.98, 26.45}, {21.02, 27.99}, {18.09, 26.74}, {15.37, 28.68},
  {13.01, 31.24}, {10.82, 34.01}, {8.90, 36.93}, {7.16, 39.90},
  {4.62, 42.15}, {1.41, 42.46}, {-1.43, 40.57}, {-3.46, 37.70},
  {-5.21, 34.67}, {-6.80, 31.55}, {-8.37, 28.42}, {-9.96, 25.30},
  {-11.53, 22.17}, {-13.05, 19.02}, {-14.56, 15.87}, {-16.03, 12.69},
  {-17.63, 9.58}, {-19.77, 6.83}, {-22.46, 4.72}, {-24.73, 2.21},
  {-24.81, -1.03}, {-23.12, -4.06}, {-22.84, -7.33}, {-23.80, -10.67},
  {-23.58, -13.98}, {-22.42, -17.29}, {-21.46, -20.59}, {-21.03, -24.07},
  {-20.80, -27.55}, {-20.46, -31.03}, {-19.96, -34.51}, {-18.98, -37.88},
  {-17.37, -40.96}, {-14.80, -42.93}, {-11.33, -43.38}, {-7.83, -43.09},
  {-4.36, -42.68}, {-0.88, -42.25}, {2.60, -41.87}, {6.03, -41.17},
  {9.22, -39.89}, {12.10, -38.04}, {14.52, -35.54}, {16.28, -32.48},
  {17.76, -29.31}, {17.18, -26.13}, {14.27, -25.51}, {12.69, -28.22},
  {11.95, -31.52}, {8.97, -32.50}, {5.95, -31.11}, {3.17, -28.99},
  {0.55, -26.66}, {-2.06, -24.32}, {-4.69, -22.01}, {-7.34, -19.73},
  {-10.02, -17.48}, {-12.75, -15.29}, {-15.40, -13.02}, {-17.35, -10.22},
  {-17.76, -6.80}, {-16.78, -3.40}, {-14.49, -1.80}, {-13.06, -4.87},
  {-11.25, -7.39}, {-8.18, -5.99}, {-5.70, -3.51}, {-2.91, -1.39},
  {0.24, -0.23}, {3.65, -0.04}, {7.13, -0.23}, {10.31, 0.48},
  {12.89, 2.83}, {14.95, 5.65}, {16.96, 8.52}, {19.01, 11.35},
};

// Current position and heading
float x_pos = 0, y_pos = 0;
float theta = 0;  // Heading in radians

// Current waypoint index
int currentWaypointIndex = 0;

// Wheel encoder ticks received from Nano
volatile int encoderTicks = 0;

// Constants
#define WHEEL_DIAMETER 10.0 // cm
#define WHEEL_CIRCUMFERENCE (3.14 * WHEEL_DIAMETER) // cm
#define TICKS_PER_ROTATION 20 // Number of encoder ticks per full rotation
#define WAYPOINT_THRESHOLD 1.0 // Threshold distance to waypoint in meters

// Timer for non-blocking delays
unsigned long previousMillis = 0;
const long interval = 100;  // Update interval (100ms)

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Initialize Servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(90); // Center the steering

  Serial.println("Mega initialized and ready.");
}

void loop() {
  // Only update at regular intervals (non-blocking)
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Update position from encoders and navigate to waypoint
    updatePositionFromEncoders();
    navigateToClosestWaypoint();
  }
}

// ----------------- Navigation -----------------
void navigateToClosestWaypoint() {
  // Look for the next 5 waypoints and choose the closest one
  int closestWaypointIndex = -1;
  float closestDistance = INFINITY; // Start with a very large number

  // Find the closest waypoint
  for (int i = currentWaypointIndex; i < NUM_WAYPOINTS; i++) {
    float targetX = waypoints[i][0];
    float targetY = waypoints[i][1];

    // Compute distance to this waypoint
    float distanceToTarget = sqrt(pow(targetX - x_pos, 2) + pow(targetY - y_pos, 2));

    // Check if this is the closest waypoint so far
    if (distanceToTarget < closestDistance) {
      closestDistance = distanceToTarget;
      closestWaypointIndex = i;
    }
  }

  // If a closest waypoint is found, move towards it
  if (closestWaypointIndex != -1) {
    currentWaypointIndex = closestWaypointIndex; // Set this waypoint as the target
    float targetX = waypoints[currentWaypointIndex][0];
    float targetY = waypoints[currentWaypointIndex][1];

    // Compute distance to target
    float distanceToTarget = sqrt(pow(targetX - x_pos, 2) + pow(targetY - y_pos, 2));

    // If the waypoint is reached, print a message and move to the next one
    if (distanceToTarget < WAYPOINT_THRESHOLD) {
      Serial.print("Waypoint ");
      Serial.print(currentWaypointIndex + 1);
      Serial.println(" reached!");

      // Debug: Print the next closest waypoint
      if (currentWaypointIndex < NUM_WAYPOINTS - 1) {
        Serial.print("Moving to next closest waypoint: ");
        Serial.print(currentWaypointIndex + 2);
        Serial.print(" at coordinates: ");
        Serial.print(waypoints[currentWaypointIndex + 1][0], 2);
        Serial.print(", ");
        Serial.println(waypoints[currentWaypointIndex + 1][1], 2);
      } else {
        Serial.println("All waypoints reached!");
        return; // All waypoints are completed
      }
    }

    // Compute angle and steering towards the closest waypoint
    float angleToTarget = atan2(targetY - y_pos, targetX - x_pos);
    float angleDiff = angleToTarget - theta;
    angleDiff = normalizeAngle(angleDiff);

    // Map angle difference to the servo's range (60° to 120°)
    int steeringAngle = 90 + (angleDiff * 30); // Adjust the factor as needed
    steeringAngle = constrain(steeringAngle, 60, 120);

    // Dynamically adjust motor speed based on distance to target
    int motorSpeed = map(distanceToTarget, 0, 50, 50, 255); // Example, adjust max speed (255) and min speed (50)

    // Control Servo for Steering
    steeringServo.write(steeringAngle);

    // Send motor speed to Nano
    sendMotorSpeedToNano(motorSpeed);

    // Debug output
    Serial.print("Target Waypoint: ");
    Serial.print(targetX, 2);
    Serial.print(", ");
    Serial.print(targetY, 2);
    Serial.print(" | Distance: ");
    Serial.print(distanceToTarget, 2);
    Serial.print(" | Angle to Target: ");
    Serial.print(angleToTarget, 2);
    Serial.print(" | Steering Angle: ");
    Serial.print(steeringAngle);
    Serial.print(" | Motor Speed: ");
    Serial.println(motorSpeed);
  }
}

// ----------------- Encoder Data -----------------
void updatePositionFromEncoders() {
  Wire.requestFrom(8, 2); // Request 2 bytes (1 integer: ticks)
  if (Wire.available() >= 2) {
    encoderTicks = Wire.read() | (Wire.read() << 8);
  } else {
    // Error handling: No data received from Nano
    Serial.println("Error: Failed to read encoder data from Nano.");
    return;
  }

  // Compute distance traveled
  float distance = (encoderTicks / (float)TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE / 100.0;

  // Update position based on distance and heading
  x_pos += distance * cos(theta);
  y_pos += distance * sin(theta);

  // Update the heading (theta) based on encoder ticks (adjust depending on your system)
  float angleTurned = (encoderTicks / (float)TICKS_PER_ROTATION) * 2 * PI; // Based on the number of encoder ticks
  theta += angleTurned;
  theta = normalizeAngle(theta); // Normalize angle to range -PI to PI

  // Debug output
  Serial.print("Position Updated -> X: ");
  Serial.print(x_pos, 2);
  Serial.print(", Y: ");
  Serial.print(y_pos, 2);
  Serial.print(" | Heading: ");
  Serial.println(theta, 2);
}

// ----------------- Helper Functions -----------------
float normalizeAngle(float angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

void sendMotorSpeedToNano(int speed) {
  Wire.beginTransmission(8);  // Address of the Nano
  Wire.write(speed);          // Send motor speed
  Wire.endTransmission();
}
