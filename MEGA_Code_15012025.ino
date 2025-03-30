// 19165327 Jurgent Gjeloshaj Silverstone Circuit Car Code 
#define Steering_gain 2.0   //Steeting gain value 
#define base_speed 200  //base speed 150/255 (255 max PWM)
#define speed_gain 30  //Speed gain 
#define min_speed 50  //minimum speed 50/255
#define max_speed 255  //maximum speed 255/255

Servo steeringServo; //servo motor variable
float previous_steering_angle = 90.0; //initial servo value 90, straight 
const float wheel_perimeter = PI * wheel_diameter;  // wheel perimeter in cm 

// Waypoints...................................................................
#define No_Waypoints 17
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

//Initial Variables..........................................................
float x_position = 8; //initial x position 0 (in m)
float y_position = 5; //initial y position 0 (in m)
float theta = 0; //initial heading position 0 (in rad)
volatile unsigned int encoderTicks = 0;  // index value to prevent overflow
static int current_waypoint_index = 0; // index for counting waypoints
static int wp_proximity_count = 0;  //this will compare to the hysterises value

void setup() { //setup variables............................................. 
  Wire.begin();  //initiate I2C communication with arduino Nano
  Wire.setClock(400000);  //This values corresponds to fast I2C communication
  Serial.begin(115200); //Serial monitor 
  steeringServo.attach(ServoMotor_pin); //Servo motor variable
  steeringServo.write(90);  // Centre steering value
  send_motor_speed(base_speed); //Motor speed starts with base speed
  lcd.begin(16, 2);  // Set up LCD display, size (16x2)
  //SD Card.........................................................
  pinMode(10, OUTPUT); // Must declare 10 an output and reserve it
  SD.begin(chipSelect); //set up SD card
}

void loop() { // functions that will run in a loop.................................................. 
 //SD card Logging the data............................................
 mySensorData = SD.open("XY.txt", FILE_WRITE);
  if (mySensorData) {
    mySensorData.print("x: ");
    mySensorData.print(x_position);
    mySensorData.print(", y: ");
    mySensorData.println(y_position);
    mySensorData.printl(" Waypoint no.: ");
    mySensorData.println(current_waypoint_index);
    mySensorData.close();
  } else {
    Serial.println("Error opening file for writing");
  }

  static unsigned long lastUpdate = 0; // last update index
  const int updateInterval = 200;  // 200ms per cycle
  if(millis() - lastUpdate >= updateInterval) { //using millies instead of delay function
    lastUpdate = millis();// program can run other tasks while waiting 
    update_encoder_position(); // Get the currnt position
    calculate_odometry(); // Use odomety to update position
  }

static unsigned long lastLCDUpdate = 0; // last lcd update 
const int LCDUpdateInterval = 200; // Update every 200ms
if (millis() - lastLCDUpdate >= LCDUpdateInterval) { //display latest update
    lastLCDUpdate = millis(); //program can run other tasks while waiting 
    lcd.clear();  // Clears old text to prevent overlapping characters
    lcd.setCursor(0, 0); //set cursor at 0:0
    lcd.print("X: "); //print x position on the dispaly
    lcd.print(x_position, 2);  // x position value,  2 decimal places
    lcd.setCursor(0, 1);//set cursor at 0:1
    lcd.print("Y: "); //print y position on the display
    lcd.print(y_position, 2);//y  position value,  2 decimal places
    lcd.setCursor(10, 1);//set cursor at 10:1
    lcd.print("Wp: "); //print Waypoint
    lcd.print(current_waypoint_index); //print the numer of counted waypoint
} 
}

// Arduino Nano variables..........................................
void send_motor_speed(int speed) { //send motor speed to Nano
  Wire.beginTransmission(8);  // Data transmission to Nano, addess 8
  Wire.write(constrain(speed, min_speed, max_speed)); //Speed is constrained between 50 and 100
  Wire.endTransmission(); //end transmission 
}
float wrap_angle(float angle) {//constrain the angle from -pi to pi radian
  return angle - TWO_PI * floor((angle + PI) / TWO_PI); //floor gives a round number
}

void calculate_odometry() {
  //This sections looks at 5 next waypoints and chooses the closest one 
 int search_range_start=current_waypoint_index; //index for start searching
 int search_range_end=min(current_waypoint_index+ 2, No_Waypoints); //index for finish searching in 5 waypoints
 int closest_waypoint=-1;
 /* closest waypoint index, set to -1 so when the program compares to current waypoint index does not return an 
 negative or nonvalid current waypoint index number */
 float closest_distance=1000; // An arbitrarely big number to compare to actual closest distance

for(int i=search_range_start; i< search_range_end; i++){ //start searching
   float target_x=waypoints[i][0]; //start searching for x position
   float target_y=waypoints[i][1]; //start searching for y position
   float distanceToTarget = sqrt(pow(target_x - x_position, 2) + pow(target_y - y_position, 2));
   // Check if this is the closest waypoint so far
    if (distanceToTarget < closest_distance) {
      closest_distance = distanceToTarget;
      closest_waypoint = i;
    }
}
 if(current_waypoint_index >= No_Waypoints) {
    send_motor_speed(0);  // Full stop when reachig final waypoint 
    Serial.println("Circuit Finished!"); //print circuit is finished
    return;
  } 
  if (current_waypoint_index != -1) {
    current_waypoint_index = closest_waypoint;  
  float target_x = waypoints[current_waypoint_index][0]; //look at the x index number after finding the closes wp
  float target_y = waypoints[current_waypoint_index][1]; //look at the y index number after finding the closes wp
  // Calculate target parameters
  float dx = target_x - x_position; //change in x distance from position and target
  float dy = target_y - y_position; //change in y distance from position and target
  float distance = sqrt(dx*dx + dy*dy); //caluclate distance
  float target_heading = atan2(dy, dx); //calcualte target angle or heading 
  float angle_error = wrap_angle(target_heading - theta); //account for angle error 

  // Speed control Porpotional controller....................................................s.
 int motor_speed = constrain( (base_speed + (distance * speed_gain)),  // motor speed constrain as previously defined
  min_speed, //minimum speed 50/255
  max_speed // maximum speed 255/255
);
  send_motor_speed(motor_speed); //send mtor speed to Nano.............................................
  // Steering control (+-30° maximum)
  int steering = 90 + constrain(angle_error * 180/PI * Steering_gain, -30, 30); //steering angle constrain
  steering = constrain(steering, 60, 120); //steering angle between 60 and 120 degrees
  steeringServo.write(steering); //steering value 
  previous_steering_angle = steering;
  // Waypoint progression with hysteresis................................................................
  if(distance < waypoint_threshold) { //if distance smather than the threashhold set
    if(++wp_proximity_count >= waypoint_hysteresis) { //if the hysterises count in the threashhold is equal to 3
      Serial.print("Reached Waypoint"); //serial monitor print 'REashed Waypoint
      Serial.println(current_waypoint_index); //Print the index value/number
      current_waypoint_index++; //increase the index value by 1
      wp_proximity_count = 0; //reset the proximity counter to 0, to repeat the process
    }
  } else {
    wp_proximity_count = 0;
  }

/* Diagnostic output to uncomment when using the Serial Monitor 
 Serial.print("x:"); Serial.print(x_position);
 Serial.print(" y:"); Serial.print(y_position);
 Serial.print(" theta: "); Serial.println(theta);
 */
}
}

void update_encoder_position() { //Update the encoder position.....................
  Wire.requestFrom(8, 2);  // Request 2 bytes from Nano
  if(Wire.available() >= 2) {
    // Read encoder ticks with overflow protection
    unsigned int new_ticks = Wire.read() | (Wire.read() << 8);
    int delta_ticks = new_ticks - encoderTicks;
    if(delta_ticks > 32767) delta_ticks -= 65536; // Handle overflow corresponds to max Arduino store capacity =2^16-1
    if(delta_ticks < -32767) delta_ticks += 65536; // Handle underflow corresponds to max Arduino store capacity =2^16-1
    encoderTicks = new_ticks; 
    // Handle encoder overflow/wrap.........................................
    if(abs(delta_ticks) > 1000) {  //manage encoder ticks overflow 
      encoderTicks = new_ticks;
      delta_ticks = 0;
    } else {
      encoderTicks = new_ticks; //otherwise continue
    }

    // Calculate distance in meters.....................................................
    float distance = (delta_ticks / (float)encoder_teeth) * wheel_perimeter / 100.0;
    x_position += distance * cos(theta); // Updated x position
    y_position += distance * sin(theta); // Updated y position
    // Ackermann steering kinematics calculation.................................................
    float steering_rad = (previous_steering_angle - 90.0) * PI/180.0;
    theta = wrap_angle(theta + (distance / wheelbase) * tan(steering_rad));
  } else {
   //Serial.println("Communication Error"); //If communication fails print this sentance:
    delay(10); //small delay
    Wire.begin();  // Reset I2C bus communication 
    //lcd.begin(16, 2);  // Reinitialize LCD to restore function
    return; //return
  }
}
