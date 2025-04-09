//19165327_Jurgent_Gjeloshaj Processing Code 
//define all waypoints the car needs to follow
float[][] waypoints = {
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

float car_x = 8.42;  //car x start position, this value correlates to first waypoint 
float car_y = 5.63; //car y start position 
float car_angle = 0; // angle theta is equal to 0, 
int currentWaypoint = 0; // start from 0, waypoints from 0 to 84 
float speed = 0.5; //car speed
float scaleFactor; //screen visualisation scale
float x_offset, y_offset; //circuit is placed in the centre of the circuit.

void setup() {
  size(1000, 1000); //screen size 
  calculateBoundsAndScale();
  background(25); //background
}

void draw() {
  background(255); //background color 
  translate(x_offset, y_offset); // x and y offest values 
  scale(scaleFactor); // Window scale factor 
  plotWaypoints(); // waypoins 
  drawPath(); // path between waypoints 
  moveToWaypoint(); // move to waypoint varuable
  drawCar(car_x, car_y, car_angle); // set x, y and angle variables
}

void calculateBoundsAndScale() {
  float minX = Float.MAX_VALUE, minY = Float.MAX_VALUE; // minimum x and y point floating value 
  //waypoints x and y coordinate will be bigger than this initiall value
  float maxX = Float.MIN_VALUE, maxY = Float.MIN_VALUE; //maximum x and y point floating value 
     //waypoints x and y coordinate will be lower than this initiall value

//find min and max x and y values is important when calculating consecutive waypoint values
  for (int i = 0; i < waypoints.length; i++) {
    if (waypoints[i][0] < minX) minX = waypoints[i][0]; //   min x values
    if (waypoints[i][1] < minY) minY = waypoints[i][1]; // min y value
    if (waypoints[i][0] > maxX) maxX = waypoints[i][0]; //max x value
    if (waypoints[i][1] > maxY) maxY = waypoints[i][1]; //max y value
  }

  float circuit_width = maxX - minX;  // the width of the circuit 
  float circuit_height = maxY - minY; // the height
  scaleFactor = 0.8 * min(width / circuit_width, height / circuit_height); // Leave padding
  x_offset = width / 2 - (minX + maxX) / 2 * scaleFactor;
  y_offset = height / 2 - (minY + maxY) / 2 * scaleFactor;
}

void plotWaypoints() {  //waypoint features such as filling 
  fill(0);
  stroke(0);
  for (int i = 0; i < waypoints.length; i++) {
    ellipse(waypoints[i][0], waypoints[i][1], 1, 1); 
  }
}

void drawPath() {     //path from one waypoint to the other
  strokeWeight(0.2); //path physical features 
  stroke(150);
  for (int i = 0; i < waypoints.length - 1; i++) { //this waypoint index will tell what waypoint numnber is being looked at
    line(waypoints[i][0], waypoints[i][1],
         waypoints[i + 1][0], waypoints[i + 1][1]);
  }
}

float carR = 0, carG = 255, carB = 0;  //set colors to, read green and blue
void drawCar(float x, float y, float angle) { //car will move through waypoints
  pushMatrix(); //save the cars state at the moment
  translate(x, y); //cars origine will move to match the new coordinates continuesly
  rotate(angle); //rotate to match the next waypoint 
  fill(carR, carG, carB);  // Use dynamic values for coloring 
  rect(-1, -1, 2, 1); //car's dimensions, square
  popMatrix();
}

void moveToWaypoint() { //move towards the waypoint
  if (currentWaypoint >= waypoints.length) {
    speed = 0; //
    return;
  }

  float target_x = waypoints[currentWaypoint][0]; // waypoint target x
  float target_y = waypoints[currentWaypoint][1]; // waypoint target y 
  float angleToTarget = atan2(target_y - car_y, target_x - car_x); //using Equation 2 to calculate angle theta
  float distance = dist(car_x, car_y, target_x, target_y); //distance to target using Equation  1
  float angleDiff = angleToTarget - car_angle; //Angle difference using Equation 3 
  while (angleDiff > PI) angleDiff -= TWO_PI;  //steering constrains between pi and -pi
  while (angleDiff < -PI) angleDiff += TWO_PI; //steering constrains between pi and -pi

  car_angle += 0.1 * angleDiff;
  if (abs(angleDiff) < 0.1) {
    car_x += cos(car_angle) * speed;  //delta x position Equation 4a on the report 
    car_y += sin(car_angle) * speed;  //delta y position Equation 4b on the report 
  }

  if (distance < 3) {
    currentWaypoint++;
  }
}

//Some refernece links for this code
//https://docs.oracle.com/javase/tutorial/2d/geometry/strokeandfill.html#:~:text=Filling%20%E2%80%93%20is%20a%20process%20of,line%20style%2C%20and%20color%20attribute
//https://processing.org/
