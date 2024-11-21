#include <NewPing.h>
#include "CytronMotorDriver.h"
#include <ArduinoSTL.h> // Include the ArduinoSTL library
#include <stack>        // Include stack header
#include <utility>      // Include utility header for std::pair

// Distance sensor pins
#define FRONT_TRIGGER_PIN 12
#define FRONT_ECHO_PIN 11
#define RIGHT_TRIGGER_PIN 8
#define RIGHT_ECHO_PIN 7
#define LEFT_TRIGGER_PIN 4
#define LEFT_ECHO_PIN 2
#define MAX_DISTANCE 200

//Line following sensors
#define LINE_SENSOR_LEFT A0
#define LINE_SENSOR_MIDDLE1 A1
#define LINE_SENSOR_MIDDLE2 A2
#define LINE_SENSOR_MIDDLE3 A3
#define LINE_SENSOR_MIDDLE4 A4
#define LINE_SENSOR_RIGHT A5

//States
enum States {INIT, GET_NEXT_GOAL, MOVE_TO_GOAL, LOOK_FOR_PERSON, PICKUP_PERSON, MOVE_TO_BASE, DONE};

// Orientation
enum Orientation {NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3};

// Movement and navigation modes
enum MovementMode { LINE_FOLLOWING, FORWARD, TURN_LEFT, TURN_RIGHT, TURN_AROUND, WALL_FOLLOWING, STOP };
MovementMode currentMode = LINE_FOLLOWING;

// Motor configuration
CytronMD motorLeft(PWM_PWM, 3, 9);
CytronMD motorRight(PWM_PWM, 10, 11);

// Ultrasonic sensors
NewPing sonarFront(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarRight(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarLeft(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);

// Line-following thresholds
const int LINE_THRESHOLD = 500; // to be changed

// Maze starting state
const int GRID_SIZE = 7;
int grid[GRID_SIZE][GRID_SIZE] = {0};  // 0 = Unvisited, 1 = Visited, 2 = Cylinder
int currentX = 0, currentY = 3;
int orientation = WEST;
std::stack<std::pair<int, int>> dfsStack;
int rescuedCylinders = 0;

// Maze representation
const int maze[15][15] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1},
    {1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1},
    {1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1},
    {1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1},
    {1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
    {1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1},
    {1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1},
    {1, 0, 0, 0, 1, 0, 0, 2, 1, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
};


// Setup
void setup() {
  Serial.begin(9600);
  Serial.println("Robot Initialized");
  grid[currentX][currentY] = 1;  // Mark the starting position as visited
  dfsStack.push({currentX, currentY});  // Start DFS
}

// Main loop
void loop() {
  // Step 1: Read sensors
  int lineLeft = analogRead(LINE_SENSOR_LEFT);
  int lineMiddle = analogRead(LINE_SENSOR_MIDDLE);
  int lineRight = analogRead(LINE_SENSOR_RIGHT);

  int distanceFront = sonarFront.ping_cm();
  int distanceRight = sonarRight.ping_cm();
  int distanceLeft = sonarLeft.ping_cm();

  // Debugging
  Serial.print("Line Sensors - Left: ");
  Serial.print(lineLeft);
  Serial.print(" Middle: ");
  Serial.print(lineMiddle);
  Serial.print(" Right: ");
  Serial.println(lineRight);

  Serial.print("Distances - Front: ");
  Serial.print(distanceFront);
  Serial.print(" Right: ");
  Serial.print(distanceRight);
  Serial.print(" Left: ");
  Serial.println(distanceLeft);

  // Step 2: Cylinder detection and rescue
  if (detectCylinder()) {
    rescueCylinder();
    return;
  }
  
  if (rescuedCylinders == 3) {
    exitMaze();  // Switch to A* for exit
  }

  // Step 3: Line-Following Logic
  if (lineDetected(lineLeft, lineMiddle, lineRight)) {
    currentMode = LINE_FOLLOWING;
    followLine(lineLeft, lineMiddle, lineRight);
    return;  // Skip other logic if the line is detected
  }

  // Step 4: Fallback to DFS if no line detected
  if (!dfsStack.empty()) {
    std::pair<int, int> currentNode = dfsStack.top();
    int x = currentNode.first;
    int y = currentNode.second;

    if (distanceFront > 30 && isUnvisited(x + 1, y)) {
      moveForward();
      currentX++;
      markVisited(currentX, currentY);
      dfsStack.push({currentX, currentY});
    } else if (distanceRight > 30 && isUnvisited(x, y + 1)) {
      turnRight();
      currentY++;
      markVisited(currentX, currentY);
      dfsStack.push({currentX, currentY});
    } else if (distanceLeft > 30 && isUnvisited(x, y - 1)) {
      turnLeft();
      currentY--;
      markVisited(currentX, currentY);
      dfsStack.push({currentX, currentY});
    } else {
      dfsStack.pop();  // Dead end, backtrack
      backtrack(currentNode);
    }
  }  else {
    stopMotors();
  }
}

// Line-following logic
void followLine(int lineLeft, int lineMiddle, int lineRight) {
  if (lineMiddle > LINE_THRESHOLD) {  // Robot is centered
    moveForward();
  } else if (lineLeft > LINE_THRESHOLD) {  // Adjust to the left
    motorLeft.setSpeed(100);
    motorRight.setSpeed(150);
  } else if (lineRight > LINE_THRESHOLD) {  // Adjust to the right
    motorLeft.setSpeed(150);
    motorRight.setSpeed(100);
  } else {
    stopMotors();  // Lost the line
  }
}

bool lineDetected(int lineLeft, int lineMiddle, int lineRight) {
  return (lineLeft > LINE_THRESHOLD || lineMiddle > LINE_THRESHOLD || lineRight > LINE_THRESHOLD);
}

// Wall-following logic
void wallFollow(int distanceRight) {
  if (distanceRight < 20) {  // Too close to wall, move left
    motorLeft.setSpeed(100);
    motorRight.setSpeed(150);
  } else if (distanceRight > 30) {  // Too far from wall, move right
    motorLeft.setSpeed(150);
    motorRight.setSpeed(100);
  } else {  // Perfect distance, move forward
    moveForward();
  }
}

// Rescue logic
bool detectCylinder() {
  int objectDistance = sonarFront.ping_cm();
  return (objectDistance > 0 && objectDistance < 10);
}

void rescueCylinder() {
  stopMotors();
  delay(1000);
  grid[currentX][currentY] = 2;
  rescuedCylinders++;
  Serial.print("Cylinder rescued! Total rescued: ");
  Serial.println(rescuedCylinders);
}

bool isUnvisited(int x, int y) {
  return (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE && grid[x][y] == 0);
}

void markVisited(int x, int y) {
  grid[x][y] = 1;
}

// Movement functions
void moveForward() {
  motorLeft.setSpeed(150);
  motorRight.setSpeed(150);
}

void turnLeft() {
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  delay(500);
}

void turnRight() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  delay(500);
}

void turnAround() {
  motorLeft.setSpeed(-150);
  motorRight.setSpeed(150);
  delay(1000);
}

void stopMotors() {
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}

void backtrack(std::pair<int, int> previousNode) {
  // Logic to navigate back to the previous node (e.g., reverse movements)
}

void exitMaze() {
  // Implement A* for shortest path to the exit
  //std::priority_queue<std::pair<int, std::pair<int, int>>> pq;  // Min-heap for A*
  //pq.push({0, {currentX, currentY}});

  // Example A* algorithm implementation
  //while (!pq.empty()) {
    // A* logic here
  //}
}
