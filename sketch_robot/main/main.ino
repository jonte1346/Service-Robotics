#include <NewPing.h>
#include "CytronMotorDriver.h"
#include <ArduinoSTL.h> // Include the ArduinoSTL library
#include <stack>        // Include stack header
#include <utility>      // Include utility header for std::pair
#include <QTRSensors.h>

// Distance sensor pins
#define FRONT_TRIGGER_PIN 12
#define FRONT_ECHO_PIN 11
#define RIGHT_TRIGGER_PIN 8
#define RIGHT_ECHO_PIN 7
#define LEFT_TRIGGER_PIN 4
#define LEFT_ECHO_PIN 2
#define MAX_DISTANCE 200

QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// Line-following thresholds
const uint16_t LINE_THRESHOLD = 600; // Adjust based on calibration
const uint16_t CENTER_POSITION = 2500;

//States
enum States {INIT, GET_NEXT_GOAL, MOVE_TO_GOAL, LOOK_FOR_PERSON, PICKUP_PERSON, MOVE_TO_BASE, DONE};

// Orientation
enum Orientation {NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3};

// Movement and navigation modes
enum MovementMode { LINE_FOLLOWING, FORWARD, TURN_LEFT, TURN_RIGHT, TURN_AROUND, WALL_FOLLOWING, STOP };
MovementMode currentMode = LINE_FOLLOWING;

enum LineType { STRAIGHT, LEFT_TURN, RIGHT_TURN, INTERSECTION, NONE };

// Define constants for edge states
#define NULL_EDGE -1  // No edge in this direction
#define DEAD_END -2   // Dead end in this direction
#define START -3

// Define a structure for a Node
struct Node {
    int id;          // Unique ID for the node
    bool visited;    // check to see if a node is visited or not
    int north;       // ID of the node to the north, DEAD_END, or NULL_EDGE
    int south;       // ID of the node to the south, DEAD_END, or NULL_EDGE
    int east;        // ID of the node to the east, DEAD_END, or NULL_EDGE
    int west;        // ID of the node to the west, DEAD_END, or NULL_EDGE
};

// Array to hold all nodes in the maze
#define NUM_NODES 11  // Total number of intersections in the maze
Node nodes[NUM_NODES];

// Motor configuration
CytronMD motorLeft(PWM_PWM, 3, 9);
CytronMD motorRight(PWM_PWM, 10, 11);

// Ultrasonic sensors
NewPing sonarFront(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarRight(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarLeft(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);

// Maze starting state
int orientation = WEST;
int rescuedCylinders = 0;
int currentNodeID;
Orientation currentOrientation;


// Setup
void setup() {
  //id, north, south, east, west
// Define nodes and their connections (IDs, DEAD_END, or NULL_EDGE)
  nodes[0] = {0, false, NULL_EDGE, START, 2, 1};  // Start/Exit node
  nodes[1] = {1, false, 4, DEAD_END, 0, 3};
  nodes[2] = {2, false, NULL_EDGE, 0, 5, DEAD_END};
  nodes[3] = {3, false, DEAD_END, 4, NULL_EDGE, 1};
  nodes[4] = {4, false, 3, 9, NULL_EDGE, 1};
  nodes[5] = {5, false, NULL_EDGE, 6, 7, 2};
  nodes[6] = {6, false, 5, 7, DEAD_END, NULL_EDGE};
  nodes[7] = {7, false, 8, 6, NULL_EDGE, 5};
  nodes[8] = {8, false, DEAD_END, NULL_EDGE, 7, 9};
  nodes[9] = {9, false, 10, NULL_EDGE, 8, 4};
  nodes[10] = {10, false, DEAD_END, 9, NULL_EDGE, DEAD_END};

  currentNodeID = 0;  // Start at nodes[0]
  currentOrientation = WEST;

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4}, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  Serial.begin(9600);
  Serial.println("Robot Initialized");

}

// Main loop
void loop() {
  // Step 1: Read QTR sensors
  uint16_t position = qtr.readLineBlack(sensorValues); // 0 for sensor 0, 1000 for sensor 1, 2000 for sensor 2 etc.
  LineType lineType = detectLineType(sensorValues, LINE_THRESHOLD);

  int distanceFront = sonarFront.ping_cm();
  int distanceRight = sonarRight.ping_cm();
  int distanceLeft = sonarLeft.ping_cm();

  // Debugging
  Serial.print("QTR Position: ");
  Serial.println(position);
  Serial.print("Sensor values: ");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  Serial.print("Distances - Front: ");
  Serial.println(distanceFront);
  Serial.print(" Right: ");
  Serial.println(distanceRight);
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

  // Step 3: Handle Line Type
  switch (lineType) {
    case STRAIGHT:
      followLine(position); // Continue line-following
      break;

    case LEFT_TURN:
      turnLeft(); // Perform left turn
      break;

    case RIGHT_TURN:
      turnRight(); // Perform right turn
      break;

    case INTERSECTION:
      handleIntersection(); // Use DFS to decide the path
      break;

    case NONE:
      stopMotors(); // Lost the line
      break;
  }

}



LineType detectLineType(uint16_t *sensorValues, uint16_t threshold) {
  int activeSensors = 0;        // Count of sensors detecting the line
  int firstActive = -1;         // Index of the first sensor detecting the line
  int lastActive = -1;          // Index of the last sensor detecting the line

  // Analyze sensor values to find active sensors
  for (uint8_t i = 0; i < 6; i++) { // Assuming 6 sensors
    if (sensorValues[i] > threshold) {
      activeSensors++;
      if (firstActive == -1) {
        firstActive = i; // Mark the first active sensor
      }
      lastActive = i; // Continuously update the last active sensor
    }
  }

  // Determine line type
  if (activeSensors == 0) {
    return NONE; // No line detected
  } else if (activeSensors <= 3) { // Straight line case (at most 3 sensors)
    return STRAIGHT;
  } else if (activeSensors >= 4) { // At least 4 sensors detecting the line
    if (firstActive <= 2 && lastActive <= 3) {
      return LEFT_TURN; // Line on the left (left turn)
    } else if (firstActive >= 2 && lastActive >= 3) {
      return RIGHT_TURN; // Line on the right (right turn)
    } else {
      return INTERSECTION; // Line spans across center and edges
    }
  }

  return NONE; // Default case (shouldn't happen)
}

// Line-following logic
void followLine(uint16_t position) {
  int error = position - CENTER_POSITION; // Calculate error relative to the center
  int turnSpeed = error / 10; // Adjust motor speed based on error

  // Adjust motors to stay on the line
  motorLeft.setSpeed(150 - turnSpeed);
  motorRight.setSpeed(150 + turnSpeed);
}

void handleIntersection() {
  stopMotors();
  delay(500);

  // Choose a path based on DFS
  Serial.print("Visiting Node ");
        Serial.println(currentNodeID);

        // Mark the current node as visited
        nodes[currentNodeID].visited = true;

        // Check adjacent nodes
        if (isUnvisited(nodes[currentNodeID].north)) {
            Serial.println("Moving North");
            if (nodes[currentNodeID].north == DEAD_END){
              nodes[currentNodeID].north = NULL_EDGE;
            } else {
            currentNodeID = nodes[currentNodeID].north;
            }
            handleTurn(NORTH);
        } else if (isUnvisited(nodes[currentNodeID].south)) {
            Serial.println("Moving South");
            if (nodes[currentNodeID].south == DEAD_END){
              nodes[currentNodeID].south = NULL_EDGE;
            } else {
            currentNodeID = nodes[currentNodeID].south;
            }
            handleTurn(SOUTH);
        } else if (isUnvisited(nodes[currentNodeID].east)) {
            Serial.println("Moving East");
            if (nodes[currentNodeID].east == DEAD_END){
              nodes[currentNodeID].east = NULL_EDGE;
            } else {
            currentNodeID = nodes[currentNodeID].east;
            }
            handleTurn(EAST);
        } else if (isUnvisited(nodes[currentNodeID].west)) {
            Serial.println("Moving West");
            if (nodes[currentNodeID].west == DEAD_END){
              nodes[currentNodeID].west = NULL_EDGE;
            } else {
            currentNodeID = nodes[currentNodeID].west;
            }
            handleTurn(WEST);
        } else {
            // No unvisited neighbors, backtrack or terminate
            Serial.println("No unvisited neighbors, stopping traversal.");
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
  rescuedCylinders++;
  Serial.print("Cylinder rescued! Total rescued: ");
  Serial.println(rescuedCylinders);
}

// Helper function to check if a node is unvisited
bool isUnvisited(int nodeID) {
    if (nodeID == NULL_EDGE) return false;  // Skip invalid edges
    if (nodeID == DEAD_END) return true; // check out dead ends and then call them NULL_EDGE
    return !nodes[nodeID].visited;  // Return true if the node is unvisited
}

// Movement functions
void moveForward() {
  motorLeft.setSpeed(150);
  motorRight.setSpeed(150);
}

void turnLeft() {
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(100);
  changeOrientation(true);
  delay(500);
}

void turnRight() {
  motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  changeOrientation(false);
  delay(500);
}

void turnAround() {
  motorLeft.setSpeed(-150);
  motorRight.setSpeed(150);
  changeOrientation(true);
  changeOrientation(true);
  delay(1000);
}

void stopMotors() {
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}

void changeOrientation(bool turnLeft) {
    if (turnLeft) {
        // Turn left: Decrement, wrap around if less than 0
        currentOrientation = static_cast<Orientation>((currentOrientation + 3) % 4);
    } else {
        // Turn right: Increment, wrap around if greater than 3
        currentOrientation = static_cast<Orientation>((currentOrientation + 1) % 4);
    }
}

// Function to handle turning to a new orientation
void handleTurn(Orientation newOrientation) {
    int difference = (newOrientation - currentOrientation + 4) % 4;

    // Determine the turn direction
    if (difference == 0) {
        Serial.println("No turn needed, already facing the correct direction.");
        moveForward();
    } else if (difference == 1) {
        Serial.println("Turn RIGHT.");
        turnRight();
    } else if (difference == 2) {
        Serial.println("Turn 180 degrees.");
        turnAround();
    } else if (difference == 3) {
        Serial.println("Turn LEFT.");
        turnLeft();
    }
}

void backtrack(std::pair<int, int> previousNode) {
  // Logic to navigate back to the previous node (e.g., reverse movements)
}

void exitMaze() {
  // Implement A* for shortest path to the exit
  
}
