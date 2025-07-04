#include <NewPing.h>
#include "CytronMotorDriver.h"
//#include <ArduinoSTL.h> // Include the ArduinoSTL library
//#include <utility>      // Include utility header for std::pair
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

// Define a Path struct to represent each direction's attributes
struct Path {
    int neighborID;  // ID of the adjacent node, DEAD_END, or NULL_EDGE
    bool traversed;  // Indicates if the path has been traversed
};

// Define the Node structure
struct Node {
    int id;          // Unique ID for the node
    Path north;      // Path to the north
    Path south;      // Path to the south
    Path east;       // Path to the east
    Path west;       // Path to the west
};

// Array to hold all nodes in the maze
#define NUM_NODES 11  // Total number of intersections in the maze
Node nodes[NUM_NODES];

// Motor configuration
CytronMD motorLeft(PWM_PWM, 3, 5);
CytronMD motorRight(PWM_PWM, 6, 13);

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
  nodes[0] = {0, {NULL_EDGE, false}, {START, true}, {2, false}, {1, false}};  // Start/Exit node
  nodes[1] = {1, {4, false}, {DEAD_END, false}, {0, false}, {3, false}};
  nodes[2] = {2, {NULL_EDGE, false}, {0, false}, {5, false}, {DEAD_END, false}};
  nodes[3] = {3, {DEAD_END, false}, {4, false}, {NULL_EDGE, false}, {1, false}};
  nodes[4] = {4, {3, false}, {9, false}, {NULL_EDGE, false}, {1, false}};
  nodes[5] = {5, {NULL_EDGE, false}, {6, false}, {7, false}, {2, false}};
  nodes[6] = {6, {5, false}, {7, false}, {DEAD_END, false}, {NULL_EDGE, false}};
  nodes[7] = {7, {8, false}, {6, false}, {NULL_EDGE, false}, {5, false}};
  nodes[8] = {8, {DEAD_END, false}, {NULL_EDGE, false}, {7, false}, {9, false}};
  nodes[9] = {9, {10, false}, {NULL_EDGE, false}, {8, false}, {4, false}};
  nodes[10] = {10, {DEAD_END, false}, {9, false}, {NULL_EDGE, false}, {DEAD_END, false}};

  currentNodeID = 0;  // Start at nodes[0]
  currentOrientation = WEST;

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);

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
  if (detectCylinder() && lineType != NONE) {
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
      // Lost the line
      handleNoLine(distanceFront, distanceRight, distanceLeft);
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

  // Choose a path based on DFS
  Serial.print("Visiting Node ");
  Serial.println(currentNodeID);

  setPathTrue(); // set the path we came from to traversed = true
  
    // Explore untraversed paths, regardless of whether the neighbor node is visited
    if (!nodes[currentNodeID].north.traversed && nodes[currentNodeID].north.neighborID != NULL_EDGE) {
        Serial.println("Moving North");
        nodes[currentNodeID].north.traversed = true;  // Mark the path as traversed
        currentNodeID = nodes[currentNodeID].north.neighborID;
        handleTurn(NORTH);
    } else if (!nodes[currentNodeID].south.traversed && nodes[currentNodeID].south.neighborID != NULL_EDGE) {
        Serial.println("Moving South");
        nodes[currentNodeID].south.traversed = true;  // Mark the path as traversed
        currentNodeID = nodes[currentNodeID].south.neighborID;
        handleTurn(SOUTH);
    } else if (!nodes[currentNodeID].east.traversed && nodes[currentNodeID].east.neighborID != NULL_EDGE) {
        Serial.println("Moving East");
        nodes[currentNodeID].east.traversed = true;  // Mark the path as traversed
        currentNodeID = nodes[currentNodeID].east.neighborID;
        handleTurn(EAST);
    } else if (!nodes[currentNodeID].west.traversed && nodes[currentNodeID].west.neighborID != NULL_EDGE) {
        Serial.println("Moving West");
        nodes[currentNodeID].west.traversed = true;  // Mark the path as traversed
        currentNodeID = nodes[currentNodeID].west.neighborID;
        handleTurn(WEST);
    } else {
        // No untraversed paths, backtrack or terminate
        Serial.println("No untraversed paths, stopping traversal.");
    }
}

void setPathTrue(){
  switch(currentOrientation){
    case NORTH:
      nodes[currentNodeID].north.traversed = true;
      break;
    case EAST:
      nodes[currentNodeID].east.traversed = true;
      break;
    case SOUTH:
      nodes[currentNodeID].south.traversed = true;
      break;
    case WEST:
      nodes[currentNodeID].west.traversed = true;
      break;
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

void handleNoLine(int distanceFront, int distanceRight, int distanceLeft){
  for(int i = 0; i < 10; i++){ // NEED TO CALIBRATE THIS, I CHOSE ARBITRARY NUMBER
    moveForward();
    if(distanceFront < 10){
      if(distanceRight < 10){
          turnLeft();
          break;
      } else if(distanceLeft < 10){
         turnRight();
         break;
      }
      handleIntersection();
      break;
    }
  }
}

// void backtrack(std::pair<int, int> previousNode) {
//   // Logic to navigate back to the previous node (e.g., reverse movements)
// }

void exitMaze() {
  // Implement A* for shortest path to the exit
  
}
