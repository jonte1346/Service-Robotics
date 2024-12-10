#include <NewPing.h>
#include "CytronMotorDriver.h"
#include <vector>
#include <utility>



#define MAZE_SIZE          7

// Define the maze as a 2D array
int maze[MAZE_SIZE][MAZE_SIZE] = {
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, },
  {1, 0, 0, 0, 1, 0, 1},
  {1, 0, 1, 0, 1, 0, 1},
  {1, 0, 1, 0, 0, 0, 1},
  {1, 0, 1, 1, 1, 0, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 1, 0, 0, 0, 0, 1},
  {1, 0, 0, 1, 0, 0, 1},
  {1, 1, 1, 1, 1, 1, 1}
};

// Define the graph as an adjacency list
std::vector<std::pair<int, int>> graph[MAZE_SIZE * MAZE_SIZE];


const int PEOPLE_COORDS[7][2] = {{0, 0}, {6, 2}, {6, 4}, {6, 5}, {4, 6}, {1, 5}, {2, 2} } //First one is known, the rest are unknown
const int Start_COORDS[2] = {0, 3};


//States
enum States {INIT, GET_NEXT_GOAL, MOVE_TO_GOAL, LOOK_FOR_PERSON, PICKUP_PERSON, MOVE_TO_BASE, DONE};

// Orientation
enum Orientation {NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3};

States currentState = INIT;
int people_found = 0;
int position[3] = {1, 4, WEST};

void setup() {
  Serial.begin(9600);
  mazeToGraph();
}

void loop() {

  // Execute movement based on the current mode
  switch (currentState) {
    case INIT:
    //Wait for button or start up logic
      break;
    case GET_NEXT_GOAL:
    //Check if there are more people to pick up & get next goal
    //Find shortest path to goal
      break;
    case MOVE_TO_GOAL:
    //Move to goal
      break;
    case LOOK_FOR_PERSON:
    //Look for person at goal
      break;
    case PICKUP_PERSON:
    //Pick up person sequence
      break;
    case MOVE_TO_BASE:
    //Move to start position
      break;
    case DONE:
      break;
  }

  stateTransitions();
}

//State Transitions
void stateTransitions() {
  if (currentState == INIT) {
    if (true) {
      setState(GET_NEXT_GOAL);
    }
  } else if (currentState == GET_NEXT_GOAL) {
    if (/* condition to move to goal */) {
      setState(MOVE_TO_GOAL);
    } 
  } else if (currentState == MOVE_TO_GOAL) {
    if (position[0] == Start_COORDS[0] && position[1] == Start_COORDS[1]) { //Check if at goal
      setState(LOOK_FOR_PERSON);
    }
  } else if (currentState == LOOK_FOR_PERSON) {
    if (/* condition to find person */) {
      setState(PICKUP_PERSON);
    } else {                          //If no person found
      setState(GET_NEXT_GOAL);
    }
  } else if (currentState == PICKUP_PERSON) {
    if (people_found < 2) {
      setState(GET_NEXT_GOAL);
    } else if (people_found == 2) {
      setState(MOVE_TO_BASE);
    }
  } else if (currentState == MOVE_TO_BASE) {
    if (position[0] == Start_COORDS[0] && position[1] == Start_COORDS[1]) {
      setState(DONE);
    }
  } else if (currentState == DONE) {
    // Final state, no transition
  }
}

// Function to change the State
void setState(States state) {
  currentState = state;
}


//Maze solving functions

// Function to convert maze to graph
void mazeToGraph() {
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      if (maze[i][j] == 0) {
        int node = i * MAZE_SIZE + j;
        // Check all four possible directions (up, down, left, right)
        if (i > 0 && maze[i - 1][j] == 0) {
          graph[node].push_back({(i - 1) * MAZE_SIZE + j, 1});
        }
        if (i < MAZE_SIZE - 1 && maze[i + 1][j] == 0) {
          graph[node].push_back({(i + 1) * MAZE_SIZE + j, 1});
        }
        if (j > 0 && maze[i][j - 1] == 0) {
          graph[node].push_back({i * MAZE_SIZE + (j - 1), 1});
        }
        if (j < MAZE_SIZE - 1 && maze[i][j + 1] == 0) {
          graph[node].push_back({i * MAZE_SIZE + (j + 1), 1});
        }
      }
    }
  }
}

void Astar(start[], goal[], maze) {
  //A* algorithm
}


