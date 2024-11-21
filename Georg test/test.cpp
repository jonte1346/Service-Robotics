#include <queue>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

// Maze dimensions
const int rows = 15, cols = 15;

    // A structure to hold the necessary parameters for A* algorithm
    struct Node {
        int row, col;
        int g, h, f;
        Node* parent;

        Node(int r, int c, int g_val, int h_val, Node* p = nullptr)
            : row(r), col(c), g(g_val), h(h_val), f(g_val + h_val), parent(p) {}

        bool operator>(const Node& other) const {
            return f > other.f;
        }
    };

// Maze representation
const int maze[15][15] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 1, 3, 1, 0, 0, 3, 1, 3, 0, 0, 1},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1},
    {1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1},
    {1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 3, 1},
    {1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1},
    {1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
    {1, 0, 1, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 3, 1, 0, 1},
    {1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1},
    {1, 2, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
};

// Graph representation (Adjacency List)
vector<vector<int>> graph(rows * cols);

// Convert (row, col) to a single index
int getIndex(int row, int col) {
    return row * cols + col;
}

// Add an edge between two nodes
void addEdge(int u, int v) {
    graph[u].push_back(v);
    //graph[v].push_back(u); // Since it's undirected
}

// Build the graph from the maze
void buildGraph() {
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            if (maze[row][col] == 1) continue; // Skip walls

            int u = getIndex(row, col);

            // Check possible neighbors (up, down, left, right)
            if (row > 0 && maze[row - 1][col] != 1) { // Up
                int v = getIndex(row - 1, col);
                addEdge(u, v);
            }
            if (row < rows - 1 && maze[row + 1][col] != 1) { // Down
                int v = getIndex(row + 1, col);
                addEdge(u, v);
            }
            if (col > 0 && maze[row][col - 1] != 1) { // Left
                int v = getIndex(row, col - 1);
                addEdge(u, v);
            }
            if (col < cols - 1 && maze[row][col + 1] != 1) { // Right
                int v = getIndex(row, col + 1);
                addEdge(u, v);
            }
        }
    }
}

// Print the adjacency list for the graph
void printGraph() {
    for (int i = 0; i < rows * cols; i++) {
        if (!graph[i].empty()) {
            cout << "Node " << i << ": ";
            for (int neighbor : graph[i]) {
                cout << neighbor << " ";
            }
            cout << endl;
        }
    }
}



    // Heuristic function: Manhattan distance
    int heuristic(int row, int col, int goalRow, int goalCol) {
        return abs(row - goalRow) + abs(col - goalCol);
    }

    // A* algorithm implementation
    void Astar(int start[], int goal[], vector<vector<int>> maze) {
        priority_queue<Node, vector<Node>, greater<Node>> openList;
        unordered_map<int, Node*> allNodes;

        int startRow = start[0], startCol = start[1];
        int goalRow = goal[0], goalCol = goal[1];

        Node* startNode = new Node(startRow, startCol, 0, heuristic(startRow, startCol, goalRow, goalCol));
        openList.push(*startNode);
        allNodes[getIndex(startRow, startCol)] = startNode;

        while (!openList.empty()) {
            Node current = openList.top();
            openList.pop();

            if (current.row == goalRow && current.col == goalCol) {
                // Reconstruct path
                vector<pair<int, int>> path;
                Node* node = &current;
                while (node) {
                    path.push_back({node->row, node->col});
                    node = node->parent;
                }
                //reverse(path.begin(), path.end());

                cout << "Path found: ";
                for (auto p : path) {
                    cout << "(" << p.first << ", " << p.second << ") ";
                }
                cout << endl;
                return;
            }

            // Explore neighbors
            vector<pair<int, int>> neighbors = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
            for (auto& neighbor : neighbors) {
                int newRow = current.row + neighbor.first;
                int newCol = current.col + neighbor.second;

                if (newRow >= 0 && newRow < rows && newCol >= 0 && newCol < cols && maze[newRow][newCol] != 1) {
                    int newG = current.g + 1;
                    int newH = heuristic(newRow, newCol, goalRow, goalCol);
                    int newIndex = getIndex(newRow, newCol);

                    if (allNodes.find(newIndex) == allNodes.end() || newG < allNodes[newIndex]->g) {
                        Node* neighborNode = new Node(newRow, newCol, newG, newH, allNodes[getIndex(current.row, current.col)]);
                        openList.push(*neighborNode);
                        allNodes[newIndex] = neighborNode;
                    }
                }
            }
        }

        cout << "No path found." << endl;
    }


int main() {
    // Build the graph from the maze
    buildGraph();

    // Print the adjacency list
    cout << "Graph Representation (Adjacency List):" << endl;
    printGraph();
   // Astar({1, 1}, {1, 14}, graph);

    return 0;
}
