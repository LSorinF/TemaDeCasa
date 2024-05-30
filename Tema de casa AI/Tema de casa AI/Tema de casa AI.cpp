#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <climits>

using namespace std;

class Node {
public:
    int city;
    vector<int> path;
    int cost;
    int heuristic;

    Node(int city, const vector<int>& path, int cost, int heuristic = 0)
        : city(city), path(path), cost(cost), heuristic(heuristic) {
        this->path.push_back(city);
    }

    bool operator<(const Node& other) const {
        return (cost + heuristic) > (other.cost + other.heuristic);
    }
};

// Declaration for the heuristic function
int Heuristic(const vector<vector<int>>& distanceMatrix, int currentCity, int startCity, const vector<int>& visitedCities);

// Breadth-First Search
vector<int> SolveTSP_BFS(const vector<vector<int>>& distanceMatrix, int startCity) {
    int n = distanceMatrix.size();
    queue<Node> frontier;
    frontier.push(Node(startCity, vector<int>(), 0));

    while (!frontier.empty()) {
        Node node = frontier.front();
        frontier.pop();

        if (node.path.size() == n) {
            node.path.push_back(startCity); // return to start city
            return node.path;
        }

        for (int i = 0; i < n; ++i) {
            if (find(node.path.begin(), node.path.end(), i) == node.path.end()) {
                frontier.push(Node(i, node.path, node.cost + distanceMatrix[node.city][i]));
            }
        }
    }
    return vector<int>(); // if there's no solution found
}

// Uniform Cost Search 
vector<int> SolveTSP_UCS(const vector<vector<int>>& distanceMatrix, int startCity) {
    int n = distanceMatrix.size();
    priority_queue<Node> frontier;
    frontier.push(Node(startCity, vector<int>(), 0));

    while (!frontier.empty()) {
        Node node = frontier.top();
        frontier.pop();

        if (node.path.size() == n) {
            node.path.push_back(startCity); // return to start city
            return node.path;
        }

        for (int i = 0; i < n; ++i) {
            if (find(node.path.begin(), node.path.end(), i) == node.path.end()) {
                frontier.push(Node(i, node.path, node.cost + distanceMatrix[node.city][i]));
            }
        }
    }
    return vector<int>(); // if there's no solution found
}

// A* Search
vector<int> SolveTSP_AStar(const vector<vector<int>>& distanceMatrix, int startCity) {
    int n = distanceMatrix.size();
    priority_queue<Node> frontier;
    frontier.push(Node(startCity, vector<int>(), 0, 0));

    while (!frontier.empty()) {
        Node node = frontier.top();
        frontier.pop();

        if (node.path.size() == n) {
            node.path.push_back(startCity); // return to start city
            return node.path;
        }

        for (int i = 0; i < n; ++i) {
            if (find(node.path.begin(), node.path.end(), i) == node.path.end()) {
                int newCost = node.cost + distanceMatrix[node.city][i];
                int heuristic = Heuristic(distanceMatrix, i, startCity, node.path);
                frontier.push(Node(i, node.path, newCost, heuristic));
            }
        }
    }
    return vector<int>(); // if there's no solution found
}

int Heuristic(const vector<vector<int>>& distanceMatrix, int currentCity, int startCity, const vector<int>& visitedCities) {
    int minDist = INT_MAX;
    for (int i = 0; i < distanceMatrix.size(); ++i) {
        if (find(visitedCities.begin(), visitedCities.end(), i) == visitedCities.end() && distanceMatrix[currentCity][i] < minDist) {
            minDist = distanceMatrix[currentCity][i];
        }
    }
    return minDist;
}

int CalculatePathCost(const vector<vector<int>>& distanceMatrix, const vector<int>& path) {
    int cost = 0;
    for (int i = 0; i < path.size() - 1; ++i) {
        cost += distanceMatrix[path[i]][path[i + 1]];
    }
    return cost;
}

int main() {
    vector<vector<int>> distanceMatrix = {
        { 0, 29, 20, 21 },
        { 29, 0, 15, 17 },
        { 20, 15, 0, 28 },
        { 21, 17, 28, 0 }
    };
    int startCity = 0;

    vector<int> pathBFS = SolveTSP_BFS(distanceMatrix, startCity);
    cout << "BFS Path: ";
    for (int city : pathBFS) {
        cout << city << " -> ";
    }
    cout << " with cost " << CalculatePathCost(distanceMatrix, pathBFS) << endl;

    vector<int> pathUCS = SolveTSP_UCS(distanceMatrix, startCity);
    cout << "UCS Path: ";
    for (int city : pathUCS) {
        cout << city << " -> ";
    }
    cout << " with cost " << CalculatePathCost(distanceMatrix, pathUCS) << endl;

    vector<int> pathAStar = SolveTSP_AStar(distanceMatrix, startCity);
    cout << "A* Path: ";
    for (int city : pathAStar) {
        cout << city << " -> ";
    }
    cout << " with cost " << CalculatePathCost(distanceMatrix, pathAStar) << endl;

    return 0;
}
