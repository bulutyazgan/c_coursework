#include "path_planning.h"
#include "robot.h"
#include <stdlib.h>

// PATHFINDING ALGORITHMS INSPIRED BY: https://github.com/rodriguesrenato/coverage-path-planning
// RELATED RESEARCH DOCS ARE INSIDE THE DOCS FOLDER OF THE PROJECT

// Action costs for coverage algorithm
const float action_costs[4] = {0.2, 0.1, 0.2, 0.4}; // Right, Forward, Left, Backward

// Import movement array from robot.c
extern const int movement[4][2];

// COVERAGE PATH PLANNING FUNCTIONS

// Create heuristic grid based on target position and heuristic type
void createHeuristic(int target_x, int target_y, HeuristicType type, float heuristic[COLS][ROWS]) {
    for (int x = 0; x < COLS; x++) {
        for (int y = 0; y < ROWS; y++) {
            switch (type) {
                case HEURISTIC_MANHATTAN:
                    heuristic[x][y] = abs(x - target_x) + abs(y - target_y);
                    break;
                case HEURISTIC_VERTICAL:
                    heuristic[x][y] = abs(y - target_y);
                    break;
                case HEURISTIC_HORIZONTAL:
                    heuristic[x][y] = abs(x - target_x);
                    break;
            }
        }
    }
}

// Turn to target orientation and move forward
void performAction(Robot* robot, Marker markers[], int map[COLS][ROWS], int target_orientation) {
    turnToDirection(robot, markers, target_orientation);
    if (canMoveForward(*robot, map)) {
        forward(robot, markers, map);
    }
}

// Coverage search algorithm
// Returns 1 if complete coverage achieved, 0 if stuck (need A* search)
int coverageSearch(Robot* robot, Marker markers[], int map[COLS][ROWS], int closed[COLS][ROWS], float heuristic[COLS][ROWS]) {
    // Check if starting position is a corner (this will mark it as visited too)
    discoverCorner(robot, markers, map);

    // If discoverCorner didn't mark it (shouldn't happen but safety check)
    if (closed[robot->x][robot->y] == 0) {
        closed[robot->x][robot->y] = 1;
    }

    int resign = 0;

    while (!resign) {
        // Get current position
        int x = robot->x;
        int y = robot->y;
        int o = robot->direction;
        float v = 0.0; // Current accumulated cost

        // Structure to hold possible next moves
        typedef struct {
            float cost;
            int x;
            int y;
            int orientation;
        } Candidate;

        Candidate possible_next[4];
        int candidate_count = 0;

        static const int orientation_changes[] = {1, 0, 3, 2}; // RIGHT, FORWARD, LEFT, BACKWARD

        for (int a = 0; a < 4; a++) {
            int o2 = (o + orientation_changes[a]) % 4;
            int x2 = x + movement[o2][0];
            int y2 = y + movement[o2][1];

            if (x2 >= 0 && x2 < COLS && y2 >= 0 && y2 < ROWS && closed[x2][y2] == 0) {
                possible_next[candidate_count].cost = v + action_costs[a] + heuristic[x2][y2];
                possible_next[candidate_count].x = x2;
                possible_next[candidate_count].y = y2;
                possible_next[candidate_count].orientation = o2;
                candidate_count++;
            }
        }

        if (candidate_count == 0) {
            resign = 1;
            // Coverage search stuck, need A* search
        } else {
            // Find candidate with lowest cost
            int best_idx = 0;
            for (int i = 1; i < candidate_count; i++) {
                if (possible_next[i].cost < possible_next[best_idx].cost) {
                    best_idx = i;
                }
            }

            // Perform the action using target orientation
            // This handles the case where discoverCorner() left the robot facing any direction
            performAction(robot, markers, map, possible_next[best_idx].orientation);

            // Discover if this position is a corner (this will mark it as visited: 1 or corner: 3)
            discoverCorner(robot, markers, map);

            // Safety: If discoverCorner didn't mark it, mark as visited
            if (closed[robot->x][robot->y] == 0) {
                closed[robot->x][robot->y] = 1;
            }

            // If robot is carrying markers and just discovered it's at a corner, drop them immediately
            if (markerCount(*robot) > 0 && checkAtCorner(*robot)) {
                dropMarker(robot, markers, map);
                // Markers dropped at corner
            }
        }
    }

    return 0; // Resigned - need A* to find unvisited cells
}

// A* search to find nearest unvisited cell
// Returns 1 if path found, 0 if no unvisited cells reachable
// If any_unvisited=1: finds ANY unvisited cell
// If any_unvisited=0: navigates to specific (target_x, target_y)
// Returns 1 if path found and executed, 0 if failed
int aStarNavigate(Robot* robot, Marker markers[], int map[COLS][ROWS], int closed[COLS][ROWS], float heuristic[COLS][ROWS],
                  int target_x, int target_y, int any_unvisited) {
    // Search-specific closed grid (different from coverage closed)
    int search_closed[COLS][ROWS] = {0};
    search_closed[robot->x][robot->y] = 1;

    // Parent tracking for path reconstruction
    int parent_x[COLS][ROWS];
    int parent_y[COLS][ROWS];
    int parent_dir[COLS][ROWS];

    // Initialize parents
    for (int i = 0; i < COLS; i++) {
        for (int j = 0; j < ROWS; j++) {
            parent_x[i][j] = -1;
            parent_y[i][j] = -1;
            parent_dir[i][j] = -1;
        }
    }

    // Open list node
    typedef struct {
        float f;  // f = g + h
        float g;  // cost from start
        int x;
        int y;
    } OpenNode;

    OpenNode open_list[COLS * ROWS];
    int open_count = 0;

    // Add start position
    float g = 0;
    float f = g + heuristic[robot->x][robot->y];
    open_list[open_count].f = f;
    open_list[open_count].g = g;
    open_list[open_count].x = robot->x;
    open_list[open_count].y = robot->y;
    open_count++;
    parent_dir[robot->x][robot->y] = robot->direction;

    int found = 0;
    int goal_x = -1, goal_y = -1;

    while (!found && open_count > 0) {
        // Find node with lowest f in open list
        int best_idx = 0;
        for (int i = 1; i < open_count; i++) {
            if (open_list[i].f < open_list[best_idx].f) {
                best_idx = i;
            }
        }

        // Get current node and remove from open list
        OpenNode current = open_list[best_idx];
        open_list[best_idx] = open_list[open_count - 1];
        open_count--;

        int x = current.x;
        int y = current.y;
        float g = current.g;

        // Check if we reached goal
        if (any_unvisited) {
            // Looking for ANY unvisited cell
            if (closed[x][y] == 0) {
                found = 1;
                goal_x = x;
                goal_y = y;
                break;
            }
        } else {
            // Looking for specific target
            if (x == target_x && y == target_y) {
                found = 1;
                goal_x = x;
                goal_y = y;
                break;
            }
        }

        // Expand neighbors in all 4 directions
        for (int i = 0; i < 4; i++) {
            int x_next = x + movement[i][0];
            int y_next = y + movement[i][1];

            if (x_next >= 0 && x_next < COLS && y_next >= 0 && y_next < ROWS &&
                closed[x_next][y_next] != -1 && search_closed[x_next][y_next] == 0) {

                float g2 = g + 1.0;

                open_list[open_count].f = g2 + heuristic[x_next][y_next];
                open_list[open_count].g = g2;
                open_list[open_count].x = x_next;
                open_list[open_count].y = y_next;
                open_count++;

                search_closed[x_next][y_next] = 1;
                parent_x[x_next][y_next] = x;
                parent_y[x_next][y_next] = y;
                parent_dir[x_next][y_next] = i;
            }
        }
    }

    if (!found) {
        return 0;
    }

    // Reconstruct path from goal to start
    int path_x[COLS * ROWS];
    int path_y[COLS * ROWS];
    int path_len = 0;

    int cx = goal_x;
    int cy = goal_y;

    while (!(cx == robot->x && cy == robot->y)) {
        path_x[path_len] = cx;
        path_y[path_len] = cy;
        path_len++;

        int px = parent_x[cx][cy];
        int py = parent_y[cx][cy];
        cx = px;
        cy = py;
    }

    // Reverse path (it's backwards)
    for (int i = 0; i < path_len / 2; i++) {
        int temp_x = path_x[i];
        int temp_y = path_y[i];
        path_x[i] = path_x[path_len - 1 - i];
        path_y[i] = path_y[path_len - 1 - i];
        path_x[path_len - 1 - i] = temp_x;
        path_y[path_len - 1 - i] = temp_y;
    }

    // Execute path
    for (int i = 0; i < path_len; i++) {
        for (int dir = 0; dir < 4; dir++) {
            int next_x = robot->x + movement[dir][0];
            int next_y = robot->y + movement[dir][1];

            if (next_x == path_x[i] && next_y == path_y[i]) {
                performAction(robot, markers, map, dir);
                break;
            }
        }
    }

    return 1;
}

// Find nearest discovered corner in robot's knowledge map
// Returns 1 if corner found, 0 if no corners discovered yet
int findNearestCorner(Robot robot, int* corner_x, int* corner_y) {
    int found = 0;
    int min_distance = COLS * ROWS;  // Maximum possible distance

    // Search through robot's knowledge for discovered corners
    for (int x = 0; x < COLS; x++) {
        for (int y = 0; y < ROWS; y++) {
            if (robot.knowledge[x][y] == 3) {  // Found a discovered corner
                // Calculate Manhattan distance
                int distance = abs(x - robot.x) + abs(y - robot.y);
                if (distance < min_distance) {
                    min_distance = distance;
                    *corner_x = x;
                    *corner_y = y;
                    found = 1;
                }
            }
        }
    }

    return found;
}
