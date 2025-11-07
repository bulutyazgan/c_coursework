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

// Structure to hold possible next moves
typedef struct {
    float cost;
    int x;
    int y;
    int orientation;
} Candidate;

// Evaluate all possible next moves from current position
int evaluateCandidates(int x, int y, int o, float v, int closed[COLS][ROWS],
                       float heuristic[COLS][ROWS], Candidate possible_next[4]) {
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
    return candidate_count;
}

// Find candidate with lowest cost
int selectBestCandidate(Candidate possible_next[], int candidate_count) {
    int best_idx = 0;
    for (int i = 1; i < candidate_count; i++) {
        if (possible_next[i].cost < possible_next[best_idx].cost) {
            best_idx = i;
        }
    }
    return best_idx;
}

// Execute coverage action and handle marker delivery
void executeCoverageAction(Robot* robot, Marker markers[], int map[COLS][ROWS],
                           int closed[COLS][ROWS], Candidate best_move) {
    performAction(robot, markers, map, best_move.orientation);
    discoverCorner(robot, markers, map);

    if (closed[robot->x][robot->y] == 0) {
        closed[robot->x][robot->y] = 1;
    }

    if (markerCount(*robot) > 0 && checkAtCorner(*robot)) {
        dropMarker(robot, markers, map);
    }
}

// Coverage search algorithm
// Returns 1 if complete coverage achieved, 0 if stuck (need A* search)
int coverageSearch(Robot* robot, Marker markers[], int map[COLS][ROWS],
                   int closed[COLS][ROWS], float heuristic[COLS][ROWS]) {
    discoverCorner(robot, markers, map);

    if (closed[robot->x][robot->y] == 0) {
        closed[robot->x][robot->y] = 1;
    }

    int resign = 0;
    while (!resign) {
        Candidate possible_next[4];
        int candidate_count = evaluateCandidates(robot->x, robot->y, robot->direction,
                                                  0.0, closed, heuristic, possible_next);

        if (candidate_count == 0) {
            resign = 1;
        } else {
            int best_idx = selectBestCandidate(possible_next, candidate_count);
            executeCoverageAction(robot, markers, map, closed, possible_next[best_idx]);
        }
    }

    return 0;
}

// Open list node structure for A*
typedef struct {
    float f;  // f = g + h
    float g;  // cost from start
    int x;
    int y;
} OpenNode;

// Find node with lowest f-cost in open list
int findBestOpenNode(OpenNode open_list[], int open_count) {
    int best_idx = 0;
    for (int i = 1; i < open_count; i++) {
        if (open_list[i].f < open_list[best_idx].f) {
            best_idx = i;
        }
    }
    return best_idx;
}

// Check if current position meets goal condition
int checkGoalCondition(int x, int y, int closed[COLS][ROWS],
                       int target_x, int target_y, int any_unvisited) {
    if (any_unvisited) {
        return closed[x][y] == 0;
    }
    return (x == target_x && y == target_y);
}

// Expand neighbors and add to open list
void expandNeighbors(int x, int y, float g, int closed[COLS][ROWS],
                     int search_closed[COLS][ROWS], float heuristic[COLS][ROWS],
                     OpenNode open_list[], int* open_count,
                     int parent_x[COLS][ROWS], int parent_y[COLS][ROWS],
                     int parent_dir[COLS][ROWS]) {
    for (int i = 0; i < 4; i++) {
        int x_next = x + movement[i][0];
        int y_next = y + movement[i][1];

        if (x_next >= 0 && x_next < COLS && y_next >= 0 && y_next < ROWS &&
            closed[x_next][y_next] != -1 && search_closed[x_next][y_next] == 0) {

            float g2 = g + 1.0;
            open_list[*open_count].f = g2 + heuristic[x_next][y_next];
            open_list[*open_count].g = g2;
            open_list[*open_count].x = x_next;
            open_list[*open_count].y = y_next;
            (*open_count)++;

            search_closed[x_next][y_next] = 1;
            parent_x[x_next][y_next] = x;
            parent_y[x_next][y_next] = y;
            parent_dir[x_next][y_next] = i;
        }
    }
}

// Reconstruct path from goal to start using parent pointers
int reconstructPath(int goal_x, int goal_y, int start_x, int start_y,
                    int parent_x[COLS][ROWS], int parent_y[COLS][ROWS],
                    int path_x[], int path_y[]) {
    int path_len = 0;
    int cx = goal_x;
    int cy = goal_y;

    while (!(cx == start_x && cy == start_y)) {
        path_x[path_len] = cx;
        path_y[path_len] = cy;
        path_len++;
        int px = parent_x[cx][cy];
        int py = parent_y[cx][cy];
        cx = px;
        cy = py;
    }
    return path_len;
}

// Reverse path array (path is built backwards)
void reversePath(int path_x[], int path_y[], int path_len) {
    for (int i = 0; i < path_len / 2; i++) {
        int temp_x = path_x[i];
        int temp_y = path_y[i];
        path_x[i] = path_x[path_len - 1 - i];
        path_y[i] = path_y[path_len - 1 - i];
        path_x[path_len - 1 - i] = temp_x;
        path_y[path_len - 1 - i] = temp_y;
    }
}

// Execute path by navigating robot through waypoints
void executePath(Robot* robot, Marker markers[], int map[COLS][ROWS],
                 int path_x[], int path_y[], int path_len) {
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
}

// Initialize parent tracking arrays
void initializeParents(int parent_x[COLS][ROWS], int parent_y[COLS][ROWS],
                       int parent_dir[COLS][ROWS]) {
    for (int i = 0; i < COLS; i++) {
        for (int j = 0; j < ROWS; j++) {
            parent_x[i][j] = -1;
            parent_y[i][j] = -1;
            parent_dir[i][j] = -1;
        }
    }
}

// A* search to find nearest unvisited cell
// Returns 1 if path found and executed, 0 if failed
int aStarNavigate(Robot* robot, Marker markers[], int map[COLS][ROWS],
                  int closed[COLS][ROWS], float heuristic[COLS][ROWS],
                  int target_x, int target_y, int any_unvisited) {
    int search_closed[COLS][ROWS] = {0};
    search_closed[robot->x][robot->y] = 1;

    int parent_x[COLS][ROWS], parent_y[COLS][ROWS], parent_dir[COLS][ROWS];
    initializeParents(parent_x, parent_y, parent_dir);

    OpenNode open_list[COLS * ROWS];
    int open_count = 0;

    // Add start position
    open_list[0].f = heuristic[robot->x][robot->y];
    open_list[0].g = 0;
    open_list[0].x = robot->x;
    open_list[0].y = robot->y;
    open_count = 1;
    parent_dir[robot->x][robot->y] = robot->direction;

    int found = 0, goal_x = -1, goal_y = -1;

    while (!found && open_count > 0) {
        int best_idx = findBestOpenNode(open_list, open_count);
        OpenNode current = open_list[best_idx];
        open_list[best_idx] = open_list[--open_count];

        if (checkGoalCondition(current.x, current.y, closed, target_x, target_y, any_unvisited)) {
            found = 1;
            goal_x = current.x;
            goal_y = current.y;
            break;
        }

        expandNeighbors(current.x, current.y, current.g, closed, search_closed,
                        heuristic, open_list, &open_count, parent_x, parent_y, parent_dir);
    }

    if (!found) return 0;

    int path_x[COLS * ROWS], path_y[COLS * ROWS];
    int path_len = reconstructPath(goal_x, goal_y, robot->x, robot->y,
                                    parent_x, parent_y, path_x, path_y);
    reversePath(path_x, path_y, path_len);
    executePath(robot, markers, map, path_x, path_y, path_len);
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
