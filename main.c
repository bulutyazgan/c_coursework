#include "graphics.h"
#include <stdlib.h>
#include <time.h>

#define ROWS 15
#define COLS 15
#define GRID_SIZE 40
#define WINDOW_WIDTH (COLS * GRID_SIZE)
#define WINDOW_HEIGHT (ROWS * GRID_SIZE)
#define MARKER_COUNT 30
#define DELAY 20
#define TEST_COURSE 3  // Select test course (1-7)

// Coverage Planner States
typedef enum {
    STANDBY,
    COVERAGE_SEARCH,
    NEAREST_UNVISITED_SEARCH,
    FOUND,
    NOT_FOUND
} PlannerState;

// Heuristic Types
typedef enum {
    HEURISTIC_MANHATTAN,
    HEURISTIC_VERTICAL,
    HEURISTIC_HORIZONTAL
} HeuristicType;

typedef struct {
        int x;
        int y;
        int direction; // 0: up, 1: right, 2: down, 3: left
        int markerCount;
        int knowledge[ROWS][COLS]; // 0: unknown, 1: visited, 3: corner, -1: obstacle
} Robot;

typedef struct {
    int x;
    int y;
    int isCarried;
} Marker;

// Action costs for coverage algorithm
static const float action_costs[] = {0.2, 0.1, 0.2, 0.4}; // Right, Forward, Left, Backward

// Movement directions: up, right, down, left (matches Robot direction encoding)
static const int movement[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};

// Drawing Functionalities
void drawArena(int map[COLS][ROWS]);
void drawMarkers(Marker markers[]);
void drawRobot(Robot robot);
void drawMovingObjects(Robot robot, Marker markers[]);

// Core robot functionalities
void forward(Robot* robot, Marker markers[], int map[COLS][ROWS]);
void left(Robot* robot, Marker markers[]);
void right(Robot* robot, Marker markers[]);
int canMoveForward(Robot robot, int map[COLS][ROWS]);
int atMarker(Robot* robot, Marker markers[], int map[COLS][ROWS]);
void pickUpMarker(Robot* robot, Marker* marker, int map[COLS][ROWS]);
void dropMarker(Robot* robot, Marker markers[], int map[COLS][ROWS]);
int markerCount(Robot robot);

// Advanced robot functionalities that utilize core functions with added complexity to work
void turnToDirection(Robot* robot, Marker markers[], int target_direction);
void discoverCorner(Robot* robot, Marker markers[], int map[COLS][ROWS]);
int checkAtCorner(Robot robot);

// Test obstacle course configurations
void setupTestCourse1(int map[COLS][ROWS]); // Empty arena
void setupTestCourse2(int map[COLS][ROWS]); // Corner obstacles
void setupTestCourse3(int map[COLS][ROWS]); // Central wall
void setupTestCourse4(int map[COLS][ROWS]); // Maze pattern
void setupTestCourse5(int map[COLS][ROWS]); // Scattered obstacles
void setupTestCourse6(int map[COLS][ROWS]); // U-shape corridor
void setupTestCourse7(int map[COLS][ROWS]); // Circular pattern

// Coverage planning functions
void performAction(Robot* robot, Marker markers[], int map[COLS][ROWS], int target_orientation);
void createHeuristic(int target_x, int target_y, HeuristicType type, float heuristic[COLS][ROWS]);
int coverageSearch(Robot* robot, Marker markers[], int map[COLS][ROWS], int closed[COLS][ROWS], float heuristic[COLS][ROWS]);
int aStarNavigate(Robot* robot, Marker markers[], int map[COLS][ROWS], int closed[COLS][ROWS], float heuristic[COLS][ROWS], int target_x, int target_y, int any_unvisited);
int findNearestCorner(Robot robot, int* corner_x, int* corner_y);

int main(int argc, char const *argv[])
{
    srand(time(NULL));  // random seed generator

    Marker markers[MARKER_COUNT];
    int map[COLS][ROWS] = {0}; // 0 = empty, bigger than 0: marker index + 1, -1: obstacle, this 2D array is for targeted location checks

    // Setup the selected test course
    switch (TEST_COURSE) {
        case 1: setupTestCourse1(map); break;
        case 2: setupTestCourse2(map); break;
        case 3: setupTestCourse3(map); break;
        case 4: setupTestCourse4(map); break;
        case 5: setupTestCourse5(map); break;
        case 6: setupTestCourse6(map); break;
        case 7: setupTestCourse7(map); break;
    }

    // Initialize robot at random free position (not on obstacles)
    Robot robot = {rand() % COLS, rand() % ROWS, 0, 0, {{0}}}; //position in terms of grid coordinates, not pixels
    while (map[robot.x][robot.y] == -1) {
        robot.x = rand() % COLS;
        robot.y = rand() % ROWS;
    }
    // Initialize markers at random free positions (not on obstacles or robot's starting position)
    for (int i = 0; i < MARKER_COUNT; i++) {
        int x = rand() % COLS;
        int y = rand() % ROWS;
        while ((map[x][y] != 0) || (x == robot.x && y == robot.y)) { // ensure no overlapping markers and not on robot's starting position
            x = rand() % COLS;
            y = rand() % ROWS;
        }
        markers[i].x = x;
        markers[i].y = y;
        markers[i].isCarried = 0;
        map[x][y] = i + 1; // Store marker index + 1 (so 0 means empty)
    }

    // Initialize coverage planning system
    float heuristic[COLS][ROWS];
    PlannerState state = COVERAGE_SEARCH;

    // Create heuristic
    createHeuristic(robot.x, robot.y, HEURISTIC_VERTICAL, heuristic);

    setWindowSize(WINDOW_WIDTH + 1, WINDOW_HEIGHT + 1);

    background();
    drawArena(map);

    foreground();
    drawRobot(robot);
    drawMarkers(markers);

    // Finite State Machine for coverage planning
    while (state != FOUND && state != NOT_FOUND) {
        if (state == COVERAGE_SEARCH) {
            if (coverageSearch(&robot, markers, map, robot.knowledge, heuristic)) {
                state = FOUND;
            } else {
                state = NEAREST_UNVISITED_SEARCH;
            }
        }
        else if (state == NEAREST_UNVISITED_SEARCH) {
            createHeuristic(robot.x, robot.y, HEURISTIC_MANHATTAN, heuristic);

            if (aStarNavigate(&robot, markers, map, robot.knowledge, heuristic, -1, -1, 1)) {
                state = COVERAGE_SEARCH;
                createHeuristic(robot.x, robot.y, HEURISTIC_VERTICAL, heuristic);
            } else {
                state = NOT_FOUND;
            }
        }
    }
    // After coverage complete, check if robot still has markers
    if (markerCount(robot) > 0) {
        // Find nearest discovered corner
        int corner_x, corner_y;
        if (findNearestCorner(robot, &corner_x, &corner_y)) {
            // Create heuristic to navigate to specific corner
            float corner_heuristic[COLS][ROWS];
            createHeuristic(corner_x, corner_y, HEURISTIC_MANHATTAN, corner_heuristic);

            // Navigate to corner using A* (any_unvisited=0, specific target)
            if (aStarNavigate(&robot, markers, map, robot.knowledge, corner_heuristic, corner_x, corner_y, 0)) {
                if (checkAtCorner(robot)) {
                    dropMarker(&robot, markers, map);
                }
            }
        }
    }
    // Algorithm complete
    // Keep display active after completion
    while (1) {
        sleep(1000);
    }
    return 0;
}

void drawArena(int map[COLS][ROWS]){
    // draw grid
    for (int r = 0; r <= ROWS; r++){
        if(r == 0 || r == ROWS){
            setColour(red);
        }
        else{
            setColour(black);
        }
        drawLine(0, r * GRID_SIZE, WINDOW_WIDTH, r * GRID_SIZE);
    }
    for (int c = 0; c <= COLS; c++){
        if(c == 0 || c == COLS){
            setColour(red);
        }
        else{
            setColour(black);
        }
        drawLine(c * GRID_SIZE, 0, c * GRID_SIZE, WINDOW_HEIGHT);
    }
    // draw obstacles
    setColour(darkgray);
    for (int x = 0; x < COLS; x++){
        for (int y = 0; y < ROWS; y++){
            if (map[x][y] == -1){
                fillRect(x * GRID_SIZE, y * GRID_SIZE, GRID_SIZE, GRID_SIZE);
            }
        }
    }
}

void drawMarkers(Marker markers[]){
    setColour(gray);
    for (int i = 0; i < MARKER_COUNT; i++) {
        fillArc(markers[i].x * GRID_SIZE + GRID_SIZE / 4, markers[i].y * GRID_SIZE + GRID_SIZE / 4, GRID_SIZE / 2, GRID_SIZE / 2, 0, 360);
    }
}

void drawRobot(Robot robot){
    //triangle representing the robot
    setColour(blue);
    int centerX = robot.x * GRID_SIZE + GRID_SIZE / 2;
    int centerY = robot.y * GRID_SIZE + GRID_SIZE / 2;
    int s = GRID_SIZE / 3;

    switch (robot.direction){
        case 0: //up
            {
                int xPoints[3] = { centerX, centerX - s, centerX + s };
                int yPoints[3] = { centerY - s, centerY + s, centerY + s };
                fillPolygon(3, xPoints, yPoints);
                break;
            }
        case 1: //right
            {
                int xPoints[3] = { centerX + s, centerX - s, centerX - s };
                int yPoints[3] = { centerY, centerY - s, centerY + s };
                fillPolygon(3, xPoints, yPoints);
                break;
            }
        case 2: //down
            {
                int xPoints[3] = { centerX, centerX - s, centerX + s };
                int yPoints[3] = { centerY + s, centerY - s, centerY - s };
                fillPolygon(3, xPoints, yPoints);
                break;
            }
        case 3: //left
            {
                int xPoints[3] = { centerX - s, centerX + s, centerX + s };
                int yPoints[3] = { centerY, centerY - s, centerY + s };
                fillPolygon(3, xPoints, yPoints);
                break;
            }
    }
}

void drawMovingObjects(Robot robot, Marker markers[]){
    clear(); // everytime there's a movement, update the elements that moved
    drawRobot(robot);
    drawMarkers(markers);
    sleep(DELAY);
}

void forward(Robot* robot, Marker markers[], int map[COLS][ROWS]){
    robot->x += movement[robot->direction][0];
    robot->y += movement[robot->direction][1];
    for (int i = 0; i < MARKER_COUNT; i++) { // Update positions of carried markers
        if (markers[i].isCarried) {
            markers[i].x = robot->x;
            markers[i].y = robot->y;
        }
    }

    if (atMarker(robot, markers, map)){ //check if robot is at a marker after moving
        pickUpMarker(robot, &markers[map[robot->x][robot->y] - 1], map); // deduct 1 to get the correct index
    }

    drawMovingObjects(*robot, markers);
}

void left(Robot* robot, Marker markers[]){
    robot->direction = (robot->direction + 3) % 4; //turn left (suggestion made by github copilot, modulo loop)
    drawMovingObjects(*robot, markers);
}
void right(Robot* robot, Marker markers[]){
    robot->direction = (robot->direction + 1) % 4; //turn right (suggestion made by github copilot, modulo loop)
    drawMovingObjects(*robot, markers);
}

// Turn robot to face target direction using shortest path (left or right)
void turnToDirection(Robot* robot, Marker markers[], int target_direction) {
    int current = robot->direction;
    int target = target_direction;

    // Calculate clockwise and counter-clockwise distances
    int clockwise_distance = (target - current + 4) % 4;
    int counter_clockwise_distance = (current - target + 4) % 4;

    // Choose shortest path
    if (clockwise_distance <= counter_clockwise_distance) {
        // Turn right (clockwise)
        for (int i = 0; i < clockwise_distance; i++) {
            right(robot, markers);
        }
    } else {
        // Turn left (counter-clockwise)
        for (int i = 0; i < counter_clockwise_distance; i++) {
            left(robot, markers);
        }
    }
}

int canMoveForward(Robot robot, int map[COLS][ROWS]){
    int next_x = robot.x + movement[robot.direction][0];
    int next_y = robot.y + movement[robot.direction][1];

    if (next_x < 0 || next_x >= COLS || next_y < 0 || next_y >= ROWS) {
        return 0;
    }
    return map[next_x][next_y] != -1;
}

// Discover if current position is a corner (2 adjacent obstacles/edges in L-shape)
// Robot physically turns to check each direction using only forward-facing sensor
// Uses robot.knowledge memory to avoid checking directions with previously visited cells
// Updates robot knowledge map with corner information
// Only performs check if this cell hasn't been analyzed yet (knowledge == 0)
void discoverCorner(Robot* robot, Marker markers[], int map[COLS][ROWS]) {
    int x = robot->x;
    int y = robot->y;

    // Skip if we've already analyzed this cell for corners
    // knowledge values: 0=unknown, 1=empty, 2=marker, 3=corner, -1=obstacle
    if (robot->knowledge[x][y] != 0) {
        return;  // Already analyzed
    }

    // Array to store if each direction is blocked (0=up, 1=right, 2=down, 3=left)
    int blocked[4] = {0};

    // Check all 4 directions starting from current facing direction
    // This avoids unnecessary initial turn and checks current direction first
    for (int offset = 0; offset < 4; offset++) {
        int dir = (robot->direction + offset) % 4;  // Start from current direction, wrap around

        // Calculate neighbor position for this direction
        int nx = x + movement[dir][0];
        int ny = y + movement[dir][1];

        // If neighbor is out of bounds, it's blocked (edge of arena)
        if (nx < 0 || nx >= COLS || ny < 0 || ny >= ROWS) {
            blocked[dir] = 1;
            continue;
        }

        // Check if we already know about this neighbor from robot's memory
        if (robot->knowledge[nx][ny] == -1) {
            blocked[dir] = 1;  // Known obstacle
            continue;
        } else if (robot->knowledge[nx][ny] == 1 || robot->knowledge[nx][ny] == 3) {
            blocked[dir] = 0;  // Already visited = definitely not blocked
            continue;
        }

        // Unknown cell - need to physically check with sensor
        // For first iteration (offset=0), robot is already facing this direction
        if (offset > 0) {
            turnToDirection(robot, markers, dir);
        }

        // Use forward-facing sensor to check if path is blocked
        if (!canMoveForward(*robot, map)) {
            blocked[dir] = 1;  // This direction is blocked (obstacle)
            robot->knowledge[nx][ny] = -1;  // Remember this obstacle
        }
    }

    // Check for L-shaped corner patterns (2 adjacent blocked directions)
    int is_corner = (blocked[0] && blocked[3]) ||  // Top-left
                    (blocked[0] && blocked[1]) ||  // Top-right
                    (blocked[1] && blocked[2]) ||  // Bottom-right
                    (blocked[2] && blocked[3]);    // Bottom-left

    robot->knowledge[x][y] = is_corner ? 3 : 1;
}

int checkAtCorner(Robot robot){
    return robot.knowledge[robot.x][robot.y] == 3;
}

int markerCount(Robot robot){
    return robot.markerCount;
}

int atMarker(Robot* robot, Marker markers[], int map[COLS][ROWS]){
    return map[robot->x][robot->y] > 0;
}

void pickUpMarker(Robot* robot, Marker* marker, int map[COLS][ROWS]) {
    marker->isCarried = 1;
    robot->markerCount += 1;
    map[marker->x][marker->y] = 0; // Remove marker from map
}

void dropMarker(Robot* robot, Marker markers[], int map[COLS][ROWS]){
    for (int i = 0; i < MARKER_COUNT; i++) { // this function gets called when the robot is at the corner and has at least one marker, and the function drops all the markers
        if (markers[i].isCarried) {
            markers[i].isCarried = 0;
            robot->markerCount -= 1;
            // markers are dropped in a corner and now inactive (mission accomplished), so we don't need to put it back in the map
        }
    }
}

// ========== COVERAGE PATH PLANNING FUNCTIONS ==========

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

// Coverage search algorithm - greedy lawnmower pattern
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
        float v = 0.0; // Current accumulated cost (simplified for C version)

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
// Generic A* navigation function
// If any_unvisited=1: finds ANY unvisited cell (uses closed grid to check)
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

// ========== TEST OBSTACLE COURSES ==========

// Course 1: Empty Arena (Baseline Test)
// Perfect for testing basic coverage algorithm
void setupTestCourse1(int map[COLS][ROWS]) {
    // No obstacles - just empty arena
    // Robot should complete a simple lawnmower pattern
}

// Course 2: Corner Obstacles (Python Test Map 1 inspired)
// Tests algorithm's ability to navigate around corner obstacles
void setupTestCourse2(int map[COLS][ROWS]) {
    // Obstacles in 4x4 blocks at three corners (scaled from 3x3 for 15x15)
    // Top-left corner
    for (int x = 0; x < 4; x++) {
        for (int y = 0; y < 4; y++) {
            map[x][y] = -1;
        }
    }

    // Top-right corner
    for (int x = 11; x < 15; x++) {
        for (int y = 0; y < 4; y++) {
            map[x][y] = -1;
        }
    }

    // Bottom-left corner
    for (int x = 0; x < 4; x++) {
        for (int y = 11; y < 15; y++) {
            map[x][y] = -1;
        }
    }

    // Leave bottom-right corner clear for marker drop
}

// Course 3: Central Wall (Tests A* Navigation)
// Vertical wall in middle forces robot to go around
void setupTestCourse3(int map[COLS][ROWS]) {
    // Vertical wall down the middle with gaps (scaled for 15x15)
    for (int y = 1; y < 14; y++) {
        if (y != 4 && y != 9) {  // Leave two gaps at y=4 and y=9
            map[7][y] = -1;  // Middle column at x=7
        }
    }
}

// Course 4: Maze Pattern (Complex Navigation Test)
// Multiple corridors and dead-ends
void setupTestCourse4(int map[COLS][ROWS]) {
    // Horizontal walls (scaled for 15x15)
    for (int x = 2; x < 7; x++) map[x][3] = -1;
    for (int x = 8; x < 13; x++) map[x][6] = -1;
    for (int x = 2; x < 7; x++) map[x][9] = -1;
    for (int x = 8; x < 13; x++) map[x][12] = -1;

    // Vertical walls
    for (int y = 4; y < 9; y++) map[5][y] = -1;
    for (int y = 2; y < 6; y++) map[10][y] = -1;
    for (int y = 10; y < 13; y++) map[4][y] = -1;

    // Additional maze complexity
    for (int y = 7; y < 11; y++) map[11][y] = -1;
}

// Course 5: Scattered Obstacles (Python Test Map 2 inspired)
// Random-looking obstacle placement
void setupTestCourse5(int map[COLS][ROWS]) {
    // Large obstacle block in center (scaled for 15x15)
    for (int x = 5; x <= 9; x++) {
        for (int y = 3; y <= 7; y++) {
            map[x][y] = -1;
        }
    }

    // Smaller obstacle blocks scattered around
    // Top area
    for (int x = 1; x <= 2; x++) {
        for (int y = 1; y <= 2; y++) {
            map[x][y] = -1;
        }
    }

    // Right side
    map[12][4] = -1; map[13][4] = -1;
    map[12][5] = -1; map[13][5] = -1;

    // Bottom left
    map[1][11] = -1; map[2][11] = -1;
    map[1][12] = -1; map[2][12] = -1;

    // Bottom right
    map[11][11] = -1; map[12][11] = -1; map[13][11] = -1;
    map[11][12] = -1; map[12][12] = -1;
}

// Course 6: U-Shape Corridor (Tests Corridor Following)
// Forces robot to follow a specific path pattern
void setupTestCourse6(int map[COLS][ROWS]) {
    // Outer walls creating U-shape (scaled for 15x15)
    for (int y = 2; y < 11; y++) {
        map[3][y] = -1;   // Left wall
        map[11][y] = -1;  // Right wall
    }

    // Bottom of U
    for (int x = 3; x <= 11; x++) {
        map[x][10] = -1;
    }

    // Inner obstacles in the corridor
    for (int x = 6; x <= 8; x++) {
        for (int y = 5; y <= 7; y++) {
            map[x][y] = -1;
        }
    }
}

// Course 7: Circular Pattern (Stage 5 Preview)
// Approximates circular obstacle using rectangular tiles
void setupTestCourse7(int map[COLS][ROWS]) {
    // Approximate circle centered at (7,7) with radius ~5 (scaled for 15x15)
    // Top arc
    for (int x = 6; x <= 8; x++) map[x][2] = -1;
    for (int x = 5; x <= 9; x++) map[x][3] = -1;
    for (int x = 4; x <= 10; x++) map[x][4] = -1;

    // Middle sides (widest part)
    for (int x = 3; x <= 11; x++) {
        if (x <= 4 || x >= 10) {  // Only sides
            for (int y = 5; y <= 9; y++) {
                map[x][y] = -1;
            }
        }
    }

    // Bottom arc (mirror of top)
    for (int x = 4; x <= 10; x++) map[x][10] = -1;
    for (int x = 5; x <= 9; x++) map[x][11] = -1;
    for (int x = 6; x <= 8; x++) map[x][12] = -1;

    // Creates a circular obstacle with free space around edges and in center
}