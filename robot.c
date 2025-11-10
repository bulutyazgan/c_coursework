#include "robot.h"
#include "graphics.h"
#include <stdlib.h>

// Movement directions: up, right, down, left
const int movement[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
extern void drawMovingObjects(Robot robot, Marker markers[]);

// CORE ROBOT FUNCTIONALITIES
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

int canMoveForward(Robot robot, int map[COLS][ROWS]){
    int next_x = robot.x + movement[robot.direction][0];
    int next_y = robot.y + movement[robot.direction][1];

    if (next_x < 0 || next_x >= COLS || next_y < 0 || next_y >= ROWS) {
        return 0;
    }
    return map[next_x][next_y] != -1;
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
            // markers are dropped in a corner and now inactive (mission accomplished), so we don't need to put it back in the map array
        }
    }
}

// ADVANCED ROBOT FUNCTIONALITIES

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


// Turn to target orientation and move forward
void performAction(Robot* robot, Marker markers[], int map[COLS][ROWS], int target_orientation) {
    turnToDirection(robot, markers, target_orientation);
    if (canMoveForward(*robot, map)) {
        forward(robot, markers, map);
    }
}


// Check if direction is blocked using memory or sensor
int checkDirectionBlocked(Robot* robot, Marker markers[], int map[COLS][ROWS], int dir, int offset) {
    int x = robot->x;
    int y = robot->y;
    int nx = x + movement[dir][0];
    int ny = y + movement[dir][1];

    // Check if we have memory about this direction
    // If the cell is within bounds, check robot's knowledge array
    if (nx >= 0 && nx < COLS && ny >= 0 && ny < ROWS) {
        if (robot->knowledge[nx][ny] == -1) return 1;  // Known obstacle
        if (robot->knowledge[nx][ny] == 1 || robot->knowledge[nx][ny] == 3) return 0;  // Known empty
    }

    // Unknown cell (either out-of-bounds or unvisited) - physically check with sensor
    if (offset > 0) {
        turnToDirection(robot, markers, dir);
    }

    if (!canMoveForward(*robot, map)) {
        // Sensor found it's blocked - save to memory if within bounds
        if (nx >= 0 && nx < COLS && ny >= 0 && ny < ROWS) {
            robot->knowledge[nx][ny] = -1;
        }
        return 1;
    }
    return 0;
}

// Check for L-shaped corner pattern (2 adjacent blocked directions)
int isCornerPattern(int blocked[4]) {
    return (blocked[0] && blocked[3]) ||  // Top-left
           (blocked[0] && blocked[1]) ||  // Top-right
           (blocked[1] && blocked[2]) ||  // Bottom-right
           (blocked[2] && blocked[3]);    // Bottom-left
}

// Discover if current position is a corner using sensor-based detection
void discoverCorner(Robot* robot, Marker markers[], int map[COLS][ROWS]) {
    if (robot->knowledge[robot->x][robot->y] != 0) return;

    int blocked[4] = {0};
    for (int offset = 0; offset < 4; offset++) {
        int dir = (robot->direction + offset) % 4;
        blocked[dir] = checkDirectionBlocked(robot, markers, map, dir, offset);
    }

    robot->knowledge[robot->x][robot->y] = isCornerPattern(blocked) ? 3 : 1;
}

int checkAtCorner(Robot robot){
    return robot.knowledge[robot.x][robot.y] == 3;
}
