#include "graphics.h"
#include "robot.h"
#include "maps.h"
#include "path_planning.h"
#include <stdlib.h>
#include <time.h>

#define GRID_SIZE 40
#define WINDOW_WIDTH (COLS * GRID_SIZE)
#define WINDOW_HEIGHT (ROWS * GRID_SIZE)
#define MARKER_COUNT 30
#define DELAY 20
#define TEST_COURSE 5  // Select test course (1-7)

// DRAWING FUNCTIONALITIES

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

// MAIN FUNCTION

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