#include "graphics.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#define ROWS 15
#define COLS 15
#define GRID_SIZE 40
#define WINDOW_WIDTH (COLS * GRID_SIZE)
#define WINDOW_HEIGHT (ROWS * GRID_SIZE)
#define MARKER_COUNT 5
#define DELAY 50

typedef struct Robot {
        int x;
        int y;
        int direction; // 0: up, 1: right, 2: down, 3: left
        int markerCount;
} Robot;

typedef struct Marker {
    int x;
    int y;
    int isCarried;
} Marker;

void drawArena();
void drawMarkers(Marker markers[]);

void drawRobot(Robot robot);
void forward(Robot* robot, Marker markers[], int markerMap[COLS][ROWS]);
void left(Robot* robot, Marker markers[]);
void right(Robot* robot, Marker markers[]);
int canMoveForward(Robot robot, int markerMap[COLS][ROWS]);
int atMarker(Robot* robot, Marker markers[], int markerMap[COLS][ROWS]);
void pickUpMarker(Robot* robot, Marker* marker, int markerMap[COLS][ROWS]);
void dropMarker(Robot* robot, Marker markers[], int markerMap[COLS][ROWS]);
int checkAtCorner(Robot robot); // gotta change the implementation of corner checks, robot shouldn't know the arena borders beforehand.
int markerCount(Robot robot);
void drawMovingObjects(Robot robot, Marker markers[]);

int main(int argc, char const *argv[])
{
    srand(time(NULL));  // random seed generator

    Robot robot = {rand() % COLS, rand() % ROWS, 0}; //position in terms of grid coordinates, not pixels
    Marker markers[MARKER_COUNT];
    int markerMap[COLS][ROWS] = {0}; // 0 = empty, the rest are marker index + 1, this 2D array is only for marker location checks
    for (int i = 0; i < MARKER_COUNT; i++) {
        int x = rand() % COLS;
        int y = rand() % ROWS;
        markers[i].x = x;
        markers[i].y = y;
        markers[i].isCarried = 0;
        markerMap[x][y] = i + 1; // Store marker index + 1 (so 0 means empty)
    }

    setWindowSize(WINDOW_WIDTH + 1, WINDOW_HEIGHT + 1);

    background();
    drawArena();

    foreground();
    drawRobot(robot);
    drawMarkers(markers);
    while (1){

        // robot mechanics
        if (canMoveForward(robot, markerMap)){
            forward(&robot, markers, markerMap);
        }
        else{
            if (checkAtCorner(robot))
            {
                right(&robot, markers);
                if (robot.direction % 4 == 1 && canMoveForward(robot, markerMap)){
                    forward(&robot, markers, markerMap);
                    right(&robot, markers);
                }
            }
            else if (robot.direction % 4 == 0){
                right(&robot, markers);
                if (canMoveForward(robot, markerMap)){
                    forward(&robot, markers, markerMap);
                }
                right(&robot, markers);
            }
            else if (robot.direction % 4 == 2){
                left(&robot, markers);
                if (canMoveForward(robot, markerMap))
                {
                    forward(&robot, markers, markerMap);
                }
                left(&robot, markers);
            }
        }
        // robot mechanics end

    }
    return 0;
}

void drawArena(){
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

void forward(Robot* robot, Marker markers[], int markerMap[COLS][ROWS]){
    switch (robot->direction){
        case 0: //up
            robot->y -= 1;
            break;
        case 1: //right
            robot->x += 1;
            break;
        case 2: //down
            robot->y += 1;
            break;
        case 3: //left
            robot->x -= 1;
            break;
    }
    for (int i = 0; i < MARKER_COUNT; i++) { // Update positions of carried markers
        if (markers[i].isCarried) {
            markers[i].x = robot->x;
            markers[i].y = robot->y;
        }
    }
    if (atMarker(robot, markers, markerMap)){ //check if robot is at a marker after moving
        pickUpMarker(robot, &markers[markerMap[robot->x][robot->y] - 1], markerMap); // deduct 1 to get the correct index
    }
    if (markerCount(*robot) > 0 && checkAtCorner(*robot)) {
        dropMarker(robot, markers, markerMap);
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

int canMoveForward(Robot robot, int markerMap[COLS][ROWS]){
    switch (robot.direction){
        case 0: //up
            return robot.y != 0;
        case 1: //right
            return robot.x != COLS - 1;
        case 2: //down
            return robot.y != ROWS - 1;
        case 3: //left
            return robot.x != 0;
    }
    return 0;
}

int checkAtCorner(Robot robot){
    return (robot.x == 0 && robot.y == 0) ||
    (robot.x == COLS - 1 && robot.y == 0) ||
    (robot.x == 0 && robot.y == ROWS - 1) ||
    (robot.x == COLS - 1 && robot.y == ROWS - 1);
}

int markerCount(Robot robot){
    return robot.markerCount;
}

int atMarker(Robot* robot, Marker markers[], int markerMap[COLS][ROWS]){
    if (markerMap[robot->x][robot->y] > 0 && !checkAtCorner(*robot)) { // markerMap[robot->x][robot->y] > 0 means there's a marker
        return 1;
    }
    return 0;
}

void pickUpMarker(Robot* robot, Marker* marker, int markerMap[COLS][ROWS]) {
    marker->isCarried = 1;
    robot->markerCount += 1;
    markerMap[marker->x][marker->y] = 0; // Remove marker from markerMap
}

void dropMarker(Robot* robot, Marker markers[], int markerMap[COLS][ROWS]){
    for (int i = 0; i < MARKER_COUNT; i++) { // this function gets called when the robot is at the corner and has at least one marker, and the function drops all the markers
        if (markers[i].isCarried) {
            markers[i].isCarried = 0;
            robot->markerCount -= 1;
            markerMap[markers[i].x][markers[i].y] = i + 1; // Store marker index + 1 back in the map
        }
    }
}