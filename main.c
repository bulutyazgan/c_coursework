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

struct Robot {
        int x;
        int y;
        int direction; // 0: up, 1: right, 2: down, 3: left
        int markerCount;
};

struct Marker {
    int x;
    int y;
    int isCarried;
};

void drawArena();
void drawMarkers(struct Marker markers[]);

void drawRobot(struct Robot robot);
void forward(struct Robot* robot, struct Marker markers[], int map[COLS][ROWS]);
void left(struct Robot* robot, struct Marker markers[]);
void right(struct Robot* robot, struct Marker markers[]);
int canMoveForward(struct Robot robot, int map[COLS][ROWS]);
int atMarker(struct Robot* robot, struct Marker markers[], int map[COLS][ROWS]);
void pickUpMarker(struct Robot* robot,struct Marker* marker);
void dropMarker(struct Robot* robot, struct Marker markers[]);
int checkAtCorner(struct Robot robot);
int markerCount(struct Robot robot);
void drawMovingObjects(struct Robot robot, struct Marker markers[]);

int main(int argc, char const *argv[])
{
    srand(time(NULL));  // random seed generator
    
    struct Robot robot = {rand() % COLS, rand() % ROWS, 0}; //position in terms of grid coordinates, not pixels
    struct Marker markers[MARKER_COUNT];
    int map[COLS][ROWS] = {0}; // 0 = empty, 1 = marker, 2 = obstacle, this 2d array is only for targeted location checks
    for (int i = 0; i < MARKER_COUNT; i++) {
        int x = rand() % COLS;
        int y = rand() % ROWS;
        markers[i].x = x;
        markers[i].y = y;
        markers[i].isCarried = 0;
        map[x][y] = 1; // Update the map to place the marker
    }

    setWindowSize(WINDOW_WIDTH + 1, WINDOW_HEIGHT + 1);

    background();
    drawArena();

    foreground();
    drawRobot(robot);
    drawMarkers(markers);
    while (1){

        // robot mechanics
        if (canMoveForward(robot, map)){
            forward(&robot, markers, map);
        }
        else{
            left(&robot, markers);
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

void drawMarkers(struct Marker markers[]){
    setColour(gray);
    for (int i = 0; i < MARKER_COUNT; i++) {
        fillArc(markers[i].x * GRID_SIZE + GRID_SIZE / 4, markers[i].y * GRID_SIZE + GRID_SIZE / 4, GRID_SIZE / 2, GRID_SIZE / 2, 0, 360);
    }
}

void drawRobot(struct Robot robot){
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

void drawMovingObjects(struct Robot robot, struct Marker markers[]){
    clear(); // everytime there's a movement, update the elements that moved
    drawRobot(robot);
    drawMarkers(markers);
    sleep(300);
}

void forward(struct Robot* robot, struct Marker markers[], int map[COLS][ROWS]){
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
    if (atMarker(robot, markers, map)){ //check if robot is at a marker after moving
                // pickUpMarker() gets called inside of the atMarker() function for optimization, so we don't need to do a linear search again to find out which marker the robot is standing on
            }
    if (markerCount(*robot) > 0 && checkAtCorner(*robot)) {
                dropMarker(robot, markers);
            }
    drawMovingObjects(*robot, markers);
}

void left(struct Robot* robot, struct Marker markers[]){
    robot->direction = (robot->direction + 3) % 4; //turn left (suggestion made by github copilot, modulo loop)
    drawMovingObjects(*robot, markers);
}
void right(struct Robot* robot, struct Marker markers[]){
    robot->direction = (robot->direction + 1) % 4; //turn right (suggestion made by github copilot, modulo loop)
    drawMovingObjects(*robot, markers);
}

int canMoveForward(struct Robot robot, int map[COLS][ROWS]){
    switch (robot.direction){
        case 0: //up
            return robot.y != 0 && map[robot.x][robot.y - 1] != 2;
        case 1: //right
            return robot.x != COLS - 1 && map[robot.x + 1][robot.y] != 2;
        case 2: //down
            return robot.y != ROWS - 1 && map[robot.x][robot.y + 1] != 2;
        case 3: //left
            return robot.x != 0 && map[robot.x - 1][robot.y] != 2;
    }
    return 0;
}

int checkAtCorner(struct Robot robot){
    return (robot.x == 0 && robot.y == 0) ||
    (robot.x == COLS - 1 && robot.y == 0) ||
    (robot.x == 0 && robot.y == ROWS - 1) ||
    (robot.x == COLS - 1 && robot.y == ROWS - 1);
}

int markerCount(struct Robot robot){
    return robot.markerCount;
}

int atMarker(struct Robot* robot, struct Marker markers[], int map[COLS][ROWS]){
    if (map[robot->x][robot->y] == 1 && !checkAtCorner(*robot)) // check if there's a marker at the robot's position
    // also checking if the marker is in the corner already. This actually isn't necessary because they would be dropped back instantly anyways, but I still checked to keep the simulation realistic
    {
        for (int i = 0; i < MARKER_COUNT; i++) { // if there is a marker, find which marker in the list. This wouldn't be necessary if we stored the addresses to the markers inside the map array, but the marker count isn't too high anyways so I went with the basic way.
            if (robot->x == markers[i].x && robot->y == markers[i].y) {
                pickUpMarker(robot, &markers[i]);
                return 1; // return that there is a marker in the location (corner markers are ignored as they're already in the right place)
            }
        }
    }
    return 0;
}

void pickUpMarker(struct Robot* robot, struct Marker* marker) {
    marker->isCarried = 1;
    robot->markerCount += 1;
}

void dropMarker(struct Robot* robot, struct Marker markers[]){
    for (int i = 0; i < MARKER_COUNT; i++) {
        if (markers[i].isCarried) {
            markers[i].isCarried = 0;
            robot->markerCount -= 1;
        }
    }
}