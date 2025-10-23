#include "graphics.h"
#include <stdlib.h>
#include <stdio.h>


#define ROWS 15
#define COLS 15
#define GRID_SIZE 40
#define WINDOW_WIDTH (COLS * GRID_SIZE)
#define WINDOW_HEIGHT (ROWS * GRID_SIZE)
#define MARKER_COUNT 1

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
void forward(struct Robot* robot);
void left(struct Robot* robot);
void right(struct Robot* robot);
int canMoveForward(struct Robot robot);
int atMarker(struct Robot robot, struct Marker markers[]);
void pickUpMarker(struct Marker* marker);
void dropMarker(struct Marker* marker);

int main(int argc, char const *argv[])
{
    struct Robot robot = {5, 5, 0}; //position in terms of grid coordinates, not pixels
    struct Marker markers[MARKER_COUNT];
    for (int i = 0; i < MARKER_COUNT; i++) {
        markers[i].x = ROWS - 1;
        markers[i].y = 2;
        markers[i].isCarried = 0;
    }

    setWindowSize(WINDOW_WIDTH + 1, WINDOW_HEIGHT + 1);

    background();
    drawArena();

    foreground();
    drawRobot(robot);
    drawMarkers(markers);
    while (1){
        clear();
        if (canMoveForward(robot)){
            forward(&robot);
            for (int i = 0; i < MARKER_COUNT; i++) {
                if (markers[i].isCarried) {
                    markers[i].x = robot.x;
                    markers[i].y = robot.y;
                }
            }
            if (atMarker(robot, markers)){
                robot.markerCount += 1;
                // pickUpMarker() gets called inside of the atMarker() function for optimization, so we don't need to do a linear search again to find out which marker the robot is standing on
            }
        }
        else{
            right(&robot);
            for (int i = 0; i < MARKER_COUNT; i++) {
                if (markers[i].isCarried) {
                    dropMarker(&markers[i]);
                    robot.markerCount -= 1;
                }
            }
        }
        drawRobot(robot);
        drawMarkers(markers);
        sleep(500);
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

void forward(struct Robot* robot){
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
}

void left(struct Robot* robot){
    robot->direction = (robot->direction + 3) % 4; //turn left (suggestion made by github copilot, modulo loop)
}
void right(struct Robot* robot){
    robot->direction = (robot->direction + 1) % 4; //turn right (suggestion made by github copilot, modulo loop)
}

int canMoveForward(struct Robot robot){
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

int markerCount(struct Robot robot){
    return robot.markerCount;
}

int atMarker(struct Robot robot, struct Marker markers[]){
    for (int i = 0; i < MARKER_COUNT; i++) {
        if (robot.x == markers[i].x && robot.y == markers[i].y) {
            pickUpMarker(&markers[i]);
            return 1;
        }
    }
    return 0;
}

void pickUpMarker(struct Marker* marker) {
    marker->isCarried = 1;
}

void dropMarker(struct Marker* marker){
    marker->isCarried = 0;
}