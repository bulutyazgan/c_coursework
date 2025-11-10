#ifndef ROBOT_H
#define ROBOT_H

#define ROWS 15
#define COLS 15

// Robot structure
typedef struct {
    int x;
    int y;
    int direction; // 0: up, 1: right, 2: down, 3: left
    int markerCount;
    int knowledge[ROWS][COLS]; // 0: unknown, 1: visited, 3: corner, -1: obstacle
} Robot;

// Marker structure
typedef struct {
    int x;
    int y;
    int isCarried;
} Marker;

extern const int movement[4][2];
extern const int MARKER_COUNT;

// CORE ROBOT FUNCTIONALITIES

// Basic movement functions
void forward(Robot* robot, Marker markers[], int map[COLS][ROWS]);
void left(Robot* robot, Marker markers[]);
void right(Robot* robot, Marker markers[]);
int canMoveForward(Robot robot, int map[COLS][ROWS]);

// Marker management functions
int atMarker(Robot* robot, Marker markers[], int map[COLS][ROWS]);
void pickUpMarker(Robot* robot, Marker* marker, int map[COLS][ROWS]);
void dropMarker(Robot* robot, Marker markers[], int map[COLS][ROWS]);
int markerCount(Robot robot);

// ADVANCED ROBOT FUNCTIONALITIES
void turnToDirection(Robot* robot, Marker markers[], int target_direction);
void discoverCorner(Robot* robot, Marker markers[], int map[COLS][ROWS]);
int checkAtCorner(Robot robot);
void performAction(Robot* robot, Marker markers[], int map[COLS][ROWS], int target_orientation);

#endif
