#ifndef MAPS_H
#define MAPS_H

#define ROWS 15
#define COLS 15

// TEST OBSTACLE COURSE CONFIGURATIONS

// Course 1: Empty Arena
void setupTestCourse1(int map[COLS][ROWS]);

// Course 2: Irregularly Shaped Arena
void setupTestCourse2(int map[COLS][ROWS]);

// Course 3: Central Wall
void setupTestCourse3(int map[COLS][ROWS]);

// Course 4: Maze Pattern
void setupTestCourse4(int map[COLS][ROWS]);

// Course 5: Scattered Obstacles
void setupTestCourse5(int map[COLS][ROWS]);

// Course 6: U-Shape Corridor
void setupTestCourse6(int map[COLS][ROWS]);

// Course 7: Circular Free Space Pattern
void setupTestCourse7(int map[COLS][ROWS]);

#endif
