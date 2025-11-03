#include "maps.h"
#include <stdlib.h>

// TEST OBSTACLE COURSES

// Course 1: Empty Arena
void setupTestCourse1(int map[COLS][ROWS]) {
    // No obstacles - just empty arena
}

// Course 2: Irregularly Shaped Arena 
void setupTestCourse2(int map[COLS][ROWS]) {
    for (int y = 6; y < 15; y++) {
        map[0][y] = -1;
        map[1][y] = -1;
    }

    for (int x = 2; x <= 5; x++) {
        map[x][12] = -1;
        map[x][13] = -1;
        map[x][14] = -1;
    }

    map[2][11] = -1;
    map[2][10] = -1;
    map[3][10] = -1;
    map[3][9] = -1;
    map[4][9] = -1;
    map[5][9] = -1;

    map[3][0] = -1;
    for (int x = 9; x < 13; x++) {
        for (int y = 0; y < 3; y++) {
            map[x][y] = -1;
        }
    }
    map[13][1] = -1;
    map[13][2] = -1;
    map[14][2] = -1;

    int numRandomObstacles = 5 + (rand() % 4);
    int placedCount = 0;

    while (placedCount < numRandomObstacles) {
        int x = rand() % COLS;
        int y = rand() % ROWS;

        if (map[x][y] == 0 && x > 2 && x < COLS - 2 && y > 2 && y < ROWS - 3) {
            map[x][y] = -1;
            placedCount++;
        }
    }
}

// Course 3: Central Wall
void setupTestCourse3(int map[COLS][ROWS]) {
    for (int y = 1; y < 14; y++) {
        if (y != 4 && y != 9) {
            map[7][y] = -1;
        }
    }
}

// Course 4: Maze Pattern
void setupTestCourse4(int map[COLS][ROWS]) {
    for (int x = 2; x < 7; x++) map[x][3] = -1;
    for (int x = 8; x < 13; x++) map[x][6] = -1;
    for (int x = 2; x < 7; x++) map[x][9] = -1;
    for (int x = 8; x < 13; x++) map[x][12] = -1;

    for (int y = 4; y < 9; y++) map[5][y] = -1;
    for (int y = 2; y < 6; y++) map[10][y] = -1;
    for (int y = 10; y < 13; y++) map[4][y] = -1;

    for (int y = 7; y < 11; y++) map[11][y] = -1;
}

// Course 5: Scattered Obstacles
void setupTestCourse5(int map[COLS][ROWS]) {
    for (int x = 5; x <= 9; x++) {
        for (int y = 3; y <= 7; y++) {
            map[x][y] = -1;
        }
    }

    for (int x = 1; x <= 2; x++) {
        for (int y = 1; y <= 2; y++) {
            map[x][y] = -1;
        }
    }

    map[12][4] = -1; map[13][4] = -1;
    map[12][5] = -1; map[13][5] = -1;

    map[1][11] = -1; map[2][11] = -1;
    map[1][12] = -1; map[2][12] = -1;

    map[11][11] = -1; map[12][11] = -1; map[13][11] = -1;
    map[11][12] = -1; map[12][12] = -1;
}

// Course 6: U-Shape Corridor
void setupTestCourse6(int map[COLS][ROWS]) {
    for (int y = 2; y < 11; y++) {
        map[3][y] = -1;
        map[11][y] = -1;
    }

    for (int x = 3; x <= 11; x++) {
        map[x][10] = -1;
    }

    for (int x = 6; x <= 8; x++) {
        for (int y = 5; y <= 7; y++) {
            map[x][y] = -1;
        }
    }
}

// Course 7: Circular Pattern
void setupTestCourse7(int map[COLS][ROWS]) {
    int centerX = 7;
    int centerY = 7;
    double radius = 7.0;

    for (int x = 0; x < COLS; x++) {
        for (int y = 0; y < ROWS; y++) {
            double dx = x - centerX;
            double dy = y - centerY;
            double distanceSquared = (dx * dx + dy * dy); 
            double radiusSquared = radius * radius;

            if (distanceSquared > radiusSquared) {
                map[x][y] = -1;
            }
        }
    }
}
