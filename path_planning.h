#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include "robot.h"

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

// Action costs for coverage algorithm
extern const float action_costs[4]; // Right, Forward, Left, Backward

// ========== COVERAGE PATH PLANNING FUNCTIONS ==========

// Heuristic creation
void createHeuristic(int target_x, int target_y, HeuristicType type, float heuristic[COLS][ROWS]);

// Coverage search algorithm
int coverageSearch(Robot* robot, Marker markers[], int map[COLS][ROWS], int closed[COLS][ROWS], float heuristic[COLS][ROWS]);

// A* navigation algorithm
int aStarNavigate(Robot* robot, Marker markers[], int map[COLS][ROWS], int closed[COLS][ROWS], float heuristic[COLS][ROWS],
                  int target_x, int target_y, int any_unvisited);

// Corner finding utility
int findNearestCorner(Robot robot, int* corner_x, int* corner_y);

#endif
