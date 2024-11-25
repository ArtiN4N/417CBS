#pragma once

#include <algorithm>
#include <queue>
#include <chrono>

#include "defs.h"

#include "cg.h"
#include "dg.h"
#include "wdg.h"
#include "map.h"

struct Collision {
    uint agentId;
    uint agentId2;

    CollisionLocation location;

    uint timeStep;

    bool isGoalWall;
};

struct Constraint {
    uint agentId;

    CollisionLocation location;

    uint timeStep;

    bool isGoalWall;
};

struct AStarNode {
    AStarLocation location;

    uint gval;
    uint hval;

    AStarNode* parent;

    uint timeStep;
};

std::vector<AStarPath> findSolution(Map map, HeuristicType type);

HeuristicTable computeHeuristics(HeuristicType type, AStarLocation goal, Map map);