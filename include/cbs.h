#pragma once

#include <algorithm>
#include <queue>
#include <chrono>

#include "defs.h"

#include "map.h"
#include "cg.h"
#include "dg.h"
#include "wdg.h"

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

    bool operator==(const Constraint& other) const {
        bool idEqual = this->agentId == other.agentId;
        bool timeEqual = this->timeStep == other.timeStep;
        bool typeEqual = this->location.isEdgeCollision == other.location.isEdgeCollision;
        bool l1Equal = this->location.l1 == other.location.l1;
        bool l2Equal = this->location.l2 == other.location.l2;
        return (idEqual && timeEqual && typeEqual && l1Equal && l2Equal);
    }
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