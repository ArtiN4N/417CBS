#pragma once

#include "map.h"

#include <unordered_map>
#include <vector>

typedef unsigned int uint;

typedef std::pair<uint, uint> AStarLocation;

struct CollisionLocation {
    AStarLocation l1;

    bool isEdgeCollision;

    AStarLocation l2;
};

typedef std::vector<std::pair<uint, uint>> AStarPath;

typedef std::unordered_map<uint, std::vector<CollisionLocation>> ConstraintTable;

typedef std::unordered_map<AStarLocation, uint> HeuristicTable;

typedef std::unordered_map<AStarLocation, uint> GoalWallTable;

enum Direction { NORTH = 0, EAST, SOUTH, WEST, NONE };

enum HeuristicType { DEFAULT = 0, CG, DG, WDG };
