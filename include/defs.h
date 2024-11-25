#pragma once

#include "map.h"

#include <unordered_map>
#include <vector>

typedef unsigned int uint;

struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ (std::hash<T2>()(pair.second) << 1);
    }
};

typedef std::pair<uint, uint> AStarLocation;

struct CollisionLocation {
    AStarLocation l1;

    bool isEdgeCollision;

    AStarLocation l2;
};

typedef std::vector<std::pair<uint, uint>> AStarPath;

typedef std::unordered_map<uint, std::vector<CollisionLocation>> ConstraintTable;

typedef std::unordered_map<AStarLocation, uint, PairHash> HeuristicTable;

typedef std::unordered_map<AStarLocation, uint, PairHash> GoalWallTable;

enum Direction { NORTH = 0, EAST, SOUTH, WEST, NONE };

enum HeuristicType { DEFAULT = 0, CG, DG, WDG };
