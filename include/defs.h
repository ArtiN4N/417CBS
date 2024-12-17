#pragma once

#include <unordered_map>
#include <vector>
#include <functional>

typedef unsigned int uint;

struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator () (const std::pair<T1, T2>& pair) const {
        auto h1 = std::hash<T1>{}(pair.first);  // hash of the first element
        auto h2 = std::hash<T2>{}(pair.second); // hash of the second element
        return h1 ^ (h2 << 1); // Combine the two hashes
    }
};

// Specializing std::hash for std::pair<unsigned int, unsigned int>
template <>
struct std::hash<std::pair<unsigned int, unsigned int>> {
    std::size_t operator()(const std::pair<unsigned int, unsigned int>& key) const {
        auto h1 = std::hash<unsigned int>{}(key.first);
        auto h2 = std::hash<unsigned int>{}(key.second);
        return h1 ^ (h2 << 1); // Combine the hashes
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

enum HeuristicType { DEFAULT = 0, CG, DG, WDG };
