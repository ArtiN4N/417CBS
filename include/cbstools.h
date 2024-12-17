#pragma once

#include <algorithm>
#include <queue>
#include <chrono>
#include <climits>
#include <set>

#include <iostream>
#include <fstream>
#include <string>

#include "defs.h"

#include "map.h"
#include "cg.h"
#include "dg.h"
#include "wdg.h"

struct Collision
{
    uint agentId;
    uint agentId2;

    CollisionLocation location;

    uint timeStep;

    bool isGoalWall;

    bool operator==(const Collision& other) const {
        return agentId == other.agentId && agentId2 == other.agentId2 && timeStep == other.timeStep && isGoalWall == other.isGoalWall && location == other.location;
    }
};

struct Constraint
{
    uint agentId;

    CollisionLocation location;

    uint timeStep;

    bool isGoalWall;

    bool operator==(const Constraint &other) const
    {
        bool idEqual = this->agentId == other.agentId;
        bool timeEqual = this->timeStep == other.timeStep;
        bool typeEqual = this->location.isEdgeCollision == other.location.isEdgeCollision;
        bool l1Equal = this->location.l1 == other.location.l1;
        bool l2Equal = this->location.l2 == other.location.l2;
        return (idEqual && timeEqual && typeEqual && l1Equal && l2Equal);
    }
};

struct AStarNode
{
    AStarLocation location;

    uint gval;
    uint hval;

    AStarNode *parent;

    uint timeStep;
};

HeuristicTable computeAstarHeuristics(AStarLocation goal, Map map);

ConstraintTable buildConstraintTable(std::vector<Constraint> constraints, uint agent, GoalWallTable goalWalls);

bool isConstrained(AStarLocation currentLoc, AStarLocation nextLoc, uint nextTime, ConstraintTable cTable);

AStarLocation move(AStarLocation location, uint dir);

uint getSumOfCost(std::vector<AStarPath> paths);

AStarLocation getLocation(AStarPath path, uint timestep);

AStarPath getPath(AStarNode *goalNode);

std::pair<bool, Collision> detectCollision(AStarPath path1, AStarPath path2, uint id1, uint id2);

std::vector<Collision> detectCollisions(std::vector<AStarPath> paths, bool parallel, uint nthreads);

std::pair<Constraint, Constraint> standardSplitting(Collision c);

bool compareAStarNodes(AStarNode n1, AStarNode n2);

AStarPath aStar(
    Map map, AStarLocation startLoc, AStarLocation goalLoc,
    HeuristicTable hTable, uint agent, std::vector<Constraint> constraints
);

void printAStarPath(AStarPath path);

void printCollisionLocation(CollisionLocation l);

void printConstraint(Constraint c);

void printResults(std::vector<AStarPath> paths, uint nExpanded, uint nGenerated, double elapsed, std::string filename);
*/