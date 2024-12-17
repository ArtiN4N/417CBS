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


//HeuristicTable computeAstarHeuristics(AStarLocation goal, Map map);

//ConstraintTable buildConstraintTable(std::vector<Constraint> constraints, uint agent, GoalWallTable& goalWalls);

//bool isConstrained(AStarLocation currentLoc, AStarLocation nextLoc, uint nextTime, ConstraintTable cTable);

//AStarLocation move(AStarLocation location, uint dir);

//uint getSumOfCost(std::vector<AStarPath> paths);

//AStarLocation getLocation(AStarPath path, uint timestep);

//AStarPath getPath(AStarNode *goalNode);

//std::pair<bool, Collision> detectCollision(AStarPath path1, AStarPath path2, uint id1, uint id2);

//std::vector<Collision> detectCollisions(std::vector<AStarPath> paths);

//std::pair<Constraint, Constraint> standardSplitting(Collision c);

//bool compareAStarNodes(AStarNode n1, AStarNode n2);

/*AStarPath aStar(
    Map map, AStarLocation startLoc, AStarLocation goalLoc,
    HeuristicTable hTable, uint agent, std::vector<Constraint> constraints
);*/
/*
void printAStarPath(AStarPath path);

void printCollisionLocation(CollisionLocation l);

void printConstraint(Constraint c);

void printResults(std::vector<AStarPath> paths, uint nExpanded, uint nGenerated, double elapsed, std::string filename);
*/