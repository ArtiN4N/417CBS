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

#include "cbstools.h"
#include "mdd.h"

std::vector<AStarPath> findSolution(Map map, HeuristicType type, std::string experimentName);

HeuristicTable computeAstarHeuristics(HeuristicType type, AStarLocation goal, Map map);

ConstraintTable buildConstraintTable(std::vector<Constraint> constraints, uint agent, GoalWallTable& goalWalls);

bool isConstrained(AStarLocation currentLoc, AStarLocation nextLoc, uint nextTime, ConstraintTable cTable);

AStarLocation move(AStarLocation location, uint dir);
