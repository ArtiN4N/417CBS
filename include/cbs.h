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
#include "cbstools.h"

#include "map.h"
#include "cg.h"
#include "dg.h"
#include "wdg.h"

#include "mdd.h"

std::vector<AStarPath> findSolution(
    Map map, HeuristicType type, std::string experimentName, bool PARALLELIZE = false, uint NTHREADS = 1
);