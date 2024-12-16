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

struct MDDNode{
    int id; //Unique ID
    int timestep;
    std::vector<MDDNode*> parents; // Cases where 2 or more nodes can be a parent of a node
    std::vector<MDDNode*> children;

    MDDNode(int nodeID, int timestepID) : id(nodeID), timestep(timestepID) {}

};
struct MDD{

};