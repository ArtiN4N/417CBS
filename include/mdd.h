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

struct MDDNode {
    int id; //Unique ID
    int timestep;
    AStarLocation nodeLoc;
    std::vector<MDDNode*> parents; // Cases where 2 or more nodes can be a parent of a node
    std::vector<MDDNode*> children;

    MDDNode(MDDNode *parent, AStarLocation loc) : nodeLoc(loc) {
        if(parent == nullptr) // no parent = root
        {
            timestep = 0;
        }
        else
        {
            timestep = parent->timestep + 1;
            parents.push_back(parent);
        }
    };
    bool operator==(const MDDNode &other) const{
        if(this->nodeLoc == other.nodeLoc && this->timestep == other.timestep){
            return true;
        }
        else{
            return false;
        }
    }
};
class MDD {
public:
    // Must maintain size of cost * vertices?
    std::vector<std::vector<MDDNode*>> mddLayers;
    int maxTimestep;
    int totalNodes;
    int lastNodeID = 0;
    // Every agent has a start and goal
    AStarLocation startNode;
    AStarLocation goalNode;
    MDD(int timestep, AStarLocation start, AStarLocation goal) : maxTimestep(timestep), startNode(start), goalNode(goal){}
    // int not void as then it can return error codes
    int createMDD(ConstraintTable ctable, Agent a);
    // Add to specific timestep, add an edge to the parent and return 0 or 1
    int addNode(MDDNode *parent, AStarLocation loc);

    // Find a specific node without know its timestep?
    MDDNode findNode(int nodeID);
    
    MDD();
    ~MDD();

};