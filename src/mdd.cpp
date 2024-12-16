#include "../include/mdd.h"
#include "../include/cbs.h"

int MDD::addNode(MDDNode *parent, AStarLocation loc){
    MDDNode *newNode = new MDDNode(parent, loc);
    int timestep = parent->timestep;
    
}