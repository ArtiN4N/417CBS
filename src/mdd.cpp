#include "../include/mdd.h"
#include "../include/cbs.h"

int MDD::addNode(MDDNode *parent, AStarLocation loc){
    MDDNode *newNode = new MDDNode(parent, loc);
    int timestep = parent->timestep;
    if (std::find(mddLayers[timestep].begin(), mddLayers[timestep].end(), newNode) == mddLayers[timestep].end()){
        // Duplicate protection
        mddLayers[timestep].push_back(newNode);
        return 0;
    }
    else
    {
        // Duplicate trying to be added
        return 1;
    }
}