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

int MDD::createMDD(ConstraintTable cTable, Agent agent1, Map map){
    std::queue<MDDNode*> openList;
    std::vector<MDDNode*> closedList;
    MDDNode *root = new MDDNode(nullptr, agent1.start);
    openList.push(root);
	closedList.push_back(root);
    while(openList.size() > 0){
        MDDNode *curr = openList.front();
        openList.pop();
        
    }

}