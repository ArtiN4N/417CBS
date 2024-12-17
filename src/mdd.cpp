#include "../include/mdd.h"
#include "../include/cbs.h"
#include <unordered_set>


bool MDD::createMDD(AStarLocation start, int time, uint agent, const std::vector<Constraint>& constraints, Map map, HeuristicTable hTable) {

    GoalWallTable goalWalls;
    ConstraintTable cTable = buildConstraintTable(constraints, agent, goalWalls);

    MDDNode* root = new MDDNode(start, NULL);

	std::queue<MDDNode*> open;
	std::list<MDDNode*> closed;

	open.push(root);
	closed.push_back(root);

	locsAtTime.resize(time + 1);

	while (!open.empty()) {
        MDDNode* curr = open.front();
		open.pop();

		if (curr->level == time) {
			locsAtTime[time].push_back(curr);
			continue;
		}

        for (uint i = 0; i < 5; i++)
		{
            uint dir = i;

            bool wrongNorth = dir == 0 && curr->location.second == 0;
            bool wrongEast = dir == 1 && curr->location.first == map.cols - 1;
            bool wrongSouth = dir == 2 && curr->location.second == map.rows - 1;
            bool wrongWest = dir == 3 && curr->location.first == 0;

            if (wrongNorth || wrongEast || wrongSouth || wrongWest) continue;

			AStarLocation nextLocation = curr->location;

            if (i != 4) nextLocation = move(nextLocation, dir);

            if (isConstrained(curr->location, nextLocation, curr->level + 1, cTable)) continue;

            if (map.tiles[nextLocation.first][nextLocation.second]) continue;

            if (goalWalls.find(nextLocation) != goalWalls.end())
                if (goalWalls[nextLocation] <= curr->level + 1)
                    continue;

            int current_cost = hTable[curr->location] + curr->level;
            int next_cost = hTable[nextLocation] + curr->level + 1;

            if (next_cost > current_cost) continue;

            // Check if the node already exists
            bool found = false;
            for (MDDNode* node : closed) {
                if (node->level == curr->level + 1 && node->location == nextLocation) {
                    node->pars.push_back(curr);
                    curr->childs.push_back(node);
                    found = true;
                    break;
                }
            }

              // If not found, create a new node
            if (!found) {
                MDDNode* nextNode = new MDDNode(nextLocation, curr);
                nextNode->level = curr->level + 1; // Update the level of the new node
                open.push(nextNode);
                closed.push_back(nextNode);
                curr->childs.push_back(nextNode);
            }        
		}
    }

     // Add nodes to the levels structure
    if (!closed.empty()) {
        for (MDDNode* node : closed) {
            if (node->level <= time) {
                locsAtTime[node->level].push_back(node);
            }
        }
    }
   

    std::unordered_set<MDDNode*> deletedNodes; // Tracks nodes that have been deleted

    // Cleanup: Remove unused nodes
    for (auto it = closed.begin(); it != closed.end();) {
        if ((*it)->childs.empty() && (*it)->level < time) {
            if (deletedNodes.find(*it) == deletedNodes.end()) { // Avoid deleting twice
                delete *it;
                deletedNodes.insert(*it);
            }
            it = closed.erase(it);
        } else {
            ++it;
        }
    }


    return true;


}


MDD::~MDD() {
    std::unordered_set<MDDNode*> deletedNodes; 
    for (auto& timeStep : locsAtTime) {      
        for (auto node : timeStep) {        
            if (deletedNodes.find(node) == deletedNodes.end()) { 
                delete node;                 
                deletedNodes.insert(node);   
            }
        }
        timeStep.clear();                  
    }
    locsAtTime.clear();                     
}




void MDD::printMDD() const {
    std::cout << "Multi-Level Directed Graph (MDD):\n";
    for (size_t level = 0; level < locsAtTime.size(); ++level) {

        std::cout << "Level " << level << ":\n";

        for (const auto* node : locsAtTime[level]) {
            std::cout << "  Node at (" << node->location.first << ", " << node->location.second << ") ";
            std::cout << "]\n";
        }
    }
}
