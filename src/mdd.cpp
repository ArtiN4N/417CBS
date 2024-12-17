#include "../include/mdd.h"
#include "../include/cbs.h"
#include <unordered_set>
#include <unordered_map>

// int current_cost = hTable[curr->location] + curr->level;
//         int next_cost = hTable[nextLocation] + curr->level + 1;

//         if (next_cost > current_cost) continue;

// Hash function for std::pair to use in unordered_map
struct pair_hash
{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &pair) const
    {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

bool MDD::createMDD(AStarLocation start, int time, uint agent, const std::vector<Constraint> &constraints, Map map, HeuristicTable hTable)
{
    // Build constraint and goal wall tables
    GoalWallTable goalWalls;
    ConstraintTable cTable = buildConstraintTable(constraints, agent, goalWalls);

    // Initialize root node and data structures
    MDDNode *root = new MDDNode(start, nullptr);
    root->level = 0; // Root starts at level 0
    std::queue<MDDNode *> open;
    std::list<MDDNode *> closed;

    // Track nodes by (location, level)
    std::unordered_map<std::pair<AStarLocation, int>, MDDNode *, pair_hash> nodeTable;

    open.push(root);
    closed.push_back(root);
    nodeTable[{root->location, root->level}] = root;

    locsAtTime.clear();
    locsAtTime.resize(time); // Resize levels array

    // BFS to construct the MDD
    while (!open.empty())
    {
        MDDNode *curr = open.front();
        open.pop();

        // Validate level before adding to locsAtTime
        if (curr->level < 0 || curr->level >= time)
        {
            std::cerr << "Error: Node level out of bounds!" << std::endl;
            return false;
        }

        // Add the current node to the correct level
        locsAtTime[curr->level].push_back(curr);

        // Stop expanding nodes beyond the last level
        if (curr->level == time - 1)
            continue;

        // Define the heuristic bound for pruning
        int heuristicBound = time - curr->level - 2;

        for (uint i = 0; i < 5; i++) // Check all possible moves
        {
            uint dir = i;

            // Validate moves to ensure they are within bounds
            if ((dir == 0 && curr->location.second == 0) ||            // North
                (dir == 1 && curr->location.first == map.cols - 1) ||  // East
                (dir == 2 && curr->location.second == map.rows - 1) || // South
                (dir == 3 && curr->location.first == 0))               // West
            {
                continue;
            }

            AStarLocation nextLocation = curr->location;

            if (i != 4)
                nextLocation = move(nextLocation, dir);

            // Check constraints and heuristic bounds
            if (isConstrained(curr->location, nextLocation, curr->level + 1, cTable) ||
                map.tiles[nextLocation.first][nextLocation.second] ||
                (goalWalls.find(nextLocation) != goalWalls.end() && goalWalls[nextLocation] <= curr->level + 1) ||
                hTable[nextLocation] > heuristicBound)
            {
                continue;
            }

            // Check if the node already exists using the hash map
            auto key = std::make_pair(nextLocation, curr->level + 1);
            if (nodeTable.find(key) != nodeTable.end())
            {
                MDDNode *existingNode = nodeTable[key];
                if (existingNode->level != curr->level + 1)
                {
                    std::cerr << "Error: Node level mismatch detected!" << std::endl;
                    return false;
                }
                existingNode->pars.push_back(curr);
            }
            else
            {
                // Create a new node if not found
                MDDNode *nextNode = new MDDNode(nextLocation, curr);
                nextNode->level = curr->level + 1;
                open.push(nextNode);
                closed.push_back(nextNode);
                nodeTable[key] = nextNode;
            }
        }
    }

    // Backward phase: build child relationships
    for (int t = time - 1; t > 0; t--)
    {
        for (auto &node : locsAtTime[t])
        {
            for (auto &parent : node->pars)
            {
                if (parent->childs.empty())
                {
                    if (std::find(locsAtTime[t - 1].begin(), locsAtTime[t - 1].end(), parent) == locsAtTime[t - 1].end())
                    {
                        locsAtTime[t - 1].push_back(parent);
                    }
                }
                parent->childs.push_back(node);
            }
        }
    }

    // Final level validation
    for (int t = 0; t < time; t++)
    {
        for (auto &node : locsAtTime[t])
        {
            if (node->level != t)
            {
                std::cerr << "Error: Node at incorrect level!" << std::endl;
                return false;
            }
        }
    }

    return true;
}

MDD::~MDD()
{
    std::unordered_set<MDDNode *> deletedNodes;
    for (auto &timeStep : locsAtTime)
    {
        for (auto node : timeStep)
        {
            if (deletedNodes.find(node) == deletedNodes.end())
            {
                delete node;
                deletedNodes.insert(node);
            }
        }
        timeStep.clear();
    }
    locsAtTime.clear();
}

void MDD::printMDD() const
{
    std::cout << "Multi-Level Directed Graph (MDD):\n";
    for (size_t level = 0; level < locsAtTime.size(); ++level)
    {

        std::cout << "Level " << level << ":\n";

        for (const auto *node : locsAtTime[level])
        {
            std::cout << "  Node at (" << node->location.first << ", " << node->location.second << ") ";
            std::cout << "]\n";
        }
    }
}


bool detectCardinalConflict(const MDD &mdd1, const MDD &mdd2)
{
    int levels = std::min(mdd1.locsAtTime.size(), mdd2.locsAtTime.size());

    for (int level = 0; level < levels; ++level)
    {
        // Get the list of nodes at the current level for both MDDs
        const std::list<MDDNode *> &nodes1 = mdd1.locsAtTime[level];
        const std::list<MDDNode *> &nodes2 = mdd2.locsAtTime[level];

        // Check if both levels have exactly one node
        if (nodes1.size() == 1 && nodes2.size() == 1)
        {
            // Retrieve the single nodes
            MDDNode *node1 = nodes1.front();
            MDDNode *node2 = nodes2.front();

            AStarLocation loc1 = node1->location;
            AStarLocation loc2 = node2->location;

            // Compare their locations at the same time
            if (loc1 == loc2)
            {
                return true; // Cardinal conflict detected
            }

            // Check for edge collision (starting from level 1)
            if (level > 0)
            {
                // Get the previous level's locations for both MDDs
                const std::list<MDDNode *> &prevNodes1 = mdd1.locsAtTime[level - 1];
                const std::list<MDDNode *> &prevNodes2 = mdd2.locsAtTime[level - 1];

                // Ensure both previous levels also have exactly one node
                if (prevNodes1.size() == 1 && prevNodes2.size() == 1)
                {
                    MDDNode *prevNode1 = prevNodes1.front();
                    MDDNode *prevNode2 = prevNodes2.front();

                    AStarLocation prevLoc1 = prevNode1->location;
                    AStarLocation prevLoc2 = prevNode2->location;

                    // Detect edge collision
                    if (prevLoc1 == loc2 && prevLoc2 == loc1)
                    {
                        return true; // Edge conflict detected
                    }
                }
            }
        }
    }

    // No cardinal or edge conflict found
    return false;
}

void createSerialMDDs(
	std::vector<MDD>& mdds, std::vector<AStarPath> &paths,
	std::vector<Constraint> &constraints, Map &map, std::vector<HeuristicTable> heuristics,
    uint nAgents
) {
    for (int i = 0; i < nAgents; i++) {
        AStarPath pathi = paths[i];
        mdds[i].createMDD(pathi[0], pathi.size(), i, constraints, map, heuristics[i]);
    }
}

void createParallelMDDs(
	std::vector<MDD>& mdds, std::vector<AStarPath> &paths,
	std::vector<Constraint> &constraints, Map &map, std::vector<HeuristicTable> heuristics,
    uint nAgents, uint nthreads
) {
    for (int i = 0; i < nAgents; i++) {
        AStarPath pathi = paths[i];
        mdds[i].createMDD(pathi[0], pathi.size(), i, constraints, map, heuristics[i]);
    }
}

void createAllMDDs(
	std::vector<MDD>& mdds, std::vector<AStarPath> &paths,
	std::vector<Constraint> &constraints, Map &map, std::vector<HeuristicTable> heuristics,
    bool parallel, uint nthreads
) {
    if (parallel) createParallelMDDs(mdds, paths, constraints, map, heuristics, map.nAgents, nthreads);
    else createSerialMDDs(mdds, paths, constraints, map, heuristics, map.nAgents);
}

void grabSerialConflictingPairs(
    std::vector<MDD>& mdds, Map& map, std::vector<std::pair<int, int>>& conflictingAgentPairs, uint nAgents
) {
    for (size_t i = 0; i < nAgents; i++) {
        for (size_t j = i + 1; j < nAgents; j++) {
            if (detectCardinalConflict(mdds[i], mdds[j]))
                conflictingAgentPairs.emplace_back(i, j);
        }
    }
}

void grabParallelConflictingPairs(
    std::vector<MDD>& mdds, Map& map, std::vector<std::pair<int, int>>& conflictingAgentPairs, uint nAgents, uint nthreads
) {
    for (size_t i = 0; i < nAgents; i++) {
        for (size_t j = i + 1; j < nAgents; j++) {
            if (detectCardinalConflict(mdds[i], mdds[j]))
                conflictingAgentPairs.emplace_back(i, j);
        }
    }
}

void grabAllConflictingPairs(
    std::vector<MDD>& mdds, Map& map, std::vector<std::pair<int, int>>& conflictingAgentPairs, bool parallel, uint nthreads
) {
    if (parallel) grabParallelConflictingPairs(mdds, map, conflictingAgentPairs, map.nAgents, nthreads);
    else grabSerialConflictingPairs(mdds, map, conflictingAgentPairs, map.nAgents);
}