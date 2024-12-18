#include "../include/mdd.h"
#include "../include/cbs.h"
#include <unordered_set>
#include <unordered_map>

#include <thread>
#include <cmath>

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

            int current_cost = hTable[curr->location] + curr->level;
            int next_cost = hTable[nextLocation] + curr->level + 1;

            // Check constraints and heuristic bounds
            if (isConstrained(curr->location, nextLocation, curr->level + 1, cTable) ||
                map.tiles[nextLocation.first][nextLocation.second] ||
                (goalWalls.find(nextLocation) != goalWalls.end() && goalWalls[nextLocation] <= curr->level + 1) ||
                next_cost > current_cost)
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

bool detectDependency(const MDD &mdd1, const MDD &mdd2)
{
    // Check for empty MDDs
    if (mdd1.locsAtTime.empty() || mdd2.locsAtTime.empty())
        return false; // No dependency if any MDD is empty

    int minLevels = std::min(mdd1.locsAtTime.size(), mdd2.locsAtTime.size());

    std::vector<std::vector<std::pair<MDDNode *, MDDNode *>>> jointMDD(minLevels);

    // Add the roots
    jointMDD[0].emplace_back(mdd1.locsAtTime[0].front(), mdd2.locsAtTime[0].front());

    // Traverse levels of the MDDs
    for (int level = 1; level < minLevels; level++)
    {
        for (const auto &pair : jointMDD[level - 1])
        {
            MDDNode *parent1 = pair.first;
            MDDNode *parent2 = pair.second;

            // Explore all child combinations of the current pair
            for (MDDNode *child1 : parent1->childs)
            {
                for (MDDNode *child2 : parent2->childs)
                {
                    if (child1->location != child2->location)
                    {
                        jointMDD[level].emplace_back(child1, child2);
                    }
                }
            }
        }
    }
    return jointMDD[minLevels - 1].empty();
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
    std::vector<MDD> &mdds, std::vector<AStarPath> &paths,
    std::vector<Constraint> &constraints, Map &map, std::vector<HeuristicTable> heuristics,
    uint nAgents)
{
    for (int i = 0; i < nAgents; i++)
    {
        AStarPath pathi = paths[i];
        mdds[i].createMDD(pathi[0], pathi.size(), i, constraints, map, heuristics[i]);
    }
}

void computeParMDDs(
    int start, int end, std::vector<MDD> &mdds, std::vector<AStarPath> &paths,
    std::vector<Constraint> &constraints, Map &map, std::vector<HeuristicTable> heuristics
) {
    for (int i = start; i < end; i++)
    {
        AStarPath pathi = paths[i];
        mdds[i].createMDD(pathi[0], pathi.size(), i, constraints, map, heuristics[i]);
    }
}

void createParallelMDDs(
    std::vector<MDD> &mdds, std::vector<AStarPath> &paths,
    std::vector<Constraint> &constraints, Map &map, std::vector<HeuristicTable> heuristics,
    uint nAgents, uint nthreads)
{
    std::vector<std::thread> threads;

    int agentsPerThread = std::ceil((float) nAgents / (float) nthreads);

    heuristics.resize(nAgents);

    for (int t = 0; t < nthreads; t++) {
        int start = t * agentsPerThread;
        int end = std::min((t + 1) * agentsPerThread, (int) nAgents);
        threads.push_back(std::thread(computeParMDDs, start, end, std::ref(mdds), std::ref(paths), std::ref(constraints), std::ref(map), std::ref(heuristics)));
    }

    for (auto& t : threads) {
        t.join();
    }
}

void createAllMDDs(
    std::vector<MDD> &mdds, std::vector<AStarPath> &paths,
    std::vector<Constraint> &constraints, Map &map, std::vector<HeuristicTable> heuristics,
    bool parallel, uint nthreads)
{
    if (parallel)
        createParallelMDDs(mdds, paths, constraints, map, heuristics, map.nAgents, nthreads);
    else
        createSerialMDDs(mdds, paths, constraints, map, heuristics, map.nAgents);
}

void grabSerialConflictingPairs(
    std::vector<MDD> &mdds, Map &map, std::vector<std::pair<int, int>> &conflictingAgentPairs, uint nAgents)
{
    int total = nAgents * nAgents;
    int n = total;
    for (int k = 0; k < total; k++) {
        int i = k / n;
        int j = k % n;

        if (i == j || j < i || i >= nAgents || j >= nAgents)
            continue;
        
        if (detectCardinalConflict(mdds[i], mdds[j]))
            conflictingAgentPairs[k] = std::make_pair(i, j);
    }
}

void computeConflictingPairs(int start, int end, std::vector<MDD> &mdds, std::vector<std::pair<int, int>> &conflictingAgentPairs, int nAgents) {
    int n = nAgents * nAgents;
    for (int k = start; k < end; k++) {
        int i = k / n;
        int j = k % n;

        if (i == j || j < i || i >= nAgents || j >= nAgents)
            continue;

        if (detectCardinalConflict(mdds[i], mdds[j]))
            conflictingAgentPairs[k] = std::make_pair(i, j);
    }
}


void grabParallelConflictingPairs(
    std::vector<MDD> &mdds, Map &map, std::vector<std::pair<int, int>> &conflictingAgentPairs, uint nAgents, uint nthreads)
{
    std::vector<std::thread> threads;

    int n = nAgents;
    int total = n * n;
    int perThread = std::ceil((float) total / (float) nthreads);

    for (int t = 0; t < nthreads; t++) {
        int start = t * perThread;
        int end = std::min((t + 1) * perThread, (int) total);
        threads.push_back(std::thread(computeConflictingPairs, start, end, std::ref(mdds), std::ref(conflictingAgentPairs), nAgents));
    }

    for (auto& t : threads) {
        t.join();
    }
}


void grabAllConflictingPairs(
    std::vector<MDD> &mdds, Map &map, std::vector<std::pair<int, int>> &conflictingAgentPairs, bool parallel, uint nthreads)
{
    int n = map.nAgents;
    int total = n * n;
    conflictingAgentPairs.resize(total);

    if (parallel)
        grabParallelConflictingPairs(mdds, map, conflictingAgentPairs, map.nAgents, nthreads);
    else
        grabSerialConflictingPairs(mdds, map, conflictingAgentPairs, map.nAgents);

    conflictingAgentPairs.erase(
            std::remove_if(conflictingAgentPairs.begin(), conflictingAgentPairs.end(),
                        [](const std::pair<int, int>& p) {
                            return p.first == 0 && p.second == 0;
                        }),
            conflictingAgentPairs.end());
}


void grabDGSerialConflictingPairs(
    std::vector<MDD> &mdds, Map &map, std::vector<std::pair<int, int>> &conflictingAgentPairs, uint nAgents)
{

    int total = nAgents * nAgents;
    int n = total;
    for (int k = 0; k < total; k++) {
        int i = k / n;
        int j = k % n;

        if (i == j || j < i || i >= nAgents || j >= nAgents)
            continue;

        if (detectCardinalConflict(mdds[i], mdds[j]))
            conflictingAgentPairs[k] = std::make_pair(i, j);
        else if (detectDependency(mdds[i], mdds[j]))
        { // explicitly check for dependency
            conflictingAgentPairs[k] = std::make_pair(i, j);
        }
    }
}

void computeDGConflictingPairs(int start, int end, std::vector<MDD> &mdds, std::vector<std::pair<int, int>> &conflictingAgentPairs, int nAgents) {
    int n = nAgents * nAgents;
    for (int k = start; k < end; k++) {
        int i = k / n;
        int j = k % n;

        if (i == j || j < i || i >= nAgents || j >= nAgents)
            continue;

        if (detectCardinalConflict(mdds[i], mdds[j]))
            conflictingAgentPairs[k] = std::make_pair(i, j);
        else if (detectDependency(mdds[i], mdds[j]))
        { // explicitly check for dependency
            conflictingAgentPairs[k] = std::make_pair(i, j);
        }
    }
}


void grabDGParallelConflictingPairs(
    std::vector<MDD> &mdds, Map &map, std::vector<std::pair<int, int>> &conflictingAgentPairs, uint nAgents, uint nthreads)
{
    std::vector<std::thread> threads;

    int n = nAgents;
    int total = n * n;
    int perThread = std::ceil((float) total / (float) nthreads);

    for (int t = 0; t < nthreads; t++) {
        int start = t * perThread;
        int end = std::min((t + 1) * perThread, (int) total);
        threads.push_back(std::thread(computeDGConflictingPairs, start, end, std::ref(mdds), std::ref(conflictingAgentPairs), nAgents));
    }

    for (auto& t : threads) {
        t.join();
    }
}


void grabAllDGConflictingPairs(
    std::vector<MDD> &mdds, Map &map, std::vector<std::pair<int, int>> &conflictingAgentPairs, bool parallel, uint nthreads)
{
    int n = map.nAgents;
    int total = n * n;
    conflictingAgentPairs.resize(total);

    if (parallel)
        grabDGParallelConflictingPairs(mdds, map, conflictingAgentPairs, map.nAgents, nthreads);
    else
        grabDGSerialConflictingPairs(mdds, map, conflictingAgentPairs, map.nAgents);

    conflictingAgentPairs.erase(
            std::remove_if(conflictingAgentPairs.begin(), conflictingAgentPairs.end(),
                        [](const std::pair<int, int>& p) {
                            return p.first == 0 && p.second == 0;
                        }),
            conflictingAgentPairs.end());
}

bool compareAStarNodesW(AStarNode n1, AStarNode n2) {
    return n1.gval + n1.hval < n2.gval + n2.hval;
}

AStarPath getPathW(AStarNode* goalNode) {
    AStarPath path;
    AStarNode* curr = goalNode;

    while (curr != nullptr) {
        path.push_back(curr->location);
        curr = curr->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

AStarPath aStarW(
    Map map, AStarLocation startLoc, AStarLocation goalLoc,
    HeuristicTable hTable, uint agent, std::vector<Constraint> constraints
) {

    struct CompareANode {
        bool operator()(const AStarNode *a, const AStarNode *b) {
            return a->gval + a->hval > b->gval + b->hval;
        }
    };

    std::priority_queue<int, std::vector<AStarNode *>, CompareANode> openList;
    std::unordered_map<std::pair<AStarLocation, uint>, AStarNode, PairHash> closedList;

    GoalWallTable goalWalls;

    uint hValue = hTable[startLoc];

    ConstraintTable cTable = buildConstraintTable(constraints, agent, goalWalls);
    uint maxTimestep = 0;

    for (auto &pair : cTable) {
        uint timeStep = pair.first;
        auto constraints = pair.second;

        for (auto &constraint : constraints) {

            bool isVertex = !constraint.isEdgeCollision;
            bool constraintAtGoal = constraint.l1 == goalLoc;

            if (isVertex && constraintAtGoal)
                if (timeStep > maxTimestep)
                    maxTimestep = timeStep;
        }
    }

    uint terminateTimestep = map.cols * map.rows;
    if (goalWalls.size() > 0) {
        int max_value = std::max_element(
            goalWalls.begin(), goalWalls.end(),
            [](const auto &a, const auto &b) {
                return a.second < b.second;
            }
        )->second;
        terminateTimestep += max_value + 1;
    }

    AStarNode root = {startLoc, 0, hValue, nullptr, 0};
    closedList[std::make_pair(startLoc, 0)] = root;
    AStarNode *nodePtr = &closedList[std::make_pair(startLoc, 0)];
    openList.push(nodePtr);

    while (openList.size() > 0) {
        AStarNode *curr = openList.top();
        openList.pop();

        if (curr->location == goalLoc && curr->timeStep >= maxTimestep)
            return getPathW(curr);

        for (int i = 0; i < 5; i++) {
            // cast int to ordered enum
            uint dir = i;

            bool wrongNorth = dir == 0 && curr->location.second == 0;
            bool wrongEast = dir == 1 && curr->location.first == map.cols - 1;
            bool wrongSouth = dir == 2 && curr->location.second == map.rows - 1;
            bool wrongWest = dir == 3 && curr->location.first == 0;

            if (wrongNorth || wrongEast || wrongSouth || wrongWest)
                continue;

            AStarLocation childLoc;

            if (i == 4)
                childLoc = curr->location;
            else
                childLoc = move(curr->location, dir);

            uint childTime = curr->timeStep + 1;

            if (childTime > terminateTimestep)
                continue;

            if (isConstrained(curr->location, childLoc, childTime, cTable))
                continue;

            if (map.tiles[childLoc.first][childLoc.second])
                continue;

            if (goalWalls.find(childLoc) != goalWalls.end())
                if (goalWalls[childLoc] <= childTime)
                    continue;

            AStarNode child = {
                childLoc,
                curr->gval + 1, hTable[childLoc],
                curr,
                childTime};

            if (closedList.find(std::make_pair(childLoc, childTime)) != closedList.end()) {
                AStarNode existingNode = closedList[std::make_pair(childLoc, childTime)];
                if (compareAStarNodesW(child, existingNode)) {
                    closedList[std::make_pair(childLoc, childTime)] = child;
                    AStarNode *nodePtr = &closedList[std::make_pair(childLoc, childTime)];
                    openList.push(nodePtr);
                }
            } else {
                closedList[std::make_pair(childLoc, childTime)] = child;
                AStarNode *nodePtr = &closedList[std::make_pair(childLoc, childTime)];
                openList.push(nodePtr);
            }
        }
    }

    return {};
}


uint getSumOfCostW(std::vector<AStarPath> paths) {
    uint ret = 0;
    for (auto& path : paths) ret += path.size() - 1;
    return ret;
}


int getConflictWeight(const Map &map, const std::vector<Constraint> &constraints,
                          const std::vector<AStarPath> &paths, int agent1, int agent2,
                          const std::vector<HeuristicTable> &heuristics)
{
    // Find original costs
    int cost1 = getSumOfCostW({paths[agent1]});
    int cost2 = getSumOfCostW({paths[agent2]});

    // Add constraints to resolve conflict
    std::vector<Constraint> tempConstraints = constraints;

    // Recalculate path without heuristics
    AStarPath oldPath1 = aStarW(map, map.starts[agent1], map.goals[agent1], heuristics[agent1], agent1, {});
    AStarPath oldPath2 = aStarW(map, map.starts[agent2], map.goals[agent2], heuristics[agent2], agent2, {});

    // Compute the delta cost
    int oldCost1 = getSumOfCostW({oldPath1});
    int oldCost2 = getSumOfCostW({oldPath2});

    return (cost1 + cost2) - (oldCost1 + oldCost2);
}

void grabWDGSerialConflictingPairs(
    std::vector<MDD> &mdds, std::vector<int>& weights, Map &map, std::vector<std::pair<int, int>> &conflictingAgentPairs, 
    std::vector<Constraint> &constraints, std::vector<HeuristicTable> heuristics, std::vector<AStarPath> &paths, uint nAgents
) {

    int total = nAgents * nAgents;
    int n = total;
    for (int k = 0; k < total; k++) {
        int i = k / n;
        int j = k % n;

        if (i == j || j < i || i >= nAgents || j >= nAgents)
            continue;

        if (detectCardinalConflict(mdds[i], mdds[j]))
        {
            // Same check as DG, but we need to get weights
            // Weight should be sum of increase in paths
            int weight = getConflictWeight(map, constraints, paths, i, j, heuristics);
            conflictingAgentPairs[k] = std::make_pair(i, j);
            weights[k] = weight;
        }
        else if (detectDependency(mdds[i], mdds[j]))
        { 
            // check  seperatey as to not call depenfency as often
            int weight = getConflictWeight(map, constraints, paths, i, j, heuristics);
            conflictingAgentPairs[k] = std::make_pair(i, j);
            weights[k] = weight;
        }
    }
}

void computeWDGConflictingPairs(
    int start, int end, std::vector<MDD> &mdds, std::vector<int>& weights, Map &map, std::vector<std::pair<int, int>> &conflictingAgentPairs, 
    std::vector<Constraint> &constraints, std::vector<HeuristicTable> heuristics, std::vector<AStarPath> &paths, int nAgents
) {
    int n = nAgents * nAgents;
    for (int k = start; k < end; k++) {
        int i = k / n;
        int j = k % n;

        if (i == j || j < i || i >= nAgents || j >= nAgents)
            continue;

        if (detectCardinalConflict(mdds[i], mdds[j]))
        {
            // Same check as DG, but we need to get weights
            // Weight should be sum of increase in paths
            int weight = getConflictWeight(map, constraints, paths, i, j, heuristics);
            conflictingAgentPairs[k] = std::make_pair(i, j);
            weights[k] = weight;
        }
        else if (detectDependency(mdds[i], mdds[j]))
        { 
            // check  seperatey as to not call depenfency as often
            int weight = getConflictWeight(map, constraints, paths, i, j, heuristics);
            conflictingAgentPairs[k] = std::make_pair(i, j);
            weights[k] = weight;
        }
    }
}


void grabWDGParallelConflictingPairs(
    std::vector<MDD> &mdds, std::vector<int>& weights, Map &map, std::vector<std::pair<int, int>> &conflictingAgentPairs, 
    std::vector<Constraint> &constraints, std::vector<HeuristicTable> heuristics, std::vector<AStarPath> &paths, uint nAgents, uint nthreads)
{
    std::vector<std::thread> threads;

    int n = nAgents;
    int total = n * n;
    int perThread = std::ceil((float) total / (float) nthreads);

    for (int t = 0; t < nthreads; t++) {
        int start = t * perThread;
        int end = std::min((t + 1) * perThread, (int) total);
        threads.push_back(
            std::thread(
                computeWDGConflictingPairs, start, end, std::ref(mdds), std::ref(weights),
                std::ref(map), std::ref(conflictingAgentPairs), std::ref(constraints), std::ref(heuristics),
                 std::ref(paths), nAgents
            )
        );
    }

    for (auto& t : threads) {
        t.join();
    }
}


void grabAllWDGConflictingPairs(
    std::vector<MDD> &mdds, std::vector<int>& weights, Map &map, std::vector<std::pair<int, int>> &conflictingAgentPairs, bool parallel, uint nthreads,
    std::vector<Constraint> &constraints, std::vector<HeuristicTable> heuristics, std::vector<AStarPath> &paths
) {
    int n = map.nAgents;
    int total = n * n;
    conflictingAgentPairs.resize(total);
    weights.resize(total, -1);

    if (parallel)
        grabWDGParallelConflictingPairs(mdds, weights, map, conflictingAgentPairs, constraints, heuristics, paths, map.nAgents, nthreads);
    else
        grabWDGSerialConflictingPairs(mdds, weights, map, conflictingAgentPairs, constraints, heuristics, paths, map.nAgents);

    conflictingAgentPairs.erase(
            std::remove_if(conflictingAgentPairs.begin(), conflictingAgentPairs.end(),
                        [](const std::pair<int, int>& p) {
                            return p.first == 0 && p.second == 0;
                        }),
            conflictingAgentPairs.end());
    
    weights.erase(std::remove(weights.begin(), weights.end(), -1), weights.end());
}