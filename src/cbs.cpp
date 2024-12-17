#include "../include/cbs.h"

bool PARALLELIZE = false;
uint nThreads = 1;

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

            // Compare their locations
            if (node1->location == node2->location)
            {
                return true; // Cardinal conflict detected
            }
        }
    }

    // No cardinal conflict found
    return false;
}

// Check if a set of vertices form a vertex cover
bool isVertexCover(const std::vector<std::pair<int, int>> &edges, const std::set<int> &vertices)
{
    for (const auto &edge : edges)
    {
        if (vertices.find(edge.first) == vertices.end() &&
            vertices.find(edge.second) == vertices.end())
        {
            return false;
        }
    }
    return true;
}

// Recursive function to find the minimum vertex cover
void findMinimumVertexCover(const std::vector<std::pair<int, int>> &edges, std::set<int> &currentCover,
                            std::set<int> &bestCover, const std::set<int> &vertices, std::set<int>::iterator it)
{
    if (it == vertices.end())
    {
        // If all vertices have been considered, check if currentCover is a valid cover
        if (isVertexCover(edges, currentCover) &&
            (bestCover.empty() || currentCover.size() < bestCover.size()))
        {
            bestCover = currentCover; // Update the best solution
        }
        return;
    }

    currentCover.insert(*it);
    findMinimumVertexCover(edges, currentCover, bestCover, vertices, std::next(it));
    currentCover.erase(*it);

    findMinimumVertexCover(edges, currentCover, bestCover, vertices, std::next(it));
}

int minimumVertexCover(const std::vector<std::pair<int, int>> &edges)
{
    std::set<int> vertices;
    for (const auto &edge : edges)
    {
        vertices.insert(edge.first);
        vertices.insert(edge.second);
    }

    std::set<int> currentCover;
    std::set<int> bestCover;

    findMinimumVertexCover(edges, currentCover, bestCover, vertices, vertices.begin());

    return bestCover.size();
}

/*
int minimumWeightedVertexCover(std::vector<int>& HG, int nAgents) {
    int ret = 0;
    std::vector<bool> finished(nAgents, false);

    for (int i = 0; i < nAgents; i++) {
        if (finished[i]) continue;

        std::queue<int> wvcq;
        wvcq.push(i);
        finished[i] = true;
    }
    return ret;
}
*/

/*
std::vector<AStarPath> aStarforHeurs(
    Map map, AStarLocation startLoc, AStarLocation goalLoc,
    HeuristicTable hTable, uint agent, std::vector<Constraint> constraints)
{

    struct CompareANode
    {
        bool operator()(const AStarNode *a, const AStarNode *b)
        {
            return a->gval + a->hval > b->gval + b->hval;
        }
    };

    std::priority_queue<int, std::vector<AStarNode *>, CompareANode> openList;
    std::unordered_map<std::pair<AStarLocation, uint>, AStarNode, PairHash> closedList;

    GoalWallTable goalWalls;

    uint hValue = hTable[startLoc];

    ConstraintTable cTable = buildConstraintTable(constraints, agent, goalWalls);
    uint maxTimestep = 0;

    for (auto &pair : cTable)
    {
        uint timeStep = pair.first;
        auto constraints = pair.second;

        for (auto &constraint : constraints)
        {
            bool isVertex = !constraint.isEdgeCollision;
            bool constraintAtGoal = constraint.l1 == goalLoc;
            if (isVertex && constraintAtGoal)
                if (timeStep > maxTimestep)
                    maxTimestep = timeStep;
        }
    }

    uint terminateTimestep = map.cols * map.rows;
    if (goalWalls.size() > 0)
    {
        int max_value = std::max_element(
                            goalWalls.begin(), goalWalls.end(),
                            [](const auto &a, const auto &b)
                            {
                                return a.second < b.second;
                            })
                            ->second;
        terminateTimestep += max_value + 1;
    }

    AStarNode root = {startLoc, 0, hValue, nullptr, 0};
    closedList[std::make_pair(startLoc, 0)] = root;
    AStarNode *nodePtr = &closedList[std::make_pair(startLoc, 0)];
    openList.push(nodePtr);

    std::vector<AStarPath> shortestPaths;
    int shortestCost = INT_MAX;

    //std::cout << "maxtimestep = " << maxTimestep << "\n";

    while (openList.size() > 0)
    {
        AStarNode *curr = openList.top();
        openList.pop();

        AStarPath cpath = getPath(curr);
        int cpathCost = cpath.size();
        if (cpathCost > shortestCost) continue;
        std::cout << "cheaper than shortest cost " << shortestCost << "\n";
        //printAStarPath(cpath);

        if (curr->location == goalLoc && curr->timeStep >= maxTimestep)
        {
            //std::cout << "found a new path\n";
            

            // Determine the path cost (length of the path)
            

            if (cpathCost < shortestCost) {
                shortestPaths.clear();
                //std::cout << "found a new shortest path\n";
            }

            if (cpathCost <= shortestCost){
                //std::cout << "found an equivalent shortest path\n";
                shortestCost = cpathCost;//pathCost;
                shortestPaths.push_back(cpath);
                //std::cout << "shortest path cost is now " << shortestCost << "\n";

                // for (const auto& path : shortestPaths) {
                //     std::cout << "Path for " << agent << ": ";
                //     for (const auto& loc : path) {
                //         std::cout << "(" << loc.first << ", " << loc.second << ") ";
                //     }
                //     std::cout << std::endl;
                // }
            }
            continue;
        }

        for (int i = 0; i < 5; i++)
        {
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

            if (closedList.find(std::make_pair(childLoc, childTime)) != closedList.end() && false)
            {
                AStarNode existingNode = closedList[std::make_pair(childLoc, childTime)];
                if (compareAStarNodes(child, existingNode))
                {
                    closedList[std::make_pair(childLoc, childTime)] = child;
                    AStarNode *nodePtr = &closedList[std::make_pair(childLoc, childTime)];
                    openList.push(nodePtr);
                }
            }
            else
            {
                closedList[std::make_pair(childLoc, childTime)] = child;
                AStarNode *nodePtr = &closedList[std::make_pair(childLoc, childTime)];
                openList.push(nodePtr);
            }
        }
    }

    return shortestPaths;
}*/

int computeCGHeuristic(const Map &map, const std::vector<Constraint> &constraints, const std::vector<AStarPath> &paths, std::vector<HeuristicTable> heuristics)
{
    std::vector<std::pair<int, int>> conflictingAgentPairs;
    std::vector<MDD> mdds(map.nAgents); // Pre-allocate memory for MDDs

    // Create MDDs for all agents
    // PARALLELIZE HERE
    for (int i = 0; i < map.nAgents; i++)
    {
        AStarPath pathi = paths[i];
        mdds[i].createMDD(pathi[0], pathi.size(), i, constraints, map, heuristics[i]);
    }
    //////////////

    // Check for conflicts between agent pairs
    // PARALLELIZE HERE
    for (size_t i = 0; i < map.nAgents; i++)
    {
        for (size_t j = i + 1; j < map.nAgents; j++)
        {
            if (detectCardinalConflict(mdds[i], mdds[j]))
            {
                // Store the pair of conflicting agents
                conflictingAgentPairs.emplace_back(i, j);
            }
        }
    }
    ////////////

    // Compute the minimum vertex cover for the conflict graph
    return minimumVertexCover(conflictingAgentPairs);
}

// bool detectDependecy(const std::vector<AStarPath>& paths1, const std::vector<AStarPath>& paths2) {
//     for (uint path1 = 0; path1 < paths1.size(); path1++) {
//         for (uint path2 = 0; path2 < paths2.size(); path2++) {

//             size_t maxTimestep = 0;
//             for (const auto& path : paths1) {
//                 maxTimestep = std::max(maxTimestep, path.size());
//             }
//             for (const auto& path : paths2) {
//                 maxTimestep = std::max(maxTimestep, path.size());
//             }

//             bool conflict = false;

//             for (uint t = 0; t < maxTimestep; t++) {
//                 if (getLocation(paths1[path1], t) == getLocation(paths2[path2],t)) {
//                     conflict = true;
//                     break;
//                 }
//             }

//             if (!conflict) { // if we go through a pair of paths with no conflicts then these agents arent dependent
//                 return false;
//             }
//         }
//     }
//     return true;
// }

// int computeDGHeuristic(const Map& map, const std::vector<HeuristicTable>& heuristics, const std::vector<Constraint>& constraints) {
//     std::unordered_map<int, std::vector<AStarPath>> agentPaths;
//     // list to store pairs of agents with cardinal conflicts
//     std::vector<std::pair<int, int>> conflictingAgentPairs;

//     for (int a = 0; a < map.nAgents; ++a) {
//         agentPaths[a] = aStar(map, map.starts[a], map.goals[a], heuristics[a], a, constraints);
//     }

//       for (size_t i = 0; i < map.nAgents; i++) {
//         for (size_t j = i + 1; j < map.nAgents; j++) {

//             const std::vector<AStarPath> paths1 = agentPaths[i];
//             const std::vector<AStarPath> paths2 = agentPaths[j];

//             if (detectCardinalConflict(paths1, paths2)) { // if they have cardinal conflict they are dependent
//                 conflictingAgentPairs.emplace_back(i, j);
//             } else if (detectDependecy(paths1, paths2)) { // explicitly check for dependency
//                 conflictingAgentPairs.emplace_back(i, j);
//             }
//         }
//     }

//        // Print conflicting agent pairs
//     std::cout << "Conflicting Agent Pairs:" << std::endl;
//     for (const auto& pair : conflictingAgentPairs) {
//         std::cout << "(" << pair.first << ", " << pair.second << ")" << std::endl;
//     }

//     return minimumVertexCover(conflictingAgentPairs);

// }

std::vector<AStarPath> findSolution(Map map, HeuristicType type, std::string experimentName)
{
    std::vector<HeuristicTable> heuristics;
    // PARALLELIZE HERE
    for (int a = 0; a < map.nAgents; a++)
    {
        heuristics.push_back(computeAstarHeuristics(map.agents[a].goal, map));
    }
    /////////////

    auto start = std::chrono::high_resolution_clock::now();

    uint nGenerated = 0;
    uint nExpanded = 0;

    struct CBSNode
    {
        uint cost;
        uint heuristic;
        std::vector<Constraint> constraints;
        std::vector<AStarPath> paths;
        std::vector<Collision> collisions;
    };

    struct CompareCBSNode
    {
        bool operator()(const CBSNode &a, const CBSNode &b)
        {
            return a.cost - a.heuristic > b.cost - b.heuristic;
        }
    };

    std::priority_queue<int, std::vector<CBSNode>, CompareCBSNode> openList;

    CBSNode root = {
        0, 0, {}, {}, {}};


    // PARALLELIZE HERE
    for (int a = 0; a < map.nAgents; a++) {
        AStarPath path = aStar(map, map.starts[a], map.goals[a], heuristics[a], a, root.constraints);
        if (path.size() == 0) {
            std::cerr << "No solutions!" << std::endl;
            return {};
        }
        root.paths.push_back(path);
    }
    ////////

    root.cost = getSumOfCost(root.paths);
    root.collisions = detectCollisions(root.paths);

    openList.push(root);
    nGenerated++;

    for (auto &collision : root.collisions)
    {
        // std::pair<Constraint, Constraint> c = standardSplitting(collision);
        // Constraint c1 = c.first;
        // Constraint c2 = c.second;

        // std::cout << "First constraint: ";
        // printConstraint(c1);
        // std::cout << "Second constraint: ";
        // printConstraint(c2);
    }

    uint maxPathLength = map.cols * map.rows * 10;
    uint maxIters = maxPathLength * 10 * map.nAgents;
    uint i = 0;

    // while (openList.size() > 0)
    // {
    //     i++;
    //     if (i > maxIters) {
    //         std::cout << "Broke from maxIters\n";
    //         break;
    //     }
    // }
    while (openList.size() > 0)
    {
        // std::cout << "\n############ EXPANDING NODE ############" << std::endl;
        CBSNode curr = openList.top();
        openList.pop();
        nExpanded++;

        if (curr.collisions.size() == 0)
        {
            auto end = std::chrono::high_resolution_clock::now();
            auto secs = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
            printResults(curr.paths, nExpanded, nGenerated, secs, experimentName);

            return curr.paths;
        }

        Collision currCollision = curr.collisions[0];

        std::pair<Constraint, Constraint> constraints = standardSplitting(currCollision);

        for (auto constraint : {constraints.first, constraints.second})
        {
            if (std::find(curr.constraints.begin(), curr.constraints.end(), constraint) != curr.constraints.end())
                continue;

            CBSNode qNode = {
                0,
                0,
                curr.constraints,
                curr.paths,
                curr.collisions,
            };
            qNode.constraints.push_back(constraint);

            uint agentId = constraint.agentId;
            AStarPath path = aStar(
                map, map.starts[agentId], map.goals[agentId],
                heuristics[agentId], agentId, qNode.constraints);

            if (path.size() > 0)
            {
                qNode.paths[agentId] = path;
                qNode.collisions = detectCollisions(qNode.paths);
                qNode.cost = getSumOfCost(qNode.paths);

                if (qNode.cost >= maxPathLength)
                    continue;

                switch (type)
                { // Default case: no heuristic just proceed and h value will be left as 0
                case CG:
                    qNode.heuristic = computeCGHeuristic(map, qNode.constraints, qNode.paths, heuristics);
                    break;
                    // std::cout << "Computed CG heuristic: " << qNode.heuristic << std::endl;
                case DG:
                    //qNode.heuristic = computeDGHeuristic(map, heuristics, qNode.constraints);
                    //break;
                    // std::cout << "Computed DG heuristic: " << qNode.heuristic << std::endl;
                case WDG:
                    qNode.heuristic = 0;
                    break;
                }

                openList.push(qNode);
                nGenerated++;
            }
        }
    }

    return {};
}