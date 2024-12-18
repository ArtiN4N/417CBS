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



// int minimumWeighedVertexCover(const std::vector<std::pair<int, int>> &edges, int nAgents) {
//     int ret = 0;
//     std::vector<bool> finished(nAgents, false);

//     for (int i = 0; i < nAgents; i++) {
//         if (finished[i]) continue;

//         std::queue<int> wvcq;
//         wvcq.push(i);
//         finished[i] = true;
//     }
//     return ret;
// }

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

int computeConflictWeight(const Map &map, const std::vector<Constraint> &constraints,
                          const std::vector<AStarPath> &paths, int agent1, int agent2,
                          const std::vector<HeuristicTable> &heuristics)
{
    // Find original costs
    int cost1 = getSumOfCost({paths[agent1]});
    int cost2 = getSumOfCost({paths[agent2]});

    // Add constraints to resolve conflict
    std::vector<Constraint> tempConstraints = constraints;

    // Recalculate path without heuristics
    AStarPath oldPath1 = aStar(map, map.starts[agent1], map.goals[agent1], heuristics[agent1], agent1, {});
    AStarPath oldPath2 = aStar(map, map.starts[agent2], map.goals[agent2], heuristics[agent2], agent2, {});

    // Compute the delta cost
    int oldCost1 = getSumOfCost({oldPath1});
    int oldCost2 = getSumOfCost({oldPath2});

    return (cost1 + cost2) - (oldCost1 + oldCost2);
}


int computeDGHeuristic(const Map &map, const std::vector<Constraint> &constraints, const std::vector<AStarPath> &paths, std::vector<HeuristicTable> heuristics)
{
    std::vector<std::pair<int, int>> conflictingAgentPairs;
    std::vector<MDD> mdds(map.nAgents); // Pre-allocate memory for MDDs

    // Create MDDs for all agents
    for (int i = 0; i < map.nAgents; i++)
    {
        AStarPath pathi = paths[i];
        mdds[i].createMDD(pathi[0], pathi.size(), i, constraints, map, heuristics[i]);
    }

    for (size_t i = 0; i < map.nAgents; i++)
    {
        for (size_t j = i + 1; j < map.nAgents; j++)
        {
            if (detectCardinalConflict(mdds[i], mdds[j]))
            { // if they have cardinal conflict they are dependent
                conflictingAgentPairs.emplace_back(i, j);
            }
            else if (detectDependency(mdds[i], mdds[j]))
            { // explicitly check for dependency
                conflictingAgentPairs.emplace_back(i, j);
            }
        }
    }
    return minimumVertexCover(conflictingAgentPairs);
}

int minimumWeightedVertexCover(std::vector<std::pair<int, int>> &edges, 
                               const std::vector<int> &weights, int nAgents)
{
    std::vector<bool> finished(nAgents, false);
    int totalCost = 0;

    while (!edges.empty())
    {
        // Get max edge weight
        int maxWeight = 0;
        size_t heavyEdge = 0;

        for (size_t i = 0; i < edges.size(); i++)
        {
            if (weights[i] > maxWeight)
            {
                maxWeight = weights[i];
                heavyEdge = i;
            }
        }

        // Pick one of the vertices in the edge and mark as finished
        int u = edges[heavyEdge].first;
        int v = edges[heavyEdge].second;

        if (!finished[u])
        {
            finished[u] = true;
            totalCost += maxWeight;
        }
        else if (!finished[v])
        {
            finished[v] = true;
            totalCost += maxWeight;
        }

        // Remove all edges that intersect with u or v
        for (size_t i = 0; i < edges.size();)
        {
            if (edges[i].first == u || edges[i].second == u || edges[i].first == v || edges[i].second == v)
            {
                edges.erase(edges.begin() + i);
            }
            else
            {
                i++;
            }
        }
    }
    return totalCost;
}

int computeWDGHeuristic(const Map &map, const std::vector<Constraint> &constraints, const std::vector<AStarPath> &paths, std::vector<HeuristicTable> heuristics)
{
    std::vector<std::pair<int, int>> conflictingAgentPairs;
    std::vector<int> weights;
    std::vector<MDD> mdds(map.nAgents);

    // Create MDDs for all agents
    for (int i = 0; i < map.nAgents; i++)
    {
        AStarPath pathi = paths[i];
        mdds[i].createMDD(pathi[0], pathi.size(), i, constraints, map, heuristics[i]);
    }

    for (size_t i = 0; i < map.nAgents; i++)
    {
        for (size_t j = i + 1; j < map.nAgents; j++)
        {
            if (detectCardinalConflict(mdds[i], mdds[j]))
            {
                // Same check as DG, but we need to get weights
                // Weight should be sum of increase in paths
                int weight = computeConflictWeight(map, constraints, paths, i, j, heuristics);
                conflictingAgentPairs.emplace_back(i, j);
                weights.push_back(weight);
            }
            else if (detectDependency(mdds[i], mdds[j]))
            { 
                // check  seperatey as to not call depenfency as often
                int weight = computeConflictWeight(map, constraints, paths, i, j, heuristics);
                conflictingAgentPairs.emplace_back(i, j);
                weights.push_back(weight);
            }
        }
    }
    std::cout << "Getting Vertex Cover \n";
    int ret = minimumWeightedVertexCover(conflictingAgentPairs, weights, map.nAgents);
    
    std::cout << ret << " Max W Vertex Cover \n";
    return ret;
}

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
    for (int a = 0; a < map.nAgents; a++)
    {
        AStarPath path = aStar(map, map.starts[a], map.goals[a], heuristics[a], a, root.constraints);
        if (path.size() == 0)
        {
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
                    qNode.heuristic = computeDGHeuristic(map, qNode.constraints, qNode.paths, heuristics);
                    break;
                    // std::cout << "Computed DG heuristic: " << qNode.heuristic << std::endl;
                case WDG:
                    qNode.heuristic = computeWDGHeuristic(map, qNode.constraints, qNode.paths, heuristics);
                    break;
                }

                openList.push(qNode);
                nGenerated++;
            }
        }
    }

    return {};
}