#include "../include/cbs.h"

AStarLocation move(AStarLocation location, uint dir)
{
    AStarLocation ret = location;
    switch (dir)
    {
    case 0:
        ret.second--;
        break;
    case 1:
        ret.first++;
        break;
    case 2:
        ret.second++;
        break;
    case 3:
        ret.first--;
        break;
    default:
        break;
    }

    return ret;
}

uint getSumOfCost(std::vector<AStarPath> paths)
{
    uint ret = 0;
    for (auto &path : paths)
        ret += path.size() - 1;
    return ret;
}

ConstraintTable buildConstraintTable(std::vector<Constraint> constraints, uint agent, GoalWallTable goalWalls)
{
    ConstraintTable table;
    for (auto &constraint : constraints)
    {
        if (agent != constraint.agentId)
            continue;

        uint cTimestep = constraint.timeStep;
        CollisionLocation cLocation = constraint.location;

        if (constraint.isGoalWall)
            goalWalls[cLocation.l1] = cTimestep;

        table[cTimestep].push_back(cLocation);
    }

    return table;
}

AStarLocation getLocation(AStarPath path, uint timestep)
{
    if (timestep < 0)
        return path[0];
    else if (timestep < path.size())
        return path[timestep];
    else
        return path[-1];
}

AStarPath getPath(AStarNode *goalNode)
{
    AStarPath path;
    AStarNode *curr = goalNode;

    while (curr != nullptr)
    {
        path.push_back(curr->location);
        curr = curr->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

bool isConstrained(AStarLocation currentLoc, AStarLocation nextLoc, uint nextTime, ConstraintTable cTable)
{
    if (cTable.find(nextTime) == cTable.end())
        return false;

    for (auto &constraint : cTable[nextTime])
    {
        if (!constraint.isEdgeCollision)
        {
            if (nextLoc == constraint.l1)
                return true;
        }
        else
        {
            bool def = currentLoc == constraint.l1 && nextLoc == constraint.l2;
            bool opp = nextLoc == constraint.l1 && currentLoc == constraint.l2;
            if (def || opp)
                return true;
        }
    }

    return false;
}

std::pair<bool, Collision> detectCollision(AStarPath path1, AStarPath path2, uint id1, uint id2)
{
    Collision ret = {};
    bool valid = false;

    AStarLocation lastPath1 = getLocation(path1, 0);
    AStarLocation lastPath2 = getLocation(path2, 0);

    for (uint t = 0; t < std::max(path1.size(), path2.size()); t++)
    {
        bool collision = getLocation(path1, t) == lastPath2 && getLocation(path2, t) == lastPath1;
        if (t < std::min(path1.size(), path2.size()) && collision)
        {
            ret = Collision{
                id1, id2,
                CollisionLocation{getLocation(path1, t), true, getLocation(path2, t)},
                t, false};
            valid = true;
            break;
        }

        if (t < path1.size())
            lastPath1 = getLocation(path1, t);
        AStarLocation loc1 = lastPath1;

        if (t < path2.size())
            lastPath2 = getLocation(path2, t);
        AStarLocation loc2 = lastPath2;

        if (loc1 == loc2)
        {
            ret = Collision{
                id1, id2,
                CollisionLocation{loc1, false, {}},
                t, false};
            valid = true;
            break;
        }
    }

    return std::make_pair(valid, ret);
}

std::vector<Collision> detectCollisions(std::vector<AStarPath> paths)
{
    std::vector<Collision> ret;
    for (int i = 0; i < paths.size(); i++)
    {
        for (int j = i + 1; j < paths.size(); j++)
        {
            if (i == j)
                continue;
            std::pair<bool, Collision> c = detectCollision(paths[i], paths[j], i, j);
            if (c.first)
                ret.push_back(c.second);
        }
    }

    return ret;
}

bool detectCardinalConflict(const std::vector<AStarPath> &paths1, const std::vector<AStarPath> &paths2)
{
    size_t maxTimestep = 0;
    for (const auto &path : paths1)
    {
        maxTimestep = std::max(maxTimestep, path.size());
    }
    for (const auto &path : paths2)
    {
        maxTimestep = std::max(maxTimestep, path.size());
    }

    for (uint t = 0; t < maxTimestep; t++)
    {

        // Get all locations agent 1 and 2 can be a time t
        std::vector<AStarLocation> locations1;
        std::vector<AStarLocation> locations2;

        // Get all possible locations for agent 1 at time t
        for (const auto &path : paths1)
        {
            AStarLocation loc = getLocation(path, t);
            locations1.push_back(loc);
        }

        // Get all possible locations for agent 1 at time t
        for (const auto &path : paths2)
        {
            AStarLocation loc = getLocation(path, t);
            locations2.push_back(loc);
        }

        std::sort(locations1.begin(), locations1.end());
        std::sort(locations2.begin(), locations2.end());

        if (locations1 == locations2)
        {
            return true; // Cardinal conflict detected
        }
    }

    return false; // No cardinal conflict detected
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

std::pair<Constraint, Constraint> standardSplitting(Collision c)
{
    uint a1 = c.agentId;
    uint a2 = c.agentId2;
    CollisionLocation location = c.location;
    uint timeStep = c.timeStep;

    if (!location.isEdgeCollision)
        return std::make_pair(
            Constraint{a1, location, timeStep, false},
            Constraint{a2, location, timeStep, false});
    else
        return std::make_pair(
            Constraint{a1, location, timeStep, false},
            Constraint{a2, CollisionLocation{location.l2, true, location.l1}, timeStep, false});
}

bool compareAStarNodes(AStarNode n1, AStarNode n2)
{
    return n1.gval + n1.hval < n2.gval + n2.hval;
}

std::vector<AStarPath> aStar(
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

    while (openList.size() > 0)
    {
        AStarNode *curr = openList.top();
        openList.pop();

        AStarPath cpath = getPath(curr);
        int cpathCost = cpath.size();
        if (cpathCost > shortestCost) continue;

        if (curr->location == goalLoc && curr->timeStep >= maxTimestep)
        {
            //std::cout << "found a shortest path\n";
            

            // Determine the path cost (length of the path)
            

            if (cpathCost < shortestCost) {
                shortestPaths.clear();
            }

            if (cpathCost <= shortestCost){
                shortestCost = cpathCost;//pathCost;
                shortestPaths.push_back(cpath);

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

            if (closedList.find(std::make_pair(childLoc, childTime)) != closedList.end())
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
}

void printCollisionLocation(CollisionLocation l)
{
    std::cout << "[ (" << l.l1.first << "," << l.l1.second << ")";
    if (l.isEdgeCollision)
    {
        std::cout << ", (" << l.l2.first << "," << l.l2.second << ")";
    }
    std::cout << " ]";
}

void printConstraint(Constraint c)
{
    std::cout << " id=" << c.agentId << " time=" << c.timeStep << " location=";
    printCollisionLocation(c.location);
    std::cout << std::endl;
}

void printResults(std::vector<AStarPath> paths, uint nExpanded, uint nGenerated, double elapsed, std::string filename) {

    std::ofstream file;

    std::string text = std::to_string(elapsed) + "," + std::to_string(getSumOfCost(paths));

    file.open("autotest/tests/" + filename, std::ios::app); // open to append mode
    file << text << std::endl;
    file.close();

    //std::cout << "Found a solution!" << std::endl;
    //std::cout << "Time elapsed : " << elapsed << std::endl;
    //std::cout << "Sum of costs : " << getSumOfCost(paths) << std::endl;
    //std::cout << "Expanded nodes : " << nExpanded << std::endl;
    //std::cout << "Generated nodes : " << nGenerated << std::endl;
}

void printAStarPath(AStarPath path)
{
    uint ret = 0;
    for (auto loc : path)
        std::cout << "(" << loc.second << "," << loc.first << ")->";
    std::cout << std::endl;
}

HeuristicTable computeAstarHeuristics(AStarLocation goal, Map map)
{
    struct HNode
    {
        AStarLocation location;
        uint cost;
    };

    struct CompareHNode
    {
        bool operator()(const HNode &a, const HNode &b)
        {
            return a.cost > b.cost;
        }
    };

    std::priority_queue<int, std::vector<HNode>, CompareHNode> openList;
    std::unordered_map<AStarLocation, HNode, PairHash> closedList;

    HNode root = {goal, 0};
    openList.push(root);
    closedList[goal] = root;

    while (openList.size() > 0)
    {
        HNode curr = openList.top();
        openList.pop();

        for (int i = 0; i < 4; i++)
        {
            // cast int to ordered enum
            uint dir = i;

            AStarLocation childLocation = move(curr.location, dir);
            uint childCost = curr.cost + 1;

            bool xBound = childLocation.first < 0 || childLocation.first >= map.cols;
            bool yBound = childLocation.second < 0 || childLocation.second >= map.rows;
            if (xBound || yBound)
                continue;
            if (map.tiles[childLocation.first][childLocation.second])
                continue;

            HNode child = {childLocation, childCost};
            // closed already
            if (closedList.find(childLocation) != closedList.end())
            {
                HNode exisitingNode = closedList[childLocation];
                if (exisitingNode.cost > childCost)
                {
                    closedList[childLocation] = child;
                    openList.push(child);
                }
            }
            else
            {
                closedList[childLocation] = child;
                openList.push(child);
            }
        }
    }

    HeuristicTable hTable;
    for (auto &pair : closedList)
    {
        hTable[pair.first] = pair.second.cost;
    }

    return hTable;
}

int computeCGHeuristic(const Map &map, const std::vector<HeuristicTable> &heuristics, const std::vector<Constraint> &constraints)
{
    std::unordered_map<int, std::vector<AStarPath>> agentPaths;
    // list to store pairs of agents with cardinal conflicts
    std::vector<std::pair<int, int>> conflictingAgentPairs;

    for (int a = 0; a < map.nAgents; ++a)
    {
        agentPaths[a] = aStar(map, map.starts[a], map.goals[a], heuristics[a], a, constraints);
    }

    for (size_t i = 0; i < map.nAgents; i++)
    {
        for (size_t j = i + 1; j < map.nAgents; j++)
        {
            // if they have cardinal conflict add pair of agents to list
            const auto &paths1 = agentPaths[i];
            const auto &paths2 = agentPaths[j];

            if (detectCardinalConflict(paths1, paths2))
            {
                // Store the pair of conflicting agents
                conflictingAgentPairs.emplace_back(i, j);
            }
        }
    }
    return minimumVertexCover(conflictingAgentPairs);
}

bool detectDependecy(const std::vector<AStarPath> &paths1, const std::vector<AStarPath> &paths2)
{
    for (uint path1 = 0; path1 < paths1.size(); path1++)
    {
        for (uint path2 = 0; path2 < paths2.size(); path2++)
        {

            size_t maxTimestep = 0;
            for (const auto &path : paths1)
            {
                maxTimestep = std::max(maxTimestep, path.size());
            }
            for (const auto &path : paths2)
            {
                maxTimestep = std::max(maxTimestep, path.size());
            }

            bool conflict = false;

            for (uint t = 0; t < maxTimestep; t++)
            {
                if (getLocation(paths1[path1], t) == getLocation(paths2[path2], t))
                {
                    conflict = true;
                    break;
                }
            }

            if (!conflict)
            { // if we go through a pair of paths with no conflicts then these agents arent dependent
                return false;
            }
        }
    }
    return true;
}

int computeDGHeuristic(const Map &map, const std::vector<HeuristicTable> &heuristics, const std::vector<Constraint> &constraints)
{
    std::unordered_map<int, std::vector<AStarPath>> agentPaths;
    // list to store pairs of agents with cardinal conflicts
    std::vector<std::pair<int, int>> conflictingAgentPairs;

    for (int a = 0; a < map.nAgents; ++a)
    {
        agentPaths[a] = aStar(map, map.starts[a], map.goals[a], heuristics[a], a, constraints);
    }

    for (size_t i = 0; i < map.nAgents; i++)
    {
        for (size_t j = i + 1; j < map.nAgents; j++)
        {

            const std::vector<AStarPath> paths1 = agentPaths[i];
            const std::vector<AStarPath> paths2 = agentPaths[j];

            if (detectCardinalConflict(paths1, paths2))
            { // if they have cardinal conflict they are dependent
                conflictingAgentPairs.emplace_back(i, j);
            }
            else if (detectDependecy(paths1, paths2))
            { // explicitly check for dependency
                conflictingAgentPairs.emplace_back(i, j);
            }
        }
    }
    return minimumVertexCover(conflictingAgentPairs);
}

std::vector<AStarPath> findSolution(Map map, HeuristicType type, std::string experimentName) {
    std::vector<HeuristicTable> heuristics;
    for (int a = 0; a < map.nAgents; a++)
    {
        heuristics.push_back(computeAstarHeuristics(map.agents[a].goal, map));
    }

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
            return a.cost + a.heuristic > b.cost + b.heuristic;
        }
    };

    std::priority_queue<int, std::vector<CBSNode>, CompareCBSNode> openList;

    CBSNode root = {
        0, 0, {}, {}, {}};

    for (int a = 0; a < map.nAgents; a++)
    {
        std::vector<AStarPath> paths = aStar(map, map.starts[a], map.goals[a], heuristics[a], a, root.constraints);

        if (paths.size() == 0)
        {
            std::cerr << "No solutions!" << std::endl;
            return {};
        }

        AStarPath path = paths[0];
        root.paths.push_back(path);
    }

    root.cost = getSumOfCost(root.paths);
    root.collisions = detectCollisions(root.paths);

    openList.push(root);
    nGenerated++;

    for (auto &collision : root.collisions)
    {
        std::pair<Constraint, Constraint> c = standardSplitting(collision);
        Constraint c1 = c.first;
        Constraint c2 = c.second;

        //std::cout << "First constraint: ";
        //printConstraint(c1);
        //std::cout << "Second constraint: ";
        //printConstraint(c2);
    }

    uint maxPathLength = map.cols * map.rows * 10;
    uint maxIters = maxPathLength * 10 * map.nAgents;
    uint i = 0;

    while (openList.size() > 0)
    {
        i++;
        if (i > maxIters) {
            std::cout << "Broke from maxIters\n";
            break;
        }

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
            std::vector<AStarPath> paths = aStar(
                map, map.starts[agentId], map.goals[agentId],
                heuristics[agentId], agentId, qNode.constraints);

            if (paths.size() > 0)
            {
                AStarPath path = paths[0];
                qNode.paths[agentId] = path;
                qNode.collisions = detectCollisions(qNode.paths);
                qNode.cost = getSumOfCost(qNode.paths);

                if (qNode.cost >= maxPathLength)
                    continue;

                switch (type)
                { // Default case: no heuristic just proceed and h value will be left as 0
                case CG:
                    qNode.heuristic = computeCGHeuristic(map, heuristics, qNode.constraints);
                case DG:
                    qNode.heuristic = computeDGHeuristic(map, heuristics, qNode.constraints);
                case WDG:
                    qNode.heuristic = 0;
                }

                openList.push(qNode);
                nGenerated++;
            }
        }
    }

    return {};
}