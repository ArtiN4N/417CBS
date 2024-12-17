#include "../include/cbstools.h"

#include <thread>
#include <cmath>

HeuristicTable computeAstarHeuristics(AStarLocation goal, Map map) {
    struct HNode {
        AStarLocation location;
        uint cost;
    };

    struct CompareHNode {
        bool operator()(const HNode &a, const HNode &b) { return a.cost > b.cost; }
    };

    std::priority_queue<int, std::vector<HNode>, CompareHNode> openList;
    std::unordered_map<AStarLocation, HNode, PairHash> closedList;

    HNode root = {goal, 0};
    openList.push(root);
    closedList[goal] = root;

    while (openList.size() > 0) {
        HNode curr = openList.top();
        openList.pop();

        for (int i = 0; i < 4; i++) {
            // cast int to ordered enum
            uint dir = i;

            AStarLocation childLocation = move(curr.location, dir);
            uint childCost = curr.cost + 1;

            bool xBound = childLocation.first < 0 || childLocation.first >= map.cols;
            bool yBound = childLocation.second < 0 || childLocation.second >= map.rows;

            if (xBound || yBound) continue;
            if (map.tiles[childLocation.first][childLocation.second]) continue;

            HNode child = {childLocation, childCost};
            // closed already
            if (closedList.find(childLocation) != closedList.end()) {
                HNode exisitingNode = closedList[childLocation];
                if (exisitingNode.cost > childCost) {
                    closedList[childLocation] = child;
                    openList.push(child);
                }
            } else {
                closedList[childLocation] = child;
                openList.push(child);
            }
        }
    }

    HeuristicTable hTable;
    for (auto &pair : closedList)
        hTable[pair.first] = pair.second.cost;

    return hTable;
}

ConstraintTable buildConstraintTable(std::vector<Constraint> constraints, uint agent, GoalWallTable goalWalls) {
    ConstraintTable table;
    for (auto &constraint : constraints) {
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

bool isConstrained(AStarLocation currentLoc, AStarLocation nextLoc, uint nextTime, ConstraintTable cTable) {
    if (cTable.find(nextTime) == cTable.end())
        return false;

    for (auto &constraint : cTable[nextTime]) {
        if (!constraint.isEdgeCollision)
            if (nextLoc == constraint.l1)
                return true;
        else {
            bool def = currentLoc == constraint.l1 && nextLoc == constraint.l2;
            bool opp = nextLoc == constraint.l1 && currentLoc == constraint.l2;
            if (def || opp)
                return true;
        }
    }

    return false;
}

AStarLocation move(AStarLocation location, uint dir) {
    AStarLocation ret = location;
    switch (dir) {
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

uint getSumOfCost(std::vector<AStarPath> paths) {
    uint ret = 0;
    for (auto &path : paths)
        ret += path.size() - 1;
    return ret;
}

AStarLocation getLocation(AStarPath path, uint timestep) {
    if (timestep < 0)
        return path[0];
    else if (timestep < path.size())
        return path[timestep];
    else
        return path[-1];
}

AStarPath getPath(AStarNode *goalNode) {
    AStarPath path;
    AStarNode *curr = goalNode;

    while (curr != nullptr) {
        path.push_back(curr->location);
        curr = curr->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

std::pair<bool, Collision> detectCollision(AStarPath path1, AStarPath path2, uint id1, uint id2) {
    Collision ret = {};
    bool valid = false;

    AStarLocation lastPath1 = getLocation(path1, 0);
    AStarLocation lastPath2 = getLocation(path2, 0);

    for (uint t = 0; t < std::max(path1.size(), path2.size()); t++) {
        bool collision = getLocation(path1, t) == lastPath2 && getLocation(path2, t) == lastPath1;
        if (t < std::min(path1.size(), path2.size()) && collision) {
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

        if (loc1 == loc2) {
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

void computeCollisionForPaths(int start, int end, std::vector<AStarPath> paths, std::vector<Collision>& ret) {
    int n = paths.size();
    for (int k = 0; k < n * n; k++) {
        int i = k / n;
        int j = k % n;

        if (i == j || j < i)
            continue;

        std::pair<bool, Collision> c = detectCollision(paths[i], paths[j], i, j);

        if (c.first)
            ret[k] = c.second;
    }
}

void detectParallelCollisions(std::vector<AStarPath> paths, std::vector<Collision>& ret, uint nthreads) {
    std::cout << "doing parallel collisions\n";
    std::vector<std::thread> threads;

    int n = paths.size();
    int total = n * (n - 1) / 2;

    int perThread = std::ceil((float) total / (float) nthreads);

    for (int t = 0; t < nthreads; t++) {
        int start = t * perThread;
        int end = std::min((t + 1) * perThread, (int) total);
        threads.push_back(std::thread(computeCollisionForPaths, start, end, std::ref(paths), std::ref(ret)));
    }

    for (auto& t : threads) {
        t.join();
    }
}

void detectSerialCollisions(std::vector<AStarPath> paths, std::vector<Collision>& ret) {
    int n = paths.size();
    int total = n * (n - 1) / 2;

    for (int k = 0; k < n * n; k++) {
        int i = k / n;
        int j = k % n;

        if (i == j || j < i)
            continue;

        std::pair<bool, Collision> c = detectCollision(paths[i], paths[j], i, j);

        if (c.first)
            ret[k] = c.second;
    }
}


std::vector<Collision> detectCollisions(std::vector<AStarPath> paths, bool parallel, uint nthreads) {
    std::vector<Collision> ret;
    int n = paths.size();
    int total = n * n;
    ret.resize(total);

    if (parallel) detectParallelCollisions(paths, ret, nthreads);
    else detectSerialCollisions(paths, ret);

    ret.erase(std::remove_if(ret.begin(), ret.end(), [](const Collision& c) { return c == Collision{}; }), ret.end());
    return ret;
}

std::pair<Constraint, Constraint> standardSplitting(Collision c) {
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

bool compareAStarNodes(AStarNode n1, AStarNode n2) {
    return n1.gval + n1.hval < n2.gval + n2.hval;
}

AStarPath aStar(
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
            return getPath(curr);

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
                if (compareAStarNodes(child, existingNode)) {
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

void printAStarPath(AStarPath path) {
    uint ret = 0;
    for (auto loc : path)
        std::cout << "(" << loc.second << "," << loc.first << ")->";
    std::cout << std::endl;
}

void printCollisionLocation(CollisionLocation l) {
    std::cout << "[ (" << l.l1.first << "," << l.l1.second << ")";
    if (l.isEdgeCollision)
        std::cout << ", (" << l.l2.first << "," << l.l2.second << ")";
    std::cout << " ]";
}

void printConstraint(Constraint c) {
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

void computeSerialAStarHeuristics(std::vector<HeuristicTable>& heuristics, Map map, uint nAgents) {
    for (int a = 0; a < nAgents; a++)
        heuristics.push_back(computeAstarHeuristics(map.agents[a].goal, map));
}

void computeHeuristicForAgent(int start, int end, std::vector<HeuristicTable>& heuristics, Map& map) {
    for (int a = start; a < end; a++) {
        heuristics[a] = computeAstarHeuristics(map.agents[a].goal, map);
    }
}

void computeParallelAStarHeuristics(std::vector<HeuristicTable>& heuristics, Map map, uint nAgents, uint nthreads) {
    std::vector<std::thread> threads;

    int agentsPerThread = std::ceil((float) nAgents / (float) nthreads);

    heuristics.resize(nAgents);

    for (int t = 0; t < nthreads; t++) {
        int start = t * agentsPerThread;
        int end = std::min((t + 1) * agentsPerThread, (int) nAgents);
        threads.push_back(std::thread(computeHeuristicForAgent, start, end, std::ref(heuristics), std::ref(map)));
    }

    for (auto& t : threads) {
        t.join();
    }
}

void computeAllAStarHeuristics(std::vector<HeuristicTable>& heuristics, Map map, bool parallel, uint nthreads) {
    if (parallel) computeParallelAStarHeuristics(heuristics, map, map.nAgents, nthreads);
    else computeSerialAStarHeuristics(heuristics, map, map.nAgents);
}

void computeSerialAgentPaths(Map map, std::vector<HeuristicTable> heuristics, CBSNode& root, uint nAgents) {
    for (int a = 0; a < nAgents; a++) {
        AStarPath path = aStar(map, map.starts[a], map.goals[a], heuristics[a], a, root.constraints);
        if (path.size() == 0)
            std::cerr << "No solutions!" << std::endl;

        root.paths.push_back(path);
    }
}

void computePathForAgent(int start, int end, std::vector<HeuristicTable>& heuristics, Map& map, CBSNode& root) {
    for (int a = start; a < end; a++) {
        AStarPath path = aStar(map, map.starts[a], map.goals[a], heuristics[a], a, root.constraints);
        if (path.size() == 0)
            std::cerr << "No solutions!" << std::endl;

        root.paths[a] = path;
    }
}

void computeParallelAgentPaths(Map map, std::vector<HeuristicTable> heuristics, CBSNode& root, uint nAgents, uint nthreads) {
    std::vector<std::thread> threads;

    int agentsPerThread = std::ceil((float) nAgents / (float) nthreads);

    root.paths.resize(nAgents);

    for (int t = 0; t < nthreads; t++) {
        int start = t * agentsPerThread;
        int end = std::min((t + 1) * agentsPerThread, (int) nAgents);
        threads.push_back(std::thread(computePathForAgent, start, end, std::ref(heuristics), std::ref(map), std::ref(root)));
    }

    for (auto& t : threads) {
        t.join();
    }
}


void computeAllAgentPaths(Map map, std::vector<HeuristicTable> heuristics, CBSNode& root, bool parallel, uint nthreads) {
    if (parallel) computeParallelAgentPaths(map, heuristics, root, map.nAgents, nthreads);
    else computeSerialAgentPaths(map, heuristics, root, map.nAgents);
}
