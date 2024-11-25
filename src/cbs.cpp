#include "../include/cbs.h"

AStarLocation move(AStarLocation location, Direction dir) {
    AStarLocation ret = location;
    switch (dir) {
        case NORTH:
            ret.second--;
        case EAST:
            ret.first++;
        case SOUTH:
            ret.second++;
        case WEST:
            ret.first--;
        default:
    }

    return ret;
}

uint getSumOfCost(std::vector<AStarPath> paths) {
    uint ret = 0;
    for (auto& path : paths) ret += path.size() - 1;
    return ret;
}

HeuristicTable computeDefaultHeuristic(AStarLocation goal, Map map) {
    struct HNode {
        AStarLocation location;
        uint cost;
    };

    struct CompareHNode {
        bool operator()(const HNode& a, const HNode& b) {
            return a.cost > b.cost;
        }
    };

    std::priority_queue<int, std::vector<HNode>, CompareHNode> openList;
    std::unordered_map<AStarLocation, HNode, PairHash> closedList;

    HNode root = { goal, 0 };
    openList.push(root);
    closedList[goal] = root;

    while (openList.size() > 0) {
        HNode curr = openList.top();
        openList.pop();

        for (int i = 0; i < 4; i++) {
            // cast int to ordered enum
            Direction dir = (Direction) i;

            AStarLocation childLocation = move(curr.location, dir);
            uint childCost = curr.cost + 1;

            bool xBound = childLocation.first < 0 || childLocation.first >= map.cols;
            bool yBound = childLocation.second < 0 || childLocation.second >= map.rows;
            if (xBound || yBound) continue;
            if (map.tiles[childLocation.first][childLocation.second]) continue;

            HNode child = { childLocation, childCost };
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
    for (auto& pair : closedList) {
        hTable[pair.first] = pair.second.cost;
    }

    return hTable;
}

HeuristicTable computeHeuristics(HeuristicType type, AStarLocation goal, Map map) {
    switch (type) {
        case CG:
            //return computeCGHeuristic(goal, map);
        case DEFAULT:
            return computeDefaultHeuristic(goal, map);
        case DG:
            return computeDGHeuristic(goal, map);
        case WDG:
            return computeWDGHeuristic(goal, map);
    }
}

ConstraintTable buildConstraintTable(std::vector<Constraint> constraints, uint agent, GoalWallTable goalWalls) {
    ConstraintTable table;
    for (auto& constraint : constraints) {
        if (agent != constraint.agentId) continue;

        uint cTimestep = constraint.timeStep;
        CollisionLocation cLocation = constraint.location;

        if (constraint.isGoalWall) goalWalls[cLocation.l1] = cTimestep;

        table[cTimestep].push_back(cLocation);
    }

    return table;
}

AStarLocation getLocation(AStarPath path, uint timestep) {
    if (timestep < 0) return path[0];
    else if (timestep < path.size()) return path[timestep];
    else return path[-1];
}

AStarPath getPath(AStarNode goalNode) {
    AStarPath path;
    AStarNode* curr = &goalNode;

    while (curr != nullptr) {
        path.push_back(curr->location);
        curr = curr->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

bool isConstrained(AStarLocation currentLoc, AStarLocation nextLoc, uint nextTime, ConstraintTable cTable) {
    if (cTable.find(nextTime) != cTable.end()) return false;

    for (auto& constraint : cTable[nextTime]) {
        if (!constraint.isEdgeCollision) {
            if (nextLoc == constraint.l1) return true;
        } else {
            bool def = currentLoc == constraint.l1 && nextLoc == constraint.l2;
            bool opp = nextLoc == constraint.l1 && currentLoc == constraint.l2;
            if (def || opp) return true;
        }
    }
    
    return false;
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
                t, false
            };
            valid = true;
            break;
        }
        
        if (t < path1.size()) lastPath1 = getLocation(path1, t);
        AStarLocation loc1 = lastPath1;

        if (t < path2.size()) lastPath2 = getLocation(path2, t);
        AStarLocation loc2 = lastPath2;

        if (loc1 == loc2) {
            ret = Collision{
                id1, id2,
                CollisionLocation{loc1, false, {}},
                t, false
            };
            valid = true;
            break;
        }
    }

    return std::make_pair(valid, ret);
}

std::vector<Collision> detectCollisions(std::vector<AStarPath> paths) {
    std::vector<Collision> ret;
    for (int i = 0; i < paths.size(); i++) {
        for (int j = i + 1; j < paths.size(); j++) {
            if (i == j) continue;
            std::pair<bool, Collision> c = detectCollision(paths[i], paths[j], i, j);
            if (c.first) ret.push_back(c.second);
        }
    }

    return ret;
}

std::pair<Constraint, Constraint> standardSplitting(Collision c) {
    uint a1 = c.agentId;
    uint a2 = c.agentId2;
    CollisionLocation location = c.location;
    uint timeStep = c.timeStep;

    if (!location.isEdgeCollision)
        return std::make_pair(
            Constraint{ a1, location, timeStep, false },
            Constraint{ a2, location, timeStep, false }
        );
    else
        return std::make_pair(
            Constraint{ a1, location, timeStep, false },
            Constraint{ a2, CollisionLocation{location.l2, true, location.l1}, timeStep, false }
        );
}

bool compareAStarNodes(AStarNode n1, AStarNode n2) {
    return n1.gval + n1.hval < n2.gval + n2.hval;
} 

AStarPath aStar(
    Map map, AStarLocation startLoc, AStarLocation goalLoc,
    HeuristicTable hTable, uint agent, std::vector<Constraint> constraints
) {

    struct CompareANode {
        bool operator()(const AStarNode& a, const AStarNode& b) {
            return a.gval + a.hval > b.gval + b.hval;
        }
    };

    std::priority_queue<int, std::vector<AStarNode>, CompareANode> openList;
    std::unordered_map<std::pair<AStarLocation, uint>, AStarNode, PairHash> closedList;

    GoalWallTable goalWalls;

    uint hValue = hTable[startLoc];

    ConstraintTable cTable = buildConstraintTable(constraints, agent, goalWalls);
    uint maxTimestep = 0;

    for (auto& pair : cTable) {
        uint timeStep = pair.first;
        auto constraints = pair.second;

        for (auto& constraint : constraints) {
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
            [](const auto& a, const auto& b) {
                return a.second < b.second;
            }
        )->second;
        terminateTimestep += max_value + 1;
    }

    AStarNode root = { startLoc, 0, hValue, nullptr, 0 };
    openList.push(root);
    closedList[std::make_pair(startLoc, 0)] = root;

    AStarNode curr;

    while (openList.size() < 0) {
        curr = openList.top();
        openList.pop();

        if (curr.location == goalLoc && curr.timeStep >= maxTimestep)
            return getPath(curr);
        
        for (int i = 0; i < 5; i++) {
            // cast int to ordered enum
            Direction dir = (Direction) i;

            AStarLocation childLoc;

            if (i == 4) childLoc = curr.location;
            else childLoc = move(curr.location, dir);

            uint childTime = curr.timeStep + 1;

            if (childTime > terminateTimestep) continue;

            if (isConstrained(curr.location, childLoc, childTime, cTable)) continue;

            bool xBound = childLoc.first < 0 || childLoc.first >= map.cols;
            bool yBound = childLoc.second < 0 || childLoc.second >= map.rows;
            if (xBound || yBound) continue;
            if (map.tiles[childLoc.first][childLoc.second]) continue;

            if (goalWalls.find(childLoc) != goalWalls.end())
                if (goalWalls[childLoc] <= childTime)
                    continue;

            AStarNode child = {
                childLoc,
                curr.gval + 1, hTable[childLoc],
                &curr,
                childTime
            };

            if (closedList.find(std::make_pair(childLoc, childTime)) != closedList.end()) {
                AStarNode existingNode = closedList[std::make_pair(childLoc, childTime)];
                if (compareAStarNodes(child, existingNode)) {
                    closedList[std::make_pair(childLoc, childTime)] = child;
                    openList.push(child);
                }
            } else {
                closedList[std::make_pair(childLoc, childTime)] = child;
                openList.push(child);
            }
        }
    }

    return {};
}

void printCollisionLocation(CollisionLocation l) {
    std::cout << "[ (" << l.l1.first << "," << l.l1.second << ")";
    if (l.isEdgeCollision) {
        std::cout << ", (" << l.l2.first << "," << l.l2.second << ")";
    }
    std::cout << " ]";
}

void printConstraint(Constraint c) {
    std::cout << " id=" << c.agentId << " time=" << c.timeStep << " location=";
    printCollisionLocation(c.location);
    std::cout << std::endl;
}

void printResults(std::vector<AStarPath> paths, uint nExpanded, uint nGenerated, double elapsed) {
    std::cout << "Found a solution!" << std::endl;
    std::cout << "Time elapsed : " << elapsed << std::endl;
    std::cout << "Sum of costs : " << getSumOfCost(paths) << std::endl;
    std::cout << "Expanded nodes : " << nExpanded << std::endl;
    std::cout << "Generated nodes : " << nGenerated << std::endl;
}

std::vector<AStarPath> findSolution(Map map, std::vector<HeuristicTable> heuristics) {
    auto start = std::chrono::high_resolution_clock::now();

    uint nGenerated = 0;
    uint nExpanded = 0;

    struct CBSNode {
        uint cost;
        std::vector<Constraint> constraints;
        std::vector<AStarPath> paths;
        std::vector<Collision> collisions;
    };

    struct CompareCBSNode {
        bool operator()(const CBSNode& a, const CBSNode& b) {
            return a.cost > b.cost;
        }
    };

    std::priority_queue<int, std::vector<CBSNode>, CompareCBSNode> openList;

    CBSNode root = {
        0, {}, {}, {}
    };

    for (int a = 0; a < map.nAgents; a++) {
        AStarPath path = aStar(map, map.starts[a], map.goals[a], heuristics[a], a, root.constraints);
        if (path.size() == 0) {
            std::cerr << "No solutions!" << std::endl;
            return {};
        }

        root.paths.push_back(path);
    }

    root.cost = getSumOfCost(root.paths);
    root.collisions = detectCollisions(root.paths);

    openList.push(root);

    for (auto& collision : root.collisions) {
        std::pair<Constraint, Constraint> c = standardSplitting(collision);
        Constraint c1 = c.first;
        Constraint c2 = c.second;

        std::cout << "First constraint: ";
        printConstraint(c1);
        std::cout << "Second constraint: ";
        printConstraint(c2);
    }

    uint maxPathLength = map.cols * map.rows * 10;

    while (openList.size() > 0) {
        std::cout << "############ EXPANDING NODE ############" << std::endl;
        CBSNode curr = openList.top();
        openList.pop();

        if (curr.collisions.size() == 0) {
            auto end = std::chrono::high_resolution_clock::now();
            auto secs = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
            printResults(curr.paths, nExpanded, nGenerated, secs);
            
            return curr.paths;
        }

        Collision currCollision = curr.collisions[0];

        std::pair<Constraint, Constraint> constraints = standardSplitting(currCollision);

        for (auto constraint : {constraints.first, constraints.second}) {
            if (std::find(curr.constraints.begin(), curr.constraints.end(), constraint) != curr.constraints.end())
                continue;
            
            CBSNode qNode = {
                0,
                curr.constraints,
                curr.paths,
                curr.collisions,
            };
            qNode.constraints.push_back(constraint);

            uint agentId = constraint.agentId;
            AStarPath path = aStar(
                map, map.starts[agentId], map.goals[agentId],
                heuristics[agentId], agentId, qNode.constraints
            );

            if (path.size() > 0) {
                qNode.paths[agentId] = path;
                qNode.collisions = detectCollisions(qNode.paths);
                qNode.cost = getSumOfCost(qNode.paths);

                if (qNode.cost >= maxPathLength) continue;

                openList.push(qNode);
            }
        }
    }

    return {};
}