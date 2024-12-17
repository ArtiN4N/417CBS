#include "../include/cbstools.h"

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

AStarPath getPath(AStarNode* goalNode) {
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

std::vector<Collision> detectCollisions(std::vector<AStarPath> paths) {
    std::vector<Collision> ret;

    // PARALLELIZE HERE
    for (int i = 0; i < paths.size(); i++) {
        for (int j = i + 1; j < paths.size(); j++) {
            if (i == j)
                continue;

            std::pair<bool, Collision> c = detectCollision(paths[i], paths[j], i, j);

            if (c.first)
                ret.push_back(c.second);
        }
    }
    //////////

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