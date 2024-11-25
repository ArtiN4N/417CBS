#include <iostream>

#include "../../include/map.h"
#include "../../include/defs.h"
#include "../../include/cbs.h"

int main() {
    HeuristicType type = CG;

    Map map = {};
    map.loadFromFile("instances/test.txt");

    map.printTiles();

    std::vector<HeuristicTable> heuristics;
    for (int a = 0; a < map.nAgents; a++) {
        heuristics.push_back(computeHeuristics(type, map.agents[a].goal, map));
    }

    std::vector<AStarPath> soln = findSolution(map, heuristics);

    map.destroy();
    return 0;
}