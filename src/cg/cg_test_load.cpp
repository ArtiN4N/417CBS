#include <iostream>

#include "../../include/map.h"
#include "../../include/defs.h"
#include "../../include/cbs.h"

int main(int argc, char* argv[]) {
    HeuristicType type = CG;

    Map map = {};
    map.initAgents();
    map.loadFromFile("instances/test_1.txt");

    std::vector<AStarPath> soln = findSolution(map, type, "useless.txt");
    if (soln.size() > 0) std::cout << "fount solution with " << map.nAgents << " agents\n";

    map.destroy();
    return 0;
}