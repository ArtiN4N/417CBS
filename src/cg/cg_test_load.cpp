#include <iostream>

#include "../../include/map.h"
#include "../../include/defs.h"
#include "../../include/cbs.h"

int main(int argc, char* argv[]) {
    HeuristicType type = CG;

    for (int i = 1; i <= 50; i++) {
        Map map = {};
        map.initAgents();
        map.loadFromFile("instances/test_" + std::to_string(i) + ".txt");

        std::vector<AStarPath> soln = findSolution(map, type, "useless.txt");
        if (soln.size() > 0) std::cout << "fount solution with " << map.nAgents << " agents for map << " << i << "\n";

        map.destroy();
    }

    
    return 0;
}