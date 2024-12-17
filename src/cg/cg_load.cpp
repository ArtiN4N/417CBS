#include <iostream>

#include "../../include/map.h"
#include "../../include/defs.h"
#include "../../include/cbs.h"

int main() {
    HeuristicType type = CG;
 
    Map map = {};
    map.loadFromFile("instances/custom.txt");

    map.printTiles();

    std::vector<AStarPath> soln = findSolution(map, type);

    map.destroy();
    return 0;
}