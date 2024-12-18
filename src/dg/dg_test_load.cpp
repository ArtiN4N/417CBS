#include <iostream>

#include "../../include/map.h"
#include "../../include/defs.h"
#include "../../include/cbs.h"

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <number from 1 to 33>\n";
        return 1;
    }

    int number = std::atoi(argv[1]); // Convert argument to an integer

    if (!(number >= 1 && number <= 50)) {
        std::cerr << "Error: Please enter a number between 1 and 50.\n";
        return 1;
    }
    
    HeuristicType type = DG;

    //for (int i = 2; i <= 50; i++) {
        Map map = {};
        map.initAgents();
        map.loadFromFile("instances/test_" + std::to_string(number) + ".txt");
        map.printTiles();

        std::vector<AStarPath> soln = findSolution(map, type, "useless.txt", true, 4);
        if (soln.size() > 0) std::cout << "fount solution with " << map.nAgents << " agents for map << " << number << "\n";

        map.destroy();
    //}

    return 0;
}