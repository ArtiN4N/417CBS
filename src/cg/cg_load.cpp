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

    if (!(number >= 1 && number <= 33)) {
        std::cerr << "Error: Please enter a number between 1 and 33.\n";
        return 1;
    }

    HeuristicType type = CG;
 
    Map map = {};
    map.initAgents();
    std::string mapName = map.loadMapBoundsFromFile(number);
    for (int scen = 1; scen < 4; scen++) {

        std::string experimentName = "map" + std::to_string(number) + "scen" + std::to_string(scen) + ".out";
        std::ifstream infile("autotest/tests/" + experimentName);
        if (infile) {
            // Clear file contents
            std::ofstream("autotest/tests/" + experimentName, std::ios::trunc).close();
        }

        map.resetAgents();

        std::string filename = "autotest/mapf-scen-even/scen-even/" + mapName + "-even-" + std::to_string(scen) + ".scen";
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << "\n";
            return -1;
        }

        int aCount = 0;
        std::string line;

        std::getline(file, line);

        while (std::getline(file, line)) {
            aCount++;

            map.addAgentFromLine(line);
            //map.printTiles();
            std::vector<AStarPath> soln = findSolution(map, type, experimentName, true, 6);
            if (soln.size() > 0) std::cout << "fount solution with " << aCount << " agents\n";
            else break;
        }
    }

    map.destroy();
    return 0;
}