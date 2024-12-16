#include "../include/map.h"

#include <sstream>

// File format:
// first line: rows columns
// n = rows, next n lines: @=wall OR .=space
// next line: agents
// a = agents, next a lines: starty startx goaly goalx

void Map::loadFromFile(std::string path) {
    std::ifstream file(path);

    if (!file.is_open()) {
        std::cerr << "Failed to open map file : " << path << std::endl;
        return;
    }

    file >> rows >> cols;

    tiles = new bool*[cols];
    for (int c = 0; c < cols; c++) tiles[c] = new bool[rows];

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            char symbol;
            file >> symbol;
            // Walls, or the "@" symbol, have a true value
            tiles[c][r] = (symbol == '@');
        }
    }

    file >> nAgents;

    agents.clear();
    starts.clear();
    goals.clear();

    for (int a = 0; a < nAgents; a++) {
        uint start_x, start_y, goal_x, goal_y;
        file >> start_y >> start_x >> goal_y >> goal_x;

        agents[a] = Agent(start_x, start_y, goal_x, goal_y);
        starts[a] = std::make_pair(start_x, start_y);
        goals[a] = std::make_pair(goal_x, goal_y);
    }

    file.close();
}

std::string getFileFromInt(int f) {
    switch (f) {
case 1:
        return "Berlin_1_256";
case 2:
        return "Boston_0_256";
case 3:
        return "Paris_1_256";
case 4:
        return "brc202d";
case 5:
        return "den312d";
case 6:
        return "den520d";
case 7:
        return "empty-16-16";
case 8:
        return "empty-32-32";
case 9:
        return "empty-48-48";
case 10:
        return "empty-8-8";
case 11:
        return "ht_chantry";
case 12:
        return "ht_mansion_n";
case 13:
        return "lak303d";
case 14:
        return "lt_gallowstemplar_n";
case 15:
        return "maze-128-128-1";
case 16:
        return "maze-128-128-10";
case 17:
        return "maze-128-128-2";
case 18:
        return "maze-32-32-2";
case 19:
        return "maze-32-32-4";
case 20:
        return "orz900d";
case 21:
        return "ost003d";
case 22:
        return "random-32-32-10";
case 23:
        return "random-32-32-20";
case 24:
        return "random-64-64-10";
case 25:
        return "random-64-64-20";
case 26:
        return "room-32-32-4";
case 27:
        return "room-64-64-16";
case 28:
        return "room-64-64-8";
case 29:
        return "w_woundedcoast";
case 30:
        return "warehouse-10-20-10-2-1";
case 31:
        return "warehouse-10-20-10-2-2";
case 32:
        return "warehouse-20-40-10-2-1";
case 33:
        return "warehouse-20-40-10-2-2";
};
return "";
}

std::string Map::loadMapBoundsFromFile(int mapFile) {
    std::string mapFileName = getFileFromInt(mapFile);
    std::ifstream file("autotest/mapf-map/" + mapFileName + ".map");

    if (!file.is_open()) {
        std::cerr << "Failed to open map file : " << ("autotest/mapf-map/" + mapFileName + ".map") << std::endl;
        return "";
    }

    std::string word;

    while (file >> word) {
        if (word == "height") {
            file >> rows;
        } else if (word == "width") {
            file >> cols;
        } else if (word == "map") {
            break;
        }
    }

    tiles = new bool*[cols];
    for (int c = 0; c < cols; c++) tiles[c] = new bool[rows];

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            char symbol;
            file >> symbol;
            // Walls, or the "@" symbol, have a true value
            tiles[c][r] = (symbol == '@');
        }
    }
    
    file.close();

    return mapFileName;
}

void Map::addAgentFromLine(std::string line) {
    std::istringstream stream(line);
    std::vector<std::string> parts;

    // Split the line into parts based on whitespace
    std::string part;
    while (stream >> part) {
        parts.push_back(part);
    }

    // Ensure we have enough elements in the parts vector
    if (parts.size() >= 8) {
        uint start_x, start_y, goal_x, goal_y;
        start_x = std::stoi(parts[4]);
        start_y = std::stoi(parts[5]);
        goal_x = std::stoi(parts[6]);
        goal_y = std::stoi(parts[7]);

        Agent a = Agent(start_x, start_y, goal_x, goal_y);
        agents.push_back(a);
        starts.push_back(std::make_pair(start_x, start_y));
        goals.push_back(std::make_pair(goal_x, goal_y));
        nAgents++;

    } else {
        std::cerr << "Error: Not enough elements in the line." << std::endl;
    }
}

void Map::initAgents() {
    nAgents = 0;

    agents = {};
    starts = {};
    goals = {};
}
void Map::resetAgents() {
    nAgents = 0;

    agents.clear();
    starts.clear();
    goals.clear();
}

void Map::destroy() {
    if (!loaded) return;
    for (int i = 0; i < rows; i++) delete[] tiles[i];

    delete[] tiles;
}

void Map::printTiles() {
    std::cout << "Start locations" << std::endl;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            std::string symbol = ".";
            if (tiles[c][r]) symbol = "@";

            for (int a = 0; a < nAgents; a++) {

                if (starts[a].first == c && starts[a].second == r) {
                    symbol = std::to_string(a);
                }
            }

            std::cout << symbol << " ";
        }
        std::cout << std::endl;
    }

    std::cout << std::endl << "Goal locations" << std::endl;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            std::string symbol = ".";
            if (tiles[c][r]) symbol = "@";

            for (int a = 0; a < nAgents; a++) {
                if (goals[a].first == c && goals[a].second == r) symbol = std::to_string(a);
            }

            std::cout << symbol << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}