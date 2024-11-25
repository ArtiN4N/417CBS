#include "../include/map.h"


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

    agents = new Agent[nAgents];
    starts = new std::pair<uint, uint>[nAgents];
    goals = new std::pair<uint, uint>[nAgents];

    for (int a = 0; a < nAgents; a++) {
        uint start_x, start_y, goal_x, goal_y;
        file >> start_y >> start_x >> goal_y >> goal_x;

        agents[a] = Agent(start_x, start_y, goal_x, goal_y);
        starts[a] = std::make_pair(start_x, start_y);
        goals[a] = std::make_pair(goal_x, goal_y);
    }

    file.close();
}

void Map::destroy() {
    if (!loaded) return;
    for (int i = 0; i < rows; i++) delete[] tiles[i];

    delete[] tiles;

    delete[] agents;
    delete[] starts;
    delete[] goals;
}

void Map::printTiles() {
    std::cout << "Start locations" << std::endl;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            std::string symbol = ".";
            if (tiles[c][r]) symbol = "@";
        
            for (int a = 0; a < nAgents; a++) {
                if (starts[a].first == c && starts[a].second == r) symbol = std::to_string(a);
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