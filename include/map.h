#pragma once

#include "defs.h"

#include <string>
#include <fstream>
#include <iostream>

struct Agent {
    AStarLocation start;
    AStarLocation goal;

    AStarLocation curr;

    Agent() : start(std::make_pair(0, 0)), goal(std::make_pair(0, 0)), curr(std::make_pair(0, 0)) {}
    Agent(uint sx, uint sy, uint gx, uint gy) : start(std::make_pair(sx, sy)), goal(std::make_pair(gx, gy)), curr(std::make_pair(sx, sy)) {}
};

struct Map {
    uint rows;
    uint cols;
    bool** tiles;

    uint nAgents;
    Agent* agents;

    AStarLocation* starts;
    AStarLocation* goals;

    bool loaded;

    void loadFromFile(std::string path);
    void destroy();

    void printTiles();
};