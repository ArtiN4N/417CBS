#pragma once

#include "defs.h"

#include <string>
#include <fstream>
#include <iostream>
#include <vector>

struct Agent {
    AStarLocation start;
    AStarLocation goal;

    AStarLocation curr;
    std::pair<float, float> animCurr;

    Agent() : start(std::make_pair(0, 0)), goal(std::make_pair(0, 0)), curr(std::make_pair(0, 0)), animCurr(std::make_pair(0.f, 0.f)) {}
    Agent(uint sx, uint sy, uint gx, uint gy) : start(std::make_pair(sx, sy)), goal(std::make_pair(gx, gy)), curr(std::make_pair(sx, sy)), animCurr(std::make_pair((float)sx, (float)sy)) {}
};

struct Map {
    uint rows;
    uint cols;
    bool** tiles;

    uint nAgents;
    std::vector<Agent> agents;

    std::vector<AStarLocation> starts;
    std::vector<AStarLocation> goals;

    bool loaded;

    void loadFromFile(std::string path);
    std::string loadMapBoundsFromFile(int mapFile);
    void addAgentFromLine(std::string line);
    void initAgents();
    void resetAgents();
    void destroy();

    void printTiles();
};