#pragma once

#include "defs.h"

#include <string>
#include <fstream>
#include <iostream>

struct Agent {
    uint startx;
    uint starty;
    uint goalx;
    uint goaly;

    uint currx;
    uint curry;

    Agent() : startx(0), starty(0), goalx(0), goaly(0), currx(0), curry(0) {}
    Agent(uint sx, uint sy, uint gx, uint gy) : startx(sx), starty(sy), goalx(gx), goaly(gy), currx(sx), curry(sy) {}
};

struct Map {
    uint rows;
    uint cols;
    bool** tiles;

    uint nAgents;
    Agent* agents;

    std::pair<uint, uint>* starts;
    std::pair<uint, uint>* goals;

    bool loaded;

    void loadFromFile(std::string path);
    void destroy();

    void printTiles();
};