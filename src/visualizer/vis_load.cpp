#include "../../include/raylib.h"
#include "../../include/map.h"
#include "../../include/defs.h"
#include "../../include/cbs.h"

#include <iostream>

#define tileSize 50

float speed = 0.f;

void drawMap(Map map) {
    Color wallColor = DARKGRAY;
    Color spaceColor = LIGHTGRAY;
    Color agentColors[] = { RED, GREEN, BLUE, YELLOW, PURPLE, GOLD, BROWN, ORANGE };
    uint nAgentColors = 8;

    for (int r = 0; r < map.rows; r++) {
        for (int c = 0; c < map.cols; c++) {
            Color draw = spaceColor;
            if (map.tiles[c][r]) draw = wallColor;

            DrawRectangle(c * tileSize, r * tileSize, tileSize, tileSize, draw);
        }
    }

    for (int a = 0; a < map.nAgents; a++) {
        Color draw = agentColors[a % nAgentColors];

        Agent actor = map.agents[a];

        uint x = actor.goal.first;
        uint y = actor.goal.second;

        uint margin = 10;

        draw.a = 150;
        DrawRectangle(x * tileSize + margin / 2, y * tileSize + margin / 2, tileSize - margin, tileSize - margin, draw);

        float ax = actor.animCurr.first;
        float ay = actor.animCurr.second;
        uint radius = tileSize / 2 - 5;
        margin = 2;
        draw.a = 255;

        DrawCircle(ax * tileSize + radius + margin / 2 + 4.4, ay * tileSize + radius + margin / 2 + 4.4, radius - margin, draw);

        DrawText(TextFormat("%d", a), ax * tileSize + radius + margin / 2 - 4 + 4.4, ay * tileSize + radius + margin / 2 - 8 + 4.4, 20, BLACK);
    }
}

void updateMap(Map& map, std::vector<AStarPath> soln, uint timestep) {
    for (int a = 0; a < map.nAgents; a++) {
        Agent& actor = map.agents[a];
        uint dir = 4;

        if (timestep + 1 >= soln[a].size()) continue;

        AStarLocation nextLoc = soln[a][timestep + 1];
        if (nextLoc.first > actor.curr.first) actor.animCurr.first += speed; // east
        else if (nextLoc.first < actor.curr.first) actor.animCurr.first -= speed; // west
        else if (nextLoc.second > actor.curr.second) actor.animCurr.second += speed; // south
        else if (nextLoc.second < actor.curr.second) actor.animCurr.second -= speed; // north
    }
}

void stepTimestep(Map& map, std::vector<AStarPath> soln, uint timestep) {
    for (int a = 0; a < map.nAgents; a++) {
        if (timestep >= soln[a].size()) continue;

        Agent& actor = map.agents[a];
        actor.curr = soln[a][timestep];

        actor.animCurr = std::make_pair((float)actor.curr.first, (float)actor.curr.second);
    }
}

int main() {
    HeuristicType type = WDG;

    Map map = {};
    map.loadFromFile("instances/test_49.txt");

    map.printTiles();

    std::vector<AStarPath> soln = findSolution(map, type, "useless.txt", true, 2);
    
    uint windowWidth = map.cols * tileSize;
    uint windowHeight = map.rows * tileSize;

    InitWindow(windowWidth, windowHeight, "Visualization test");

    float elapsed = 0.f;
    uint timestep = 0;

    uint FPS = 60;

    float interval = .25f;

    speed = 1.f / ((float) FPS * interval);

    SetTargetFPS(FPS);
    while (!WindowShouldClose()) {
        elapsed += GetFrameTime();

        if (elapsed >= interval) {
            timestep++;
            elapsed = 0.f;
            stepTimestep(map, soln, timestep);
        }

        updateMap(map, soln, timestep);

        BeginDrawing();

        ClearBackground(BLACK);
        DrawFPS(20, 20);

        drawMap(map);

        EndDrawing();
    }
    
    map.destroy();

    CloseWindow();
    return 0;
}