#include "../../include/raylib.h"
#include "../../include/map.h"
#include "../../include/defs.h"

#include <iostream>

#define tileSize 50

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

        uint x = actor.currx;
        uint y = actor.curry;
        uint radius = tileSize / 2;
        uint margin = 2;
        DrawCircle(x * tileSize + radius + margin / 2, y * tileSize + radius + margin / 2, radius - margin, draw);

        x = actor.goalx;
        y = actor.goaly;

        margin = 10;

        draw.a = 150;
        DrawRectangle(x * tileSize + margin / 2, y * tileSize + margin / 2, tileSize - margin, tileSize - margin, draw);
    }
}

int main() {
    Map map = {};
    map.loadFromFile("instances/test.txt");

    map.printTiles();

    uint windowWidth = map.cols * tileSize;
    uint windowHeight = map.rows * tileSize;

    InitWindow(windowWidth, windowHeight, "Visualization test");

    SetTargetFPS(30); 
    while (!WindowShouldClose()) {
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