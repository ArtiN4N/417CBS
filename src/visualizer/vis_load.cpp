#include "../../include/raylib.h"

#include <iostream>

int main() {
    std::cout << "hello world" << std::endl;

    InitWindow(800, 800, "Visualization test");

    SetTargetFPS(30); 
    while (!WindowShouldClose()) {
        BeginDrawing();

        ClearBackground(BLACK);
        DrawFPS(20, 20);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}