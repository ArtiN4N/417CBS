CC = g++
INC = -I include/
SRC = src/map.cpp src/cbs.cpp src/cg/heuristic.cpp src/dg/heuristic.cpp src/wdg/heuristic.cpp

VIS_TEST_SRC = src/visualizer/vis_load.cpp src/visualizer/vis.cpp
VIS_TEST_EXE = -o bin/vis_test.exe
VIS_TEST_LIB = -Llib/ -lraylib -lopengl32 -lgdi32 -lwinmm
VIS_TEST_FLAG = -O3

FLAG = -std=c++14 -O3 -pthread

CG_SRC = src/cg/cg_load.cpp
CG_EXE = -o bin/cg_test

DG_SRC = src/dg/dg_load.cpp
DG_EXE = -o bin/dg_test

WDG_SRC = src/wdg/wdg_load.cpp
WDG_EXE = -o bin/wdg_test


cg_test:
	$(CC) $(CG_SRC) $(SRC) $(CG_EXE) $(FLAG) $(INC)

dg_test:
	$(CC) $(DG_SRC) $(SRC) $(DG_EXE) $(FLAG) $(INC)

wdg_test:
	$(CC) $(WDG_SRC) $(SRC) $(WDG_EXE) $(FLAG) $(INC)

vis_test:
	$(CC) $(VIS_TEST_SRC) $(SRC) $(VIS_TEST_EXE) $(VIS_TEST_FLAG) $(FLAG) $(INC) $(VIS_TEST_LIB)

clean:
	rm -f bin/*