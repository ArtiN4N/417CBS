CC = g++
INC = -I include/

VIS_TEST_SRC = src/visualizer/vis_load.cpp src/visualizer/vis.cpp
VIS_TEST_EXE = -o bin/vis_test.exe
VIS_TEST_LIB = -Llib/ -lraylib -lopengl32 -lgdi32 -lwinmm
VIS_TEST_FLAG = -mwindows -O3 -Wno-missing-braces

NON_VIS_FLAG = -std=c++14 -O3 -pthread

CG_SRC = src/cg/cg_load.cpp
CG_EXE = -o bin/cg_test

DG_SRC = src/dg/dg_load.cpp
DG_EXE = -o bin/dg_test

WDG_SRC = src/wdg/wdg_load.cpp
WDG_EXE = -o bin/wdg_test


cg_test:
	$(CC) $(CG_SRC) $(CG_EXE) $(NON_VIS_FLAG) $(INC)

dg_test:
	$(CC) $(DG_SRC) $(DG_EXE) $(NON_VIS_FLAG) $(INC)

wdg_test:
	$(CC) $(WDG_SRC) $(WDG_EXE) $(NON_VIS_FLAG) $(INC)

vis_test:
	$(CC) $(VIS_TEST_SRC) $(VIS_TEST_EXE) $(VIS_TEST_FLAG) $(INC) $(VIS_TEST_LIB)
