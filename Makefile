CC = g++
INC = -I include/
SRC = src/map.cpp src/cbs.cpp 

VIS_TEST_SRC = src/visualizer/vis_load.cpp src/visualizer/vis.cpp
VIS_TEST_EXE = -o bin/vis_test.exe
VIS_TEST_LIB = -Llib/ -lraylib -lopengl32 -lgdi32 -lwinmm
VIS_TEST_FLAG = -O3

NON_VIS_FLAG = -std=c++14 -O3 -pthread

TEST_SRC = src/cg/cg_load.cpp src/dg/dg_load.cpp src/wdg/wdg_load.cpp
CG_EXE = -o bin/cg_test
DG_EXE = -o bin/dg_test
WDG_EXE = -o bin/wdg_test


cg_test:
	$(CC) $(TEST_SRC) $(SRC) $(CG_EXE) $(NON_VIS_FLAG) $(INC)

dg_test:
	$(CC) $(TEST_SRC) $(SRC) $(DG_EXE) $(NON_VIS_FLAG) $(INC)

wdg_test:
	$(CC) $(TEST_SRC) $(SRC) $(WDG_EXE) $(NON_VIS_FLAG) $(INC)

vis_test:
	$(CC) $(VIS_TEST_SRC) $(SRC) $(VIS_TEST_EXE) $(VIS_TEST_FLAG) $(INC) $(VIS_TEST_LIB)

clean:
	rm -f bin/*