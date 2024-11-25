SRC = src/visualizer/vis_load.cpp src/visualizer/vis.cpp
LIB = -Llib/ -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
INC = -I include/
FLAG = -O3 -Wno-missing-braces
EXE = -o bin/vis_test

visualizer: $(SRC) | $(BIN_DIR)
	g++ $(SRC) $(EXE) $(FLAG) $(INC) $(LIB)