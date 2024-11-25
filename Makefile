visualizer:
	g++ src/visualizer/vis_load.cpp src/visualizer/vis.cpp -o bin/vis_test -O3 -Wno-missing-braces -I include/ -Llib/ -lraylib -lGL -lm -lpthread -ldl -lrt -lX11