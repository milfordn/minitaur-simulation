COMMON=-O2 -I./include -L./bin -std=c++11 -mavx
run:
	g++ $(COMMON) *.cpp ./Controllers/*.cpp -L./include -lmujoco150 -lGL -lglew ./bin/libglfw.so.3 -o run.out