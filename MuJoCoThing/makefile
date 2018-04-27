COMMON=-O2 -I./include -L./bin -std=c++11 ./*.so 
run:
	g++ $(COMMON) *.cpp ./NewControllers/*.cpp ./Systems/*.cpp -L./include -lmujoco150 -lGL -lglew ./bin/libglfw.so.3 ./bin/libmujoco150.so -o run.out

