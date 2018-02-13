COMMON=-O2 -I./include -L./bin -std=c++11 -mavx
run:
	g++ $(COMMON) CustomSimulate.cpp KeyboardController.cpp ModelController.cpp PIDController.cpp pid.cpp main.cpp -L./include -lmujoco150 -lGL -lglew ./bin/libglfw.so.3 -o run
clean:
	rm run
