COMMON=-O2 -I../include -L../bin -std=c++11 -mavx

default:
	g++ $(COMMON) CustomSimulate.cpp include/pid.cpp -L../mujoco/include -lmujoco150 -lGL -lglew include/libglfw.so.3 -o run

all: default
