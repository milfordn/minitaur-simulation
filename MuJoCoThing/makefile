COMPILER=g++
EXE_FILE=run
COM_MUJOCO=-O2 -L. -std=c++11 -lstdc++ -B./bin -mavx -lmujoco150 -lglfw.3 -I include

# $(EXE_FILE): Main.o ModelController.o KeyboardController.o CustomSimulate.o
# 	$(COMPILER) Main.o ModelController.o KeyboardController.o CustomSimulate.o -o $(EXE_FILE)

test:
	$(COMPILER) $(COM_MUJOCO) CustomSimulate.cpp KeyBoardController.cpp ModelController.cpp main.cpp -o run

# Main.o: Main.cpp
# 	$(COMPILER) $(COM_MUJOCO) main.cpp -lKeyboardController -lCustomSimulate	-lmujoco150 -lglfw.3 -o main.o
# ModelController.o: ModelController.cpp
# 	$(COMPILER) $(COM_MUJOCO) -c ModelController.cpp 		-lmujoco150 -lglfw.3 -o ModelController.o
# KeyboardContoller.o: KeyboardContoller.cpp
# 	$(COMPILER) $(COM_MUJOCO) -c KeyboardController.cpp 	-lmujoco150 -lglfw.3 -o KeyboardController.o
# CustomSimulate.o: CustomSimulate.cpp
# 	$(COMPILER) $(COM_MUJOCO) -c CustomSimulate.cpp			-lmujoco150 -lglfw.3 -o CustomSimulate.o
clean:
	rm -f *.out *.o $(exe_file)

# all:
# 	# $(COMPILER) $(COM_MUJOCO) ModelController.cpp		-lmujoco150 -lglfw.3 -o bin/ModelController.o
# 	# $(COMPILER) $(COM_MUJOCO) KeyboardController.cpp	-lmujoco150 -lglfw.3 -bin/ModelController.o -o bin/KeyboardController.o
# 	$(COMPILER) $(COM_MUJOCO) CustomSimulate.cpp		-lmujoco150 -lglfw.3 -bin/ModelController.o -o bin/CustomSimulate.o
# 	$(COMPILER) $(COM_MUJOCO) main.cpp				-lmujoco150 -lglfw.3 -bin/CustomSimulate.o -bin/ModelController.o -bin/KeyboardController.o -o run
# 	# $(COMPILER) $(COMMON) experiment.cpp -lmujoco150 -lglfw.3 -o
# 	# $(COMPILER) $(COMMON) basic.cpp      -lmujoco150 -lglfw.3 -o ../bin/basic
# 	# $(COMPILER) $(COMMON) simulate.cpp   -lmujoco150 -lglfw.3 -o ../bin/simulate
# 	# $(COMPILER) $(COMMON) test.cpp       -lmujoco150          -o ../bin/test
# 	# $(COMPILER) $(COMMON) compile.cpp    -lmujoco150          -o ../bin/compile
# 	# $(COMPILER) $(COMMON) derivative.cpp -lmujoco150          -o ../bin/derivative
# 	# $(COMPILER) $(COMMON) record.cpp     -lmujoco150 -lglfw.3 -o ../bin/record
	