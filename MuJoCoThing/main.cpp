#include <iostream>
#include "Mediator.h"
#include "Controller.h"
#include "./Systems/MujocoSystem.h"
#include "./NewControllers/CPGController.h"
#include "./Genome.h"

using std::cout;
using std::endl;

int main(int argc, char ** argv) {
	mj_activate("mjkey.txt");
	srand(time(NULL));

	const int params = 28;
	const int size = 16;
	const int population = 16;
	const int epochs = 2000;
	std::vector<Genome> pool;

	MujocoSystem mjSys((char*)"MinitaurFull.xml");
	mjSys.setRealTime(false);
	mjSys.setGraphics(false);

	for(int i = 0; i < population; i++){
		pool.push_back(Genome(params, size));
	}

	for(int i = 0; i < epochs; i++){
		//Run simulation for every genome in pool
		cout << "beginning epoch " << i << " with population of " << pool.size() << endl;
		for(int j = 0; j < pool.size(); j++){
			double parameters[params];
			for(int k = 0; k < params; k++){
				parameters[k] = pool[j].getCodon(k);
			}
			if(j == 0 && i % 5 == 0){
				cout << "displaying most fit genome." << endl;
				mjSys.setRealTime(true);
				mjSys.setGraphics(true);
			}else{
				mjSys.setRealTime(false);
				mjSys.setGraphics(false);
			}
			CPGController c = CPGController(parameters);
			Mediator m(&c, &mjSys);
			m.run(4);
			pool[j].setFitness(c.exit());
			mjSys.reset();
			cout << ".";
		}
		cout << endl;
		//Sort genomes by fitness
		cout << " considering " << pool.size() << " genomes.\n";
		for(int j = 0; j < pool.size(); j++){
			for(int k = 0; k < j; k++){
				if(pool[j].getFitness() > pool[k].getFitness()){
					pool.insert(pool.begin() + k, pool[j]);
					pool.erase(pool.begin() + j);
					break;
				}
			}
		}

		cout << "epoch ended. Fitnesses (in order): " << endl;
		for(int j = 0; j < pool.size(); j++){
			cout << pool[j].getFitness() << endl;
		}
		//Cull the weak
		cout << "culling unfit genomes" << endl;
		int inital_pop = pool.size();
		for(int j = population; j < inital_pop; j++){
			pool.pop_back();
		}
		cout << "beginning crossbreed." << endl;
		int desired_breeding_pairs = pool.size()/2;
		for(int j = 0; j < desired_breeding_pairs; j++){
			pool.push_back(pool[j].crossbreed(pool[pool.size()-j-1]));
			pool.push_back(pool[j].crossbreed(pool[j+1]));
		}
		cout << "pool size is now " << pool.size() << endl;
	}
	return 0;
}
