#include <iostream>
#include <fstream>
#include <algorithm>
#include "Mediator.h"
#include "Controller.h"
#include "./Systems/MujocoSystem.h"
#include "./NewControllers/CPGController.h"
#include "./Genome.h"

using std::cout;
using std::endl;
using std::flush;
ofstream output;

int main(int argc, char ** argv) {
	mj_activate("mjkey.txt");
	srand(time(NULL));

	const int params = 28;
	const int size = 16;
	const int population = 8;
	const int epochs = 2000;
	std::vector<Genome> pool;

	MujocoSystem mjSys((char*)"MinitaurFull.xml");

	for(int i = 0; i < population; i++){
		pool.push_back(Genome(params, size));
	}

	for(int i = 0; i < epochs; i++){
		//Run simulation for every genome in pool
		cout << "***************************\nbeginning epoch " << i << " with population of " << pool.size() << endl;
		cout << "beginning crossbreed." << endl;
		int desired_breeding_pairs = pool.size()/2;
		for(int j = 0; j < desired_breeding_pairs; j++){
			pool.push_back(pool[j].crossbreed(pool[pool.size()-j-1]));
			pool.push_back(pool[j].crossbreed(pool[j+1]));
			cout << "." << flush;
		}
		cout << endl;
		cout << "simulating genomes" << endl;
		for(int j = 0; j < pool.size(); j++){
			if (j % (pool.size() / 10) == 0) cout << "." << endl;

			double parameters[params];

			for(int k = 0; k < params; k++){
				parameters[k] = pool[j].getCodon(k);
			}

			mjSys.setRealTime(false);
			mjSys.setGraphics(false);
			CPGController c = CPGController(parameters);
			Mediator m(&c, &mjSys);
			m.run(4);
			pool[j].setFitness(c.exit());
			mjSys.reset();
		}
		cout << endl;
		cout << "sorting " << pool.size() << " genomes.\n";
		std::sort(pool.rbegin(), pool.rend());

		cout << "epoch ended. Fitnesses (in order): " << endl;
		for(int j = 0; j < pool.size(); j++){
			cout << pool[j].getFitness() << endl;
		}
		//Cull the weak
		//cout << "culling unfit genomes" << endl;
		int inital_pop = pool.size();
		for(int j = population; j < inital_pop; j++){
			pool.pop_back();
		}
		if(i % 3 == 0){
			mjSys.setRealTime(true);
			mjSys.setGraphics(true);
			cout << "displaying fittest genome" << endl;
			double p[params];
			for(int k = 0; k < params; k++){
				p[k] = pool[0].getCodon(k);
			}
			CPGController c = CPGController(p);
			Mediator m(&c, &mjSys);
			m.run(4);
			pool[0].printGenome();
		}
		cout << "***************************\n" <<  endl;
	}

	return 0;
}
