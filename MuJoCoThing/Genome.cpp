#include "./Genome.h"
#include <stdlib.h>
#include <math.h>
#include <iostream>
using namespace std;

Genome::Genome(int length, int size){
  this->length = length;
  this->codonLength = size;
  this->genome = new int[length*size];
  this->fitness = 0;
  for(int i = 0; i < length*size; i++){
    if(rand() & 1) genome[i] = 0;
    else genome[i] = 1;
  }
}
Genome::Genome(const Genome& g){
  this->length = g.length;
  this->codonLength = g.codonLength;
  this->genome = new int[length*codonLength];
  this->fitness = g.fitness;
  for(int i = 0; i < length*codonLength; i++){
    this->genome[i] = g.genome[i];
  }
}
Genome Genome::operator=(const Genome& g){
  this->length = g.length;
  this->codonLength = g.codonLength;
  this->fitness = g.fitness;
  for(int i = 0; i < length*codonLength; i++){
    genome[i] = g.genome[i];
  }
  return *this;
}
Genome::~Genome(){
  free(genome);
}
Genome Genome::crossbreed(const Genome& partner) const{
  Genome offspring(length, codonLength);
  for(int i = 0; i < length*codonLength; i+= codonLength){
    if(rand() & 1){
      for(int j = 0; j < codonLength; j++){
        offspring.genome[i+j] = partner.genome[i+j];
      }
    }else{
      for(int j = 0; j < codonLength; j++){
        offspring.genome[i+j] = genome[i+j];
      }
    }
  }
  if(rand() % 100 < 2) {
    int idx = rand()%(length*codonLength);
    offspring.genome[idx] = !offspring.genome[idx];
  }
  return offspring;
}
double Genome::getCodon(int index){
  double sum = 0;
  for(int i = index*codonLength; i < index*codonLength + codonLength; i++){
    sum += genome[i] * pow(2, i - index*codonLength);
  }
  return sum/50000;
}
void Genome::printGenome(){
  for(int i = 0; i < length*codonLength; i++){
    cout << genome[i];
  }
  cout << endl;
}
void Genome::setFitness(double fitness){
  this->fitness = fitness;
}
double Genome::getFitness(){
  return this->fitness;
}
