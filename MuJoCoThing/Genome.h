#ifndef GENETIC_H
#define GENETIC_H

class Genome{
public:
  Genome(int length, int codonLength);
  Genome(const Genome& g);
  Genome operator=(const Genome& g);
  Genome crossbreed(const Genome& parent) const;
  double getCodon(int index);

  void printGenome();
  ~Genome();
private:
  int* genome;
  int length;
  int codonLength;
  double fitness;
};

#endif
