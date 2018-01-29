#include "CustomSimulate.cpp"
#include <cstdio>

int main(int argc, char ** argv) {

	ModelController m("MinitaurLeg.xml");

	char * out = Simulate::run(m);

	if (out) puts(out);

	return 0;
}