#ifndef MJRENDER_H
#define MJRENDER_H

#include "./include/mujoco.h"
#include "./include/glfw3.h"

class MujocoRenderer {
public:
	MujocoRenderer(mjModel * m);
	~MujocoRenderer();
	void render(mjData * d);
private:
	void scroll(GLFWwindow* window, double xoffset, double yoffset);
	void mouse_button(GLFWwindow * w, int button, int act, int mods);
	void mouse_move(GLFWwindow* window, double xpos, double ypos);
	void key_callback(GLFWwindow * w, int key, int scanCode, int action, int mods);
	
	mjtNum simStart;

	mjvCamera cam;                      // abstract camera
	mjvPerturb pert;                    // perturbation object
	mjvOption opt;                      // visualization options
	mjvScene scn;                       // abstract scene
	mjrContext con;                     // custom GPU context

	GLFWwindow * window;

	bool button_left;
	bool button_middle;
	bool button_right;
	double lastx, lasty;

	int lastKey;
	mjModel * model;
};

#endif