#ifndef RENDER_H
#define RENDER_H

#include "include/mujoco.h"
#include "include/glfw3.h"

class mjRender {
public:
	mjRender(mjModel * m);
	~mjRender();

	void init(const char * title, int xres, int yres);
	void close();
	void render(mjData * d);
	static void cb_scroll(GLFWwindow * window, double xoff, double yoff);
	static void cb_mousemove(GLFWwindow * window, double xpos, double ypos);
	static void cb_mousebutton(GLFWwindow * window, int button, int act, int mods);
	static void cb_keyboard(GLFWwindow * window, int key, int scancode, int act, int mods);
	static void cb_close(GLFWwindow * window);

protected:
	void scroll(double xoff, double yoff);
	void mouse_button(int btn, int act, int mods);
	void mouse_move(double xpos, double ypos);
	void keyboard(int key, int scancode, int act, int mods);
	void win_close();

private:
	mjModel * model; 
	mjvCamera cam;                      // abstract camera
	mjvPerturb pert;                    // perturbation object
	mjvOption opt;                      // visualization options
	mjvScene scn;                       // abstract scene
	mjrContext con;                     // custom GPU context

	mjtNum simstart;

	GLFWwindow * window = NULL;

	bool button_left;
	bool button_middle;
	bool button_right;
	double lastx, lasty;

	int lastKey;
};

#endif