#include "render.h"

#ifndef LEGACY_SIMULATE

mjModel* model = NULL;                  // MuJoCo model
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

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
	// require model
	if (!model)
		return;

	// scroll: emulate vertical mouse motion = 5% of window height
	mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

void mouse_button(GLFWwindow * w, int button, int act, int mods) {
	button_left = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
	button_middle = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
	button_right = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
	// no buttons down: nothing to do
	if (!button_left && !button_middle && !button_right) {
		lastx = xpos;
		lasty = ypos;
		return;
	}

	// compute mouse displacement, save
	double dx = xpos - lastx;
	double dy = ypos - lasty;
	lastx = xpos;
	lasty = ypos;

	// require model
	if (!model)
		return;

	// get current window size
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	// get shift key state
	bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
		glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

	// determine action based on mouse button
	mjtMouse action;
	if (button_right)
		action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
	else if (button_left)
		action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
	else
		action = mjMOUSE_ZOOM;

	// move camera
	mjv_moveCamera(model, action, dx / height, dy / height, &scn, &cam);
}

void key_callback(GLFWwindow * w, int key, int scanCode, int action, int mods) {
	if (action == GLFW_PRESS) {
		lastKey = key;
	}
	if (action == GLFW_RELEASE) {
		lastKey = -1;
	}
}

void window_close_callback(GLFWwindow * w) {
	// close GLFW, free visualization storage
	glfwTerminate();
	mjv_freeScene(&scn);
	mjr_freeContext(&con);
}

void init(mjModel * m, int xres, int yres) {
	if (window) return;

	model = m;

	glfwInit();
	window = glfwCreateWindow(xres, yres, "Minitaur", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	glfwSetKeyCallback(window, key_callback);
	glfwSetMouseButtonCallback(window, mouse_button);
	glfwSetCursorPosCallback(window, mouse_move);
	glfwSetScrollCallback(window, scroll); 
	glfwSetWindowCloseCallback(window, window_close_callback);

	mjv_defaultCamera(&cam);	
	mjv_defaultPerturb(&pert);
	mjv_defaultOption(&opt);
	mjr_defaultContext(&con);
	mjv_makeScene(&scn, 1000);                     // space for 1000 objects
	mjr_makeContext(model, &con, mjFONTSCALE_100);     // model-specific context
}

void render(mjData * d) {
	if (!window) return;

	// get framebuffer viewport
	mjrRect viewport = { 0, 0, 0, 0 };
	glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

	// update scene and render
	mjv_updateScene(model, d, &opt, &pert, &cam, mjCAT_ALL, &scn);
	mjr_render(viewport, &scn, &con);

	// swap OpenGL buffers (blocking call due to v-sync)
	glfwSwapBuffers(window);

	// process pending GUI events, call GLFW callbacks
	glfwPollEvents();

}

void close() {
	if (window) glfwSetWindowShouldClose(window, true);
}

#else
#include <cstdio>

void init(mjModel * m, int a, int b) {
	puts("Legacy Simulate is enabled - Disable it to use the new features");
}

void render(mjData * d) {
	puts("Legacy Simulate is enabled - Disable it to use the new features");
}

void close() {
	puts("Legacy Simulate is enabled - Disable it to use the new features");
}
#endif