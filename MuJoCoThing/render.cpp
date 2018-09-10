#include "render.h"
#include <cstdio>

#ifndef LEGACY_SIMULATE

void mjRender::cb_scroll(GLFWwindow * window, double xoff, double yoff) {
	((mjRender*)(glfwGetWindowUserPointer(window)))->scroll(xoff, yoff);
}

void mjRender::cb_mousemove(GLFWwindow * window, double xpos, double ypos) {
	((mjRender*)(glfwGetWindowUserPointer(window)))->mouse_move(xpos, ypos);
}

void mjRender::cb_mousebutton(GLFWwindow * window, int button, int act, int mods) {
	((mjRender*)(glfwGetWindowUserPointer(window)))->mouse_button(button, act, mods);
}

void mjRender::cb_keyboard(GLFWwindow * window, int key, int scancode, int act, int mods) {
	((mjRender*)(glfwGetWindowUserPointer(window)))->keyboard(key, scancode, act, mods);
}

void mjRender::cb_close(GLFWwindow * window) {
	((mjRender*)(glfwGetWindowUserPointer(window)))->win_close();
}

mjRender::mjRender(mjModel * m) {
	this->model = m;
}

mjRender::~mjRender() {
	this->close();
}

void mjRender::scroll(double xoffset, double yoffset) {
	// require model
	if (!model)
		return;

	// scroll: emulate vertical mouse motion = 5% of window height
	mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

void mjRender::mouse_button(int button, int act, int mods) {
	button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
	button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
	button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
}

void mjRender::mouse_move(double xpos, double ypos) {
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

void mjRender::keyboard(int key, int scanCode, int action, int mods) {
	if (action == GLFW_PRESS) {
		lastKey = key;
	}
	if (action == GLFW_RELEASE) {
		lastKey = -1;
	}
}

void mjRender::win_close() {
	// close GLFW, free visualization storage
	glfwTerminate();
	mjv_freeScene(&scn);
	mjr_freeContext(&con);
}

void mjRender::init(const char * title, int xres, int yres) {
	if (window) return;

	glfwInit();
	window = glfwCreateWindow(xres, yres, "Minitaur", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	glfwSetKeyCallback(window, cb_keyboard);
	glfwSetMouseButtonCallback(window, cb_mousebutton);
	glfwSetCursorPosCallback(window, cb_mousemove);
	glfwSetScrollCallback(window, cb_scroll); 
	glfwSetWindowCloseCallback(window, cb_close);

	glfwSetWindowUserPointer(window, this);

	mjv_defaultCamera(&cam);	
	mjv_defaultPerturb(&pert);
	mjv_defaultOption(&opt);
	mjr_defaultContext(&con);
	mjv_makeScene(&scn, 1000);                     // space for 1000 objects
	mjr_makeContext(model, &con, mjFONTSCALE_100);     // model-specific context
}

void mjRender::render(mjData * d) {
	if (!window || glfwWindowShouldClose(window)) return;

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

bool mjRender::isOpen()
{
	return window;
}

void mjRender::close() {
	glfwDestroyWindow(window);
	this->window = NULL;
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
