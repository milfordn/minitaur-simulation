#include "MujocoRenderer.h"

MujocoRenderer::MujocoRenderer(mjModel * m)
{
	this->model = m;

	glfwInit();
	GLFWwindow * window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	glfwSetKeyCallback(window, this->key_callback);
	glfwSetMouseButtonCallback(window, this->mouse_button);
	glfwSetCursorPosCallback(window, this->mouse_move);
	glfwSetScrollCallback(window, this->scroll);

	mjv_defaultCamera(&cam);

	mjv_defaultPerturb(&pert);
	mjv_defaultOption(&opt);
	mjr_defaultContext(&con);
	mjv_makeScene(&scn, 1000);                     // space for 1000 objects
	mjr_makeContext(model, &con, mjFONTSCALE_100);     // model-specific context
}

MujocoRenderer::~MujocoRenderer(){
	glfwTerminate();
	mjv_freeScene(&scn);
	mjr_freeContext(&con);
}

void MujocoRenderer::render(mjData * d)
{		
	if (d->time - simStart < 1.0 / 60.0)
		return;

	simStart = d->time;

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

void MujocoRenderer::scroll(GLFWwindow * window, double xoffset, double yoffset){
	// scroll: emulate vertical mouse motion = 5% of window height
	mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

void MujocoRenderer::mouse_button(GLFWwindow * w, int button, int act, int mods){
	button_left = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
	button_middle = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
	button_right = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
}

void MujocoRenderer::mouse_move(GLFWwindow * window, double xpos, double ypos){	// no buttons down: nothing to do
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

void MujocoRenderer::key_callback(GLFWwindow * w, int key, int scanCode, int action, int mods)
{
	if (action == GLFW_PRESS) {
		lastKey = key;
	}
	if (action == GLFW_RELEASE) {
		lastKey = -1;
	}
}
