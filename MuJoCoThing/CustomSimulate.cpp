// MuJoCoThing.cpp : Defines the entry point for the console application.
//

#include <cstdio>
#include "include\mujoco.h"
#include "include\glfw3.h"

mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvPerturb pert;                    // perturbation object
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

bool button_left;
bool button_middle;
bool button_right;
double lastx, lasty;

bool hit;
bool prep;
double gospeed;

int lastKey;

char err[1000];

void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
	// require model
	if (!m)
		return;

	// scroll: emulate vertical mouse motion = 5% of window height
	mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

void mouse_button(GLFWwindow * w, int button, int act, int mods) {
	button_left = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
	button_middle = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
	button_right = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
	// no buttons down: nothing to do
	if (!button_left && !button_middle && !button_right) 
		return;

	// compute mouse displacement, save
	double dx = xpos - lastx;
	double dy = ypos - lasty;
	lastx = xpos;
	lasty = ypos;

	// require model
	if (!m)
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
	mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void key_callback(GLFWwindow * w, int key, int scanCode, int action, int mods) {
	if (action == GLFW_PRESS) {
		lastKey = key;
	}
	if (action == GLFW_RELEASE) {
		lastKey = -1;
	}

}

void stepOverCenter(mjModel * m, mjData * d) {
	bool go = lastKey == GLFW_KEY_SPACE;

	//printf("before: %f\n", d->qvel[0]);
	if (d->qvel[0] <= 0) {
		d->qvel[0] = 0;
		d->qacc[0] = 0;
		mj_forward(m, d);
	}
	//printf("after: %f\n", d->qvel[0]);

	if (hit && !d->sensordata[0]) {
		hit = false;
		prep = true;
		gospeed = 0;
		puts("stopped");
	}
	else if (!hit && d->sensordata[0]) {
		hit = true;
		prep = false;
		gospeed = 0;
		puts("done");
	}
	else if (go && hit) {
		gospeed = 10;
		puts("going");
	}
	else if (!prep && !hit && go) {
		gospeed = 10;
		puts("prep");
	}

	d->ctrl[0] = go ? 10 : 1;
}

void stepWCD(mjModel * m, mjData * d) {

	int power = 100;

	//evens left, odds right

	if (lastKey == GLFW_KEY_W) {
		d->ctrl[0] = d->ctrl[1] = power;
	}
	else if (lastKey == GLFW_KEY_S) {
		d->ctrl[0] = d->ctrl[1] = -power;
	}
	else if (lastKey == GLFW_KEY_A) {
		d->ctrl[0] = -power;
		d->ctrl[1] = power;
	}
	else if (lastKey == GLFW_KEY_D) {
		d->ctrl[0] = power;
		d->ctrl[1] = -power;
	}
	else {
		d->ctrl[0] = d->ctrl[1] = 0;
	}

	for (int i = 0; i < 6; i++) {
		d->ctrl[i] = d->ctrl[i % 2];
	}
}

int main()
{
	mj_activate("mjkey.txt");

	//replace OverCenterModel with any model you choose
	m = mj_loadXML("MinitaurLeg.xml", NULL, err, sizeof(err));
	d = mj_makeData(m);

	glfwInit();
	GLFWwindow * window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1); 

	glfwSetKeyCallback(window, key_callback);
	glfwSetMouseButtonCallback(window, mouse_button);
	glfwSetCursorPosCallback(window, mouse_move);
	glfwSetScrollCallback(window, scroll);
	
	mjv_defaultCamera(&cam);
	if (m->ncam > 0) {
		cam.type = mjCAMERA_FIXED;
		cam.fixedcamid = 0;
	}

	mjv_defaultPerturb(&pert);
	mjv_defaultOption(&opt);
	mjr_defaultContext(&con);
	mjv_makeScene(&scn, 1000);                     // space for 1000 objects
	mjr_makeContext(m, &con, mjFONTSCALE_100);     // model-specific context

	hit = false;

	while (!glfwWindowShouldClose(window)) {

		mjtNum simstart = d->time;


		//printf("%f\n", d->sensordata[0]);

		while (d->time - simstart < 1.0 / 60.0) {
			mj_step1(m, d);

			//replace this with the step function for your specific model
			//stepOverCenter(m, d);
			mj_step2(m, d);
		}
		
		// get framebuffer viewport
		mjrRect viewport = { 0, 0, 0, 0 };
		glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

		// update scene and render
		mjv_updateScene(m, d, &opt, &pert, &cam, mjCAT_ALL, &scn);
		mjr_render(viewport, &scn, &con);

		// swap OpenGL buffers (blocking call due to v-sync)
		glfwSwapBuffers(window);

		// process pending GUI events, call GLFW callbacks
		glfwPollEvents();
	}

	// close GLFW, free visualization storage
	glfwTerminate();
	mjv_freeScene(&scn);
	mjr_freeContext(&con);

	mj_deleteModel(m);
	mj_deleteData(d);

    return 0;
}
