#ifndef RENDER_H
#define RENDER_H

#include "include/mujoco.h"
#include "include/glfw3.h"

void init(mjModel * m, int xres, int yres);
void render(mjData * d);
void close();

#endif