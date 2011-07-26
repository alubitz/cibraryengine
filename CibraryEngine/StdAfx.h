#pragma once

#ifdef WIN32
	#include <windows.h>
#endif

// TODO: [maybe] switch to glee or glew sometime
#include <GL/gl.h>
#include <GL/gl3.h>
#include <GL/glext.h>
#include <GL/glu.h>
//#include <GL/glfw.h>
#include "GL2API.h"

#include <AL/al.h>
#include <AL/alc.h>

#include <stdio.h>
#include <time.h>

#include <iostream>
#include <fstream>
#include <sstream>

#include <vector>
#include <list>
#include <map>
#include <set>
#include <cmath>
// gotta manually define M_PI :(
#define M_PI		3.14159265358979323846
#include <cassert>
#include <string>

extern "C"
{
	#include <lua.h>
	#include <lualib.h>
	#include <lauxlib.h>
}
