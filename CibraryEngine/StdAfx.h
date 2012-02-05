#pragma once

#ifdef WIN32
	#ifndef WIN32_LEAN_AND_MEAN
	#define WIN32_LEAN_AND_MEAN
	#endif

	#define _USE_MATH_DEFINES				// math constants like M_PI
	#define NOMINMAX						// no silly macros "min" and "max"

	#include <windows.h>
#endif

#include "GLee.h"
#include <GL/gl.h>
#include <GL/gl3.h>
#include <GL/glu.h>

#include <AL/al.h>
#include <AL/alc.h>

#include <stdio.h>
#include <time.h>

#include <iostream>
#include <fstream>
#include <sstream>

#define BOOST_SYSTEM_USE_LIB
#include <boost/asio.hpp>

#define BOOST_THREAD_USE_LIB
#include <boost/thread.hpp>

#include <boost/unordered/unordered_map.hpp>
#include <boost/unordered/unordered_set.hpp>

#include <vector>
#include <list>
#include <map>
#include <set>
#include <cmath>
#include <stack>
#include <cassert>
#include <string>

extern "C"
{
	#include <lua.h>
	#include <lualib.h>
	#include <lauxlib.h>
}
