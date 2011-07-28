#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	/** Writes a string to the debug log */
	void Debug(string s);

	void Debug(int line, string file);

	/** Writes the current line number and filename to the debug log */
#define DEBUG() \
	Debug(__LINE__, __FILE__)

	/** Like GLErrorDebug, but with __LINE__ and __FILE__ automatically provided */
#define GLDEBUG() \
	GLErrorDebug(__LINE__, __FILE__)

	/** If an OpenGL error has occurred, writes information to the debug log */
	void GLErrorDebug(int line, string file);
	/** If an OpenGL error has occurred, writes information to the debug log; otherwise, if it is not an empty string, outputs no_error_message to the debug log */
	void GLErrorDebug(int line, string file, string no_error_message);
}
