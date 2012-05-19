#include "StdAfx.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	ofstream* debug_logfile = NULL;

	void Debug(string s)
	{
		if(debug_logfile == NULL)
			debug_logfile = new ofstream("debug.txt");

		const char* cstr = s.c_str();
		debug_logfile->write(cstr, s.length());
		debug_logfile->flush();

		// also print to console
		printf(cstr);
	}

	void Debug(int line, const char* file) { Debug(((stringstream&)(stringstream() << "Executing line " << line << " of " << file << endl)).str()); }

	static void DisplayGLError(int error, int line, const char* file)
	{
		char* error_string;
		switch(error)
		{
			case GL_INVALID_FRAMEBUFFER_OPERATION:
				error_string = "GL_INVALID_FRAMEBUFFER_OPERATION";
				break;
			default:
				error_string = (char*)gluErrorString(error);
				break;
		}
		Debug(((stringstream&)(stringstream() << error_string << " at line " << line << " of " << file << endl)).str());
	}

	void GLErrorDebug(int line, const char* file)
	{
		if(int err = glGetError())
			DisplayGLError(err, line, file);
	}
	void GLErrorDebug(int line, const char* file, const string& no_error_message)
	{
		if(int err = glGetError())
			DisplayGLError(err, line, file);
		else if(no_error_message.length() != 0)
			Debug(((stringstream&)(stringstream() << no_error_message << " at line " << line << " of " << file << endl)).str());
	}
}
