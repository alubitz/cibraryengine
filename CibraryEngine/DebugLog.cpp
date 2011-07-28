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

	void Debug(int line, string file)
	{
		char msg[100];
		sprintf(msg, "Executing line %u of %s\n", line, file.c_str());
		Debug(msg);
	}

	void GLErrorDebug(int line, string file) { GLErrorDebug(line, file, string()); }
	void GLErrorDebug(int line, string file, string no_error_message)
	{
		int err = glGetError();
		if(err != 0)
		{
			string error_string;
			switch(err)
			{
				case GL_INVALID_FRAMEBUFFER_OPERATION:
					error_string = "GL_INVALID_FRAMEBUFFER_OPERATION";
					break;
				default:
					error_string = (char*)gluErrorString(err);
					break;
			}
			stringstream ss;
			ss << error_string << " at line " << line << " of " << file << endl;
			Debug(ss.str());
		}
		else if(no_error_message.length() != 0)
		{
			stringstream ss;
			ss << no_error_message << " at line " << line << " of " << file << endl;
			Debug(ss.str());
		}
	}
}
