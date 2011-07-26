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
			string error_string = (char*)gluErrorString(err);
			char msg[100];
			sprintf(msg, "%s at line %u of %s\n", error_string.c_str(), line, file.c_str());
			Debug(msg);
		}
		else if(no_error_message.length() != 0)
		{
			char msg[100];
			sprintf(msg, "%s at line %u of %s\n", no_error_message.c_str(), line, file.c_str());
			Debug(msg);
		}
	}
}
