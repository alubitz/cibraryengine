// this define affects how GL2API.h is processed, don't remove it!
#define GL2LOCAL
#include "GL2API.h"

#include "StdAfx.h"

#include "DebugLog.h"

using namespace std;

void GetProcFailureMessage(const char* name)
{
	stringstream ss;
	ss << "Failed to load OpenGL extension function " << name << endl;
	CibraryEngine::Debug(ss.str());
}