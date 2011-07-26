#include "CmdArgs.h"

using namespace CibraryEngine;

CmdArgs::CmdArgs(int argc, char** argv)
{
	this->argc = argc;
	this->argv = argv;
}

CmdArgs::CmdArgs()
{
	argc = 0;
	argv = NULL;
}
