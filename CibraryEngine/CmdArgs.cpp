#include "StdAfx.h"
#include "CmdArgs.h"

namespace CibraryEngine
{
	/*
	 * CmdArgs methods
	 */
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
}
