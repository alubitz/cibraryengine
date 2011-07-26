#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	/** Class representing command line arguments */
	class CmdArgs
	{

	private:

		int		argc;
		char**	argv;

	public:

		CmdArgs();
		CmdArgs(int argc, char** argv);

	};
}