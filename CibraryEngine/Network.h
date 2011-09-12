#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	struct Network
	{
		static boost::asio::io_service& GetIOService();

		static void StartAsyncSystem();
		static void StopAsyncSystem();

		static void DoTestProgram();
	};
}
