#include "StdAfx.h"

#include "Network.h"

#define BOOST_THREAD_USE_LIB
#include <boost/thread.hpp>

namespace CibraryEngine
{
	using namespace boost::asio;




	/*
	 * Network methods
	 */
	io_service* global_io_service = NULL;
	io_service& Network::GetIOService()
	{
		if(global_io_service == NULL)
			global_io_service = new io_service();

		return *global_io_service;
	}

	void DoAsyncStuff();

	boost::thread* async_thread = NULL;
	bool async_go = false;
	void Network::StartAsyncSystem()
	{
		if(async_thread == NULL)
		{
			async_go = true;
			async_thread = new boost::thread(DoAsyncStuff);
		}
	}

	void Network::StopAsyncSystem()
	{
		if(async_go)
			async_go = false;
	}




	void DoAsyncStuff()
	{
		while(async_go)
			Network::GetIOService().run_one();
	}
}
