#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	class ThreadTask
	{
		public:
			virtual void DoTask() = 0;
	};

	struct TaskThread
	{
		ThreadTask* task;
		bool stopped;

		boost::thread* my_thread;
		boost::mutex* mutex;
		boost::condition_variable* cond;

		TaskThread();

		void StartTask(ThreadTask* task);
		void WaitForCompletion();
		void Shutdown();									// called to inform the thread that it is no longer needed and should shut down

		void Run();
				
		struct Runner
		{
			TaskThread* data;
			Runner(TaskThread* data) : data(data) { }

			void operator()() { data->Run(); }				// thread's "main" function
		};
	};
}
