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
		bool shutdown;

		thread* my_thread;
		mutex* my_mutex;
		condition_variable* cond;

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
