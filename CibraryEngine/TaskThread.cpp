#include "StdAfx.h"
#include "TaskThread.h"

#include <condition_variable>

namespace CibraryEngine
{
	/*
	 * TaskThread methods
	 */
	TaskThread::TaskThread() :
		task(NULL),
		shutdown(false),
		my_thread(NULL),
		my_mutex(new std::mutex()),
		cond(new condition_variable())
	{
	}

	void TaskThread::StartTask(ThreadTask* task_)
	{
		unique_lock<std::mutex> lock(*my_mutex);
		if (shutdown || task != NULL)
			return;

		task = task_;
		if (my_thread == NULL)
			my_thread = new thread(Runner(this));
		else
			cond->notify_all();
	}

	void TaskThread::WaitForCompletion()
	{
		cond->wait(unique_lock<std::mutex>(*my_mutex), [&] { return task == NULL; });
	}

	void TaskThread::Shutdown()
	{
		{
			unique_lock<std::mutex> lock(*my_mutex);
			shutdown = true;
			cond->notify_all();
		}

		if (my_thread != NULL)
		{
			my_thread->join();
			delete my_thread;
			my_thread = NULL;
		}

		if(my_mutex) { delete my_mutex;	my_mutex = NULL; }
		if(cond) { delete cond; cond = NULL; }

	}

	void TaskThread::Run()
	{
		bool looping = true;
		while (looping)
		{
			unique_lock<std::mutex> lock(*my_mutex);
			cond->wait(lock, [&] { return shutdown || task != NULL; });
			if (shutdown)
				looping = false;
			else if (task != NULL)
			{
				task->DoTask();
				task = NULL;
				cond->notify_all();
			}
		}
	}
}
