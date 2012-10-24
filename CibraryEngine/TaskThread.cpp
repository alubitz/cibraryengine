#include "StdAfx.h"
#include "TaskThread.h"

namespace CibraryEngine
{
	/*
	 * TaskThread methods
	 */
	TaskThread::TaskThread() :
		task(NULL),
		stopped(false),
		my_thread(NULL),
		mutex(new boost::mutex()),
		cond(new boost::condition_variable())
	{
	}

	void TaskThread::StartTask(ThreadTask* task_)
	{
		task = task_;

		if(!my_thread)
			my_thread = new boost::thread(Runner(this));
		else
		{
			boost::unique_lock<boost::mutex> lock(*mutex);

			stopped = false;			
			cond->notify_one();
		}
	}

	void TaskThread::WaitForCompletion()
	{
		boost::unique_lock<boost::mutex> lock(*mutex);
		while(my_thread && !stopped)
			cond->wait(lock);
	}

	void TaskThread::Shutdown()
	{
		if(boost::thread* temp = my_thread)
		{
			{
				boost::unique_lock<boost::mutex> lock(*mutex);

				my_thread = NULL;
				cond->notify_one();
			}

			temp->join();
			delete temp;
		}

		if(mutex)	{ delete mutex;	mutex = NULL; }
		if(cond)	{ delete cond;	cond = NULL; }

	}

	void TaskThread::Run()
	{
		while(!my_thread) { }			// TODO: don't busy wait?

		while(my_thread)
		{
			task->DoTask();

			{	// curly braces for scope
				boost::unique_lock<boost::mutex> lock(*mutex);
				stopped = true;
			}

			// a call to WaitForCompletion should unblock once this point is reached
			cond->notify_one();

			{	// curly braces for scope; wait for a call to StartTask or Shutdown
				boost::unique_lock<boost::mutex> lock(*mutex);
				while(my_thread && stopped)
					cond->wait(lock);
			}
		}
	}
}
