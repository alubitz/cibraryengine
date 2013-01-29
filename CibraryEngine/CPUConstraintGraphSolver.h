#pragma once

#include "StdAfx.h"
#include "ConstraintGraphSolver.h"

#include "Physics.h"

#define USE_MULTIPLE_THREADS 0

#if USE_MULTIPLE_THREADS
	#include "TaskThread.h"
#endif

namespace CibraryEngine
{
	using namespace std;

	class CPUConstraintGraphSolver : public ConstraintGraphSolver
	{
		public:

#if USE_MULTIPLE_THREADS
			static const unsigned int NUM_THREADS = 8;

			TaskThread* task_threads[NUM_THREADS];



			CPUConstraintGraphSolver()
			{
				for(unsigned int i = 0; i < NUM_THREADS; ++i)
					task_threads[i] = new TaskThread();
			}

			~CPUConstraintGraphSolver()
			{
				for(unsigned int i = 0; i < NUM_THREADS; ++i)
				{
					TaskThread*& task_thread = task_threads[i];

					task_thread->Shutdown();

					delete task_thread;
					task_thread = NULL;
				}
			}

#else

			void Solve(float timestep, unsigned int iterations, vector<PhysicsConstraint*>& constraints)
			{
				for(unsigned int i = 0; i < iterations; ++i)
					for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
						(*iter)->DoConstraintAction();
			}

#endif
	};
}
