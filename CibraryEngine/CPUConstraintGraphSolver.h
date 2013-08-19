#pragma once

#include "StdAfx.h"
#include "ConstraintGraphSolver.h"

#include "SmartHashSet.h"

#include "TaskThread.h"

#include "Physics.h"

namespace CibraryEngine
{
	using namespace std;

	class CPUConstraintGraphSolver : public ConstraintGraphSolver
	{
		public:

			struct SolverThread : public ThreadTask
			{
				vector<PhysicsConstraint*>* constraints;
				unsigned int from, to;

				void SetTaskParams(vector<PhysicsConstraint*>* constraints_, unsigned int from_, unsigned int to_) { constraints = constraints_; from = from_; to = to_; }

				void DoTask()
				{
					PhysicsConstraint **data = constraints->data(), **start_ptr = data + from, **finish_ptr = data + to;
					for(PhysicsConstraint** iter = start_ptr; iter != finish_ptr; ++iter)
						(*iter)->DoConstraintAction();
				}
			};
			vector<SolverThread> solver_threads;
			vector<TaskThread*>* task_threads;

			CPUConstraintGraphSolver(vector<TaskThread*>* task_threads) : task_threads(task_threads), solver_threads(task_threads->size()) { }

			void Solve(float timestep, unsigned int iterations, vector<PhysicsConstraint*>& constraints);
	};
}
