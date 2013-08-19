#include "StdAfx.h"
#include "CPUConstraintGraphSolver.h"

#include "RigidBody.h"

#include "TaskThread.h"

#include "DebugLog.h"

#define STOP_BATCHING_THRESHOLD		100					// when there are less than this many constraints left to put in batches, just lump all the rest in sequentially
#define MULTITHREADING_THRESHOLD	40					// if there are more than this many constraints in a batch, do multithreading
#define HASH_SIZE_PER_CONSTRAINT	5					// hash set will have have room for this * (number of constraints) rigid bodies

#define DEBUG_HASH_COLLISIONS		0

#define MAX_THREADS					4u

namespace CibraryEngine
{
	/*
	 * CPUConstraintGraphSolver methods
	 */
	void CPUConstraintGraphSolver::Solve(float timestep, unsigned int iterations, vector<PhysicsConstraint*>& constraints)
	{
		// collect batches containing no adjacent edges
		vector<vector<PhysicsConstraint*>> batches;

		// we'll be doing some back-and-forth updating between these two vectors
		vector<PhysicsConstraint*> remaining_a, remaining_b;
		remaining_a.reserve(constraints.size());
		remaining_b.reserve(constraints.size());

		vector<PhysicsConstraint*>& remaining		= constraints;
		vector<PhysicsConstraint*>& nu_remaining	= remaining_b;

		struct RBHashSet
		{
#if DEBUG_HASH_COLLISIONS
			unsigned int searches;
			unsigned int hash_collisions;
#endif

			unsigned int size;
			RigidBody** data;
			RigidBody** end;

			RBHashSet(unsigned int max_size) :
#if DEBUG_HASH_COLLISIONS
				searches(0),
				hash_collisions(0),
#endif
				size(max_size),
				data(new RigidBody*[max_size]),
				end(data + max_size)
			{
			}

			~RBHashSet()
			{
				delete[] data;
#if DEBUG_HASH_COLLISIONS
				Debug(((stringstream&)(stringstream() << hash_collisions << " increments / " << searches << " searches = " << (float)hash_collisions / searches << endl)).str());
#endif
			}

			void Reset(unsigned int size_)
			{
				assert(size_ <= size);

				memset(data, 0, size_ * sizeof(RigidBody*));
				end = data + size_;

				size = size_;
			}

			RigidBody** Find(RigidBody* query)
			{
#if DEBUG_HASH_COLLISIONS
				++searches;
#endif
				RigidBody **search_start = data + ((unsigned int)query / sizeof(RigidBody)) % size, **ptr = search_start;
				while(RigidBody* stored = *ptr)
				{
					if(stored == query)
						return ptr;
					else
					{
						++ptr;
#if DEBUG_HASH_COLLISIONS
						++hash_collisions;
#endif
						if(ptr == end)
							ptr = data;
						else if(ptr == search_start)
							return NULL;
					}
				}

				return ptr;
			}

		} batch_bodies(remaining.size() * HASH_SIZE_PER_CONSTRAINT + 1);

		// find batches containing no adjacent constraint edges (i.e. no shared rigid bodies, unless they're immobile)
		while(remaining.size() >= STOP_BATCHING_THRESHOLD)
		{
			nu_remaining.clear();
			batch_bodies.Reset(remaining.size() * HASH_SIZE_PER_CONSTRAINT + 1);

			batches.push_back(vector<PhysicsConstraint*>());
			vector<PhysicsConstraint*>& batch = *batches.rbegin();

			for(PhysicsConstraint **iter = remaining.data(), **end = iter + remaining.size(); iter != end; ++iter)
			{
				PhysicsConstraint* c = *iter;

				bool ok = true;

				// find out if this constraint edge is adjacent to any of the ones already in this batch; if it is, this edge will have to wait for another batch
				RigidBody** found_a = NULL;
				if(c->obj_a->MergesSubgraphs())
				{
					found_a = batch_bodies.Find(c->obj_a);

					if(found_a == NULL)
						ok = false;
					else if(*found_a != NULL)
						ok = false;
					else
						*found_a = c->obj_a;					// temporarily fill this position (ensures subsequent call to Find doesn't return the same blank)
				}

				if(ok)
				{
					RigidBody** found_b = NULL;
					if(c->obj_b->MergesSubgraphs())
					{
						found_b = batch_bodies.Find(c->obj_b);

						if(found_b == NULL)
							ok = false;
						else if(*found_b != NULL)
							ok = false;
						else
							*found_b = c->obj_b;				// temporarily fill this position
					}

					// if ok, add to batch; batch_bodies insertions will be permanent
					if(ok)
						batch.push_back(c);
					else if(found_b != NULL)
						*found_b = NULL;
				}

				if(!ok)
				{
					if(found_a != NULL)
						*found_a = NULL;

					nu_remaining.push_back(c);
				}
			}

			// would just use swap, but we want to start out using param "constraints", rather than having to copy its contents to "remaining" first
			if(nu_remaining == remaining_b)
			{
				remaining		= remaining_b;
				nu_remaining	= remaining_a;
			}
			else
			{
				remaining		= remaining_a;
				nu_remaining	= remaining_b;
			}
		}

		// now process those batches however many times in a row
		unsigned int max_threads = min(task_threads->size(), MAX_THREADS);
		solver_threads.resize(max_threads);

		// TODO: zomg thread safety! particularly in collision callbacks
		vector<PhysicsConstraint*> *batches_begin = batches.data(), *batches_end = batches_begin + batches.size();
		for(unsigned int i = 0; i < iterations; ++i)
		{
			for(vector<PhysicsConstraint*>* iter = batches_begin; iter != batches_end; ++iter)
			{
				vector<PhysicsConstraint*>& batch = *iter;
				unsigned int num_constraints = batch.size();
				unsigned int use_threads = num_constraints > MULTITHREADING_THRESHOLD ? max_threads : 1;

				if(use_threads > 1)
				{
					// split the work of evaluating the constraints in this batch across multiple threads
					for(unsigned int j = 0; j < use_threads; ++j)
					{
						solver_threads[j].SetTaskParams(&batch, j * num_constraints / use_threads, (j + 1) * num_constraints / use_threads);
						(*task_threads)[j]->StartTask(&solver_threads[j]);
					}
					for(unsigned int j = 0; j < use_threads; ++j)
						(*task_threads)[j]->WaitForCompletion();
				}
				else
				{
					// this batch is so small doing multithreading may not be worthwhile; evaluate these constraints sequentially
					for(PhysicsConstraint **iter = batch.data(), **end = iter + batch.size(); iter != end; ++iter)
						(*iter)->DoConstraintAction();
				}
			}

			// evaluate un-batched constraints sequentially
			if(!remaining.empty())
			{
				for(PhysicsConstraint **iter = remaining.data(), **end = iter + remaining.size(); iter != end; ++iter)
					(*iter)->DoConstraintAction();
			}
		}
	}
}
