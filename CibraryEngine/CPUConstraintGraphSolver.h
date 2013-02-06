#pragma once

#include "StdAfx.h"
#include "ConstraintGraphSolver.h"

#include "SmartHashSet.h"

#include "Physics.h"

namespace CibraryEngine
{
	using namespace std;

	class CPUConstraintGraphSolver : public ConstraintGraphSolver
	{
		public:

			void Solve(float timestep, unsigned int iterations, vector<PhysicsConstraint*>& constraints)
			{
#if 1
				unordered_map<RigidBody*, vector<PhysicsConstraint*> > body_constraints;
				for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
				{
					PhysicsConstraint* pc = *iter;

					RigidBody* obj_a = pc->obj_a;
					unordered_map<RigidBody*, vector<PhysicsConstraint*> >::iterator found = body_constraints.find(obj_a);
					if(found == body_constraints.end())
					{
						vector<PhysicsConstraint*>& target = body_constraints[obj_a] = vector<PhysicsConstraint*>();
						target.push_back(pc);
					}
					else
						found->second.push_back(pc);

					RigidBody* obj_b = pc->obj_b;
					if(obj_b->MergesSubgraphs())
					{
						found = body_constraints.find(obj_b);
						if(found == body_constraints.end())
						{
							vector<PhysicsConstraint*>& target = body_constraints[obj_b] = vector<PhysicsConstraint*>();
							target.push_back(pc);
						}
						else
							found->second.push_back(pc);
					}
				}

				unordered_map<PhysicsConstraint*, vector<PhysicsConstraint*> > wakeup_map;
				SmartHashSet<PhysicsConstraint, 13> constraints_a, constraints_b, *active_constraints = &constraints_a, *nu_active_constraints = &constraints_b;

				for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
				{
					PhysicsConstraint* pc = *iter;
					constraints_a.Insert(pc);
					
					unordered_set<PhysicsConstraint*> adjacent(100);

					unordered_map<RigidBody*, vector<PhysicsConstraint*> >::iterator found = body_constraints.find(pc->obj_a);
					if(found != body_constraints.end())
						adjacent.insert(found->second.begin(), found->second.end());

					found = body_constraints.find(pc->obj_b);
					if(found != body_constraints.end())
						adjacent.insert(found->second.begin(), found->second.end());

					adjacent.erase(pc);

					wakeup_map[pc] = vector<PhysicsConstraint*>(adjacent.begin(), adjacent.end());
				}

				for(unsigned int i = 0; i < iterations; ++i)
				{
					nu_active_constraints->Clear();

					Debug(((stringstream&)(stringstream() << "iteration " << i << ";\tactive constraints = " << active_constraints->count << endl)).str());

					for(unsigned int j = 0; j < active_constraints->hash_size; ++j)
						for(vector<PhysicsConstraint*>::iterator iter = active_constraints->buckets[j].begin(), bucket_end = active_constraints->buckets[j].end(); iter != bucket_end; ++iter)
							if((*iter)->DoConstraintAction())
							{
								vector<PhysicsConstraint*>& wakeup_list = wakeup_map[*iter];
								for(vector<PhysicsConstraint*>::iterator jter = wakeup_list.begin(); jter != wakeup_list.end(); ++jter)
									nu_active_constraints->Insert(*jter);
							}

					if(nu_active_constraints->count == 0)
						break;
					else
						swap(active_constraints, nu_active_constraints);
				}

				Debug("\n");
#else

				for(unsigned int i = 0; i < iterations; ++i)
					for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
						(*iter)->DoConstraintAction();
#endif
			}
	};
}
