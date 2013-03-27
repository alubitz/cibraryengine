#include "StdAfx.h"
#include "CPUConstraintGraphSolver.h"

#include "RigidBody.h"

#define USE_SMART_ITERATION 0

namespace CibraryEngine
{
	/*
	 * CPUConstraintGraphSolver methods
	 */
	void CPUConstraintGraphSolver::Solve(float timestep, unsigned int iterations, vector<PhysicsConstraint*>& constraints)
	{
#if USE_SMART_ITERATION
		struct Edge
		{
			PhysicsConstraint* constraint;

			Edge *next, *nu_next;

			bool included;
			unsigned int wakeup_index;			// index into edge adjacency table
		};

		vector<Edge> edges(constraints.size());

		Edge* first = NULL;


		// map nodes (rigid bodies) to the edges adjacent to them
		unordered_map<RigidBody*, vector<Edge*> > body_constraints;
		for(unsigned int i = 0; i < edges.size(); ++i)
		{
			// create edge
			Edge* edge = &edges[i];
			PhysicsConstraint* constraint = edge->constraint = constraints[i];

			edge->next = edge->nu_next = first;
			first = edge;

			edge->included = false;


			// add edge's endpoints to the map
			RigidBody* obj_a = constraint->obj_a;
			unordered_map<RigidBody*, vector<Edge*> >::iterator found = body_constraints.find(obj_a);
			if(found == body_constraints.end())
			{
				vector<Edge*>& target = body_constraints[obj_a] = vector<Edge*>();
				target.push_back(edge);
			}
			else
				found->second.push_back(edge);

			RigidBody* obj_b = constraint->obj_b;
			if(obj_b->MergesSubgraphs())
			{
				found = body_constraints.find(obj_b);
				if(found == body_constraints.end())
				{
					vector<Edge*>& target = body_constraints[obj_b] = vector<Edge*>();
					target.push_back(edge);
				}
				else
					found->second.push_back(edge);
			}
		}


		// map edges to the edges adjacent to them
		vector<Edge*> edge_adjacency;
		edge_adjacency.reserve(constraints.size() * 3);

		for(unsigned int i = 0; i < edges.size(); ++i)
		{
			Edge* edge = &edges[i];
			PhysicsConstraint* constraint = edge->constraint;

			unordered_set<Edge*> adjacent(100);

			unordered_map<RigidBody*, vector<Edge*> >::iterator found = body_constraints.find(constraint->obj_a);
			if(found != body_constraints.end())
				adjacent.insert(found->second.begin(), found->second.end());

			found = body_constraints.find(constraint->obj_b);
			if(found != body_constraints.end())
				adjacent.insert(found->second.begin(), found->second.end());

			adjacent.erase(edge);

			edge->wakeup_index = edge_adjacency.size();

			edge_adjacency.insert(edge_adjacency.end(), adjacent.begin(), adjacent.end());
			edge_adjacency.push_back(NULL);
		}

		// iterate until there's no active constraints left to evaluate or we hit the max number of iterations
		for(unsigned int i = 0; i < iterations && first != NULL; ++i)
		{
			Edge* nu_first = NULL;

			for(Edge* edge = first; edge != NULL; edge = edge->next)
			{
				if(edge->constraint->DoConstraintAction())
				{
					for(Edge** adjacent_ptr = &edge_adjacency[edge->wakeup_index]; *adjacent_ptr != NULL; ++adjacent_ptr)
					{
						Edge* adjacent = *adjacent_ptr;

						if(!adjacent->included)
						{
							adjacent->included = true;
							adjacent->nu_next = nu_first;
							nu_first = adjacent;
						}
					}
				}
			}

			for(Edge* edge = nu_first; edge != NULL; edge = edge->next)
			{
				edge->next = edge->nu_next;
				edge->included = false;
			}

			first = nu_first;
		}
#else
		PhysicsConstraint **begin = constraints.data(), **end = begin + constraints.size();
		for(unsigned int i = 0; i < iterations; ++i)
			for(PhysicsConstraint** iter = begin; iter != end; ++iter)
				(*iter)->DoConstraintAction();
#endif
	}
}
