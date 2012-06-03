#include "StdAfx.h"
#include "CollisionGraph.h"
#include "RigidBody.h"

namespace CibraryEngine
{
	using namespace std;




	/*
	 * CollisionGraph methods
	 */
	CollisionGraph::CollisionGraph() : nodes(), contact_points() { }
	CollisionGraph::~CollisionGraph()
	{
		for(map<RigidBody*, Node*>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
			delete iter->second;
		nodes.clear();

		for(vector<ContactPoint*>::iterator iter = contact_points.begin(); iter != contact_points.end(); ++iter)
			delete *iter;
		contact_points.clear();
	}

	void CollisionGraph::AddContactPoint(const ContactPoint& cp)
	{
		RigidBody* body_a = cp.a.obj;
		if(body_a->MergesSubgraphs())
		{
			unsigned int cp_index = contact_points.size();
			ContactPoint* cp_ptr = new ContactPoint(cp);
			contact_points.push_back(cp_ptr);

			Node* node_a;
			map<RigidBody*, Node*>::iterator found_a = nodes.find(body_a);
			if(found_a != nodes.end())
				node_a = found_a->second;
			else
				node_a = nodes[body_a] = new Node(body_a);

			RigidBody* body_b = cp.b.obj;
			if(body_b->MergesSubgraphs())
			{
				Node* node_b;
				map<RigidBody*, Node*>::iterator found_b = nodes.find(body_b);
				if(found_b != nodes.end())
					node_b = found_b->second;
				else
					node_b = nodes[body_b] = new Node(body_b);

				node_a->edges.push_back(Edge(cp_ptr, &cp_ptr->a, &cp_ptr->b, node_b));
				node_b->edges.push_back(Edge(cp_ptr, &cp_ptr->b, &cp_ptr->a, node_a));
			}
			else
				node_a->edges.push_back(Edge(cp_ptr, &cp_ptr->a, &cp_ptr->b, NULL));
		}
		else
		{
			// TODO: output to the effect of "excuse me wtf r u doin"
		}
	}
}