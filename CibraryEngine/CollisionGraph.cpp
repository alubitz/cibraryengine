#include "StdAfx.h"
#include "CollisionGraph.h"
#include "RigidBody.h"

namespace CibraryEngine
{
	using namespace std;

	using boost::unordered_map;




	/*
	 * Recycle bins for ContactPoint and CollisionGraph::Node pointers
	 */
	static vector<ContactPoint*> cp_recycle_bin;
	static vector<CollisionGraph::Node*> node_recycle_bin;

	static ContactPoint* NewCP(const ContactPoint& cp)
	{
		if(cp_recycle_bin.empty())
			return new ContactPoint(cp);
		else
		{
			ContactPoint* result = new (*cp_recycle_bin.rbegin()) ContactPoint(cp);
			cp_recycle_bin.pop_back();
			return result;
		}
	}

	static CollisionGraph::Node* NewNode(RigidBody* body)
	{
		if(node_recycle_bin.empty())
			return new CollisionGraph::Node(body);
		else
		{
			CollisionGraph::Node* result = new (*node_recycle_bin.rbegin()) CollisionGraph::Node(body);
			node_recycle_bin.pop_back();
			return result;
		}
	}

	static void DeleteCP(ContactPoint* cp) { cp_recycle_bin.push_back(cp); }
	static void DeleteNode(CollisionGraph::Node* node) { node_recycle_bin.push_back(node); }




	/*
	 * CollisionGraph methods
	 */
	CollisionGraph::CollisionGraph() : nodes(), contact_points() { }
	CollisionGraph::~CollisionGraph()
	{
		for(unordered_map<RigidBody*, Node*>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
			DeleteNode(iter->second);
		nodes.clear();

		for(vector<ContactPoint*>::iterator iter = contact_points.begin(); iter != contact_points.end(); ++iter)
			DeleteCP(*iter);
		contact_points.clear();
	}

	void CollisionGraph::AddContactPoint(const ContactPoint& cp)
	{
		RigidBody* body_a = cp.a.obj->GetCollisionProxy();
		if(body_a->MergesSubgraphs())
		{
			unsigned int cp_index = contact_points.size();
			ContactPoint* cp_ptr = NewCP(cp);
			contact_points.push_back(cp_ptr);

			Node* node_a;
			unordered_map<RigidBody*, Node*>::iterator found_a = nodes.find(body_a);
			if(found_a != nodes.end())
				node_a = found_a->second;
			else
			{
				node_a = new Node(body_a);
				nodes.insert(pair<RigidBody*, Node*>(body_a, node_a));
			}

			RigidBody* body_b = cp.b.obj->GetCollisionProxy();
			if(body_b->MergesSubgraphs())
			{
				Node* node_b;
				unordered_map<RigidBody*, Node*>::iterator found_b = nodes.find(body_b);
				if(found_b != nodes.end())
					node_b = found_b->second;
				else
				{
					node_b = NewNode(body_b);
					nodes.insert(pair<RigidBody*, Node*>(body_b, node_b));
				}

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

	void CollisionGraph::EmptyRecycleBins()
	{
		for(vector<ContactPoint*>::iterator iter = cp_recycle_bin.begin(); iter != cp_recycle_bin.end(); ++iter)
			delete *iter;
		cp_recycle_bin.clear();

		for(vector<Node*>::iterator iter = node_recycle_bin.begin(); iter != node_recycle_bin.end(); ++iter)
			delete *iter;
		node_recycle_bin.clear();
	}
}