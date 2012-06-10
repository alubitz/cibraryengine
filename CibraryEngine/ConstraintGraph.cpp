#include "StdAfx.h"
#include "ConstraintGraph.h"
#include "RigidBody.h"

namespace CibraryEngine
{
	using namespace std;

	using boost::unordered_map;




	/*
	 * Recycle bins for ContactPoint and ConstraintGraph::Node pointers
	 */
	static vector<ContactPoint*> cp_recycle_bin;
	static vector<ConstraintGraph::Node*> node_recycle_bin;

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
	static void DeleteCP(ContactPoint* cp) { cp_recycle_bin.push_back(cp); }

	static ConstraintGraph::Node* NewNode(RigidBody* body)
	{
		if(node_recycle_bin.empty())
			return new ConstraintGraph::Node(body);
		else
		{
			ConstraintGraph::Node* result = new (*node_recycle_bin.rbegin()) ConstraintGraph::Node(body);
			node_recycle_bin.pop_back();
			return result;
		}
	}
	static void DeleteNode(ConstraintGraph::Node* node) { node_recycle_bin.push_back(node); }




	/*
	 * ConstraintGraph methods
	 */
	ConstraintGraph::ConstraintGraph() : nodes(), constraints(), contact_points() { }
	ConstraintGraph::~ConstraintGraph()
	{
		for(unordered_map<RigidBody*, Node*>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
			DeleteNode(iter->second);
		nodes.clear();

		for(vector<ContactPoint*>::iterator iter = contact_points.begin(); iter != contact_points.end(); ++iter)
			DeleteCP(*iter);
		constraints.clear();
	}

	void ConstraintGraph::AddContactPoint(const ContactPoint& cp)
	{
		ContactPoint* cp_ptr = NewCP(cp);
		contact_points.push_back(cp_ptr);
		AddConstraint(cp_ptr);
	}

	void ConstraintGraph::AddConstraint(PhysicsConstraint* constraint)
	{
		RigidBody* body_a = constraint->obj_a->GetCollisionProxy();
		if(body_a->MergesSubgraphs())
		{
			unsigned int cp_index = constraints.size();
			constraints.push_back(constraint);

			Node* node_a;
			unordered_map<RigidBody*, Node*>::iterator found_a = nodes.find(body_a);
			if(found_a != nodes.end())
				node_a = found_a->second;
			else
			{
				node_a = new Node(body_a);
				nodes.insert(pair<RigidBody*, Node*>(body_a, node_a));
			}

			RigidBody* body_b = constraint->obj_b->GetCollisionProxy();
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

				node_a->edges.push_back(Edge(constraint, node_b));
				node_b->edges.push_back(Edge(constraint, node_a));
			}
			else
				node_a->edges.push_back(Edge(constraint, NULL));
		}
		else
		{
			// TODO: output to the effect of "excuse me wtf r u doin"
		}
	}

	void ConstraintGraph::EmptyRecycleBins()
	{
		for(vector<ContactPoint*>::iterator iter = cp_recycle_bin.begin(); iter != cp_recycle_bin.end(); ++iter)
			delete *iter;
		cp_recycle_bin.clear();

		for(vector<Node*>::iterator iter = node_recycle_bin.begin(); iter != node_recycle_bin.end(); ++iter)
			delete *iter;
		node_recycle_bin.clear();
	}
}