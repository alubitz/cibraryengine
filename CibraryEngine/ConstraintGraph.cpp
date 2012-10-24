#include "StdAfx.h"
#include "ConstraintGraph.h"
#include "RigidBody.h"

namespace CibraryEngine
{
	using namespace std;

	using boost::unordered_map;




	/*
	 * Recycle bins for ConstraintGraph::Node pointers
	 */
	static vector<ConstraintGraph::Node*> node_recycle_bin;
	static vector<vector<ConstraintGraph::Edge>*> edges_recycle_bin;

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
	static void DeleteNode(ConstraintGraph::Node* node) { node->~Node(); node_recycle_bin.push_back(node); }

	static vector<ConstraintGraph::Edge>* NewEdgesVector()
	{
		if(edges_recycle_bin.empty())
			return new vector<ConstraintGraph::Edge>();
		else
		{
			vector<ConstraintGraph::Edge>* result = *edges_recycle_bin.rbegin();
			edges_recycle_bin.pop_back();

			result->clear();
			return result;
		}
	}
	static void DeleteEdgesVector(vector<ConstraintGraph::Edge>* edges) { edges_recycle_bin.push_back(edges); }



	/*
	 * ConstraintGraph::Node methods
	 */
	ConstraintGraph::Node::Node(RigidBody* body) : body(body) { edges = NewEdgesVector(); }

	ConstraintGraph::Node::~Node() { if(edges) { DeleteEdgesVector(edges); edges = NULL; } }




	/*
	 * ConstraintGraph methods
	 */
	ConstraintGraph::ConstraintGraph() : nodes(), constraints(), contact_points() { constraints.reserve(1000); contact_points.reserve(1000); }
	ConstraintGraph::~ConstraintGraph()
	{
		for(unordered_map<RigidBody*, Node*>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
			DeleteNode(iter->second);
		nodes.clear();

		contact_points.clear();
		constraints.clear();
	}

	void ConstraintGraph::AddContactPoint(ContactPoint* cp) { contact_points.push_back(cp); }

	void ConstraintGraph::AddConstraint(PhysicsConstraint* constraint)
	{
		RigidBody* body_a = constraint->obj_a;
		if(body_a->MergesSubgraphs())
		{
			constraints.push_back(constraint);

			Node* node_a;
			unordered_map<RigidBody*, Node*>::iterator found_a = nodes.find(body_a);
			if(found_a != nodes.end())
				node_a = found_a->second;
			else
			{
				node_a = NewNode(body_a);
				nodes.insert(pair<RigidBody*, Node*>(body_a, node_a));
			}

			RigidBody* body_b = constraint->obj_b;
			if(body_b && body_b->MergesSubgraphs())
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

				node_a->edges->push_back(Edge(constraint, node_b));
				node_b->edges->push_back(Edge(constraint, node_a));
			}
			else
				node_a->edges->push_back(Edge(constraint, NULL));
		}
		else
		{
			// TODO: output to the effect of "excuse me wtf r u doin"
		}
	}

	void ConstraintGraph::EmptyRecycleBins()
	{
		for(vector<Node*>::iterator iter = node_recycle_bin.begin(); iter != node_recycle_bin.end(); ++iter)
			delete *iter;
		node_recycle_bin.clear();

		for(vector<vector<Edge>*>::iterator iter = edges_recycle_bin.begin(); iter != edges_recycle_bin.end(); ++iter)
			delete *iter;
		edges_recycle_bin.clear();
	}
}