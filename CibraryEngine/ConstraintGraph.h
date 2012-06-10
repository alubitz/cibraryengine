#pragma once

#include "StdAfx.h"
#include "Physics.h"

namespace CibraryEngine
{
	using namespace std;

	struct ConstraintGraph
	{
		struct Node;

		struct Edge
		{
			PhysicsConstraint* constraint;

			Node* other_node;

			Edge() : constraint(NULL), other_node(NULL) { }			// just so vector<Edge> doesn't barf all over the place
			Edge(PhysicsConstraint* constraint, Node* other_node) : constraint(constraint), other_node(other_node) { }
		};

		struct Node
		{
			RigidBody* body;
			vector<Edge> edges;

			Node(RigidBody* body) : body(body), edges() { }
		};

		boost::unordered_map<RigidBody*, Node*> nodes;
		vector<PhysicsConstraint*> constraints;

		vector<ContactPoint*> contact_points;

		ConstraintGraph();
		~ConstraintGraph();

		/**
		 * Call this whenever you find a collision
		 * The first object has to be something that merges subgraphs
		 */
		void AddContactPoint(const ContactPoint& cp);

		void AddConstraint(PhysicsConstraint* constraint);

		static void EmptyRecycleBins();
	};

}