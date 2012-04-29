#pragma once

#include "StdAfx.h"
#include "Physics.h"

namespace CibraryEngine
{
	using namespace std;

	class CollisionGraphSolver
	{
		public:
			virtual void Solve(const ContactPoint& cp) = 0;
	};

	struct CollisionGraph
	{
		struct Node;

		struct Edge
		{
			ContactPoint* cp;
			ContactPoint::Part* self;
			ContactPoint::Part* other;

			Node* other_node;

			Edge() : cp(NULL), self(NULL), other(NULL), other_node(NULL) { }			// just so vector<Edge> doesn't barf all over the place
			Edge(ContactPoint* cp, ContactPoint::Part* self, ContactPoint::Part* other, Node* other_node) : cp(cp), self(self), other(other), other_node(other_node) { }
		};

		struct Node
		{
			RigidBody* body;
			vector<Edge> edges;

			Node(RigidBody* body) : body(body), edges() { }
		};

		map<RigidBody*, Node*> nodes;
		vector<ContactPoint*> contact_points;

		CollisionGraph();
		~CollisionGraph();

		/**
		 * Call this whenever you find a collision
		 * The first object has to be something that merges subgraphs
		 */
		void AddContactPoint(const ContactPoint& cp);

		/** Call this after all collisions have been found and added */
		void Solve(CollisionGraphSolver* solver);
	};

}