#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	struct IKChain
	{
		struct Node
		{
			JointConstraint* jc;
			RigidBody *cur_rb, *next_rb;

			Bone *cur_bone, *next_bone;

			bool parent_next;					// true if the parent bone is the next bone in the chain
			bool a_next;						// true if obj_a in the joint constraint is the next rigid body in the chain

			Node(Bone* parent, Bone* child, bool parent_next);
		};

		vector<Node> chain;

		IKChain(Bone* root, Bone* end);
	};
}
