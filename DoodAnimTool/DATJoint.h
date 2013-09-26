#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	struct DATJoint
	{
		Bone* child_bone;

		ModelPhysics::JointPhysics* joint;
		bool child_reversed;

		DATJoint() : child_bone(NULL), joint(NULL), child_reversed(false) { }
	};
}
