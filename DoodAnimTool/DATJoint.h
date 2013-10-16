#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	struct DATJoint
	{
		int child_index;

		ModelPhysics::JointPhysics* joint;
		bool child_reversed;

		DATJoint() : child_index(-1), joint(NULL), child_reversed(false) { }
	};
}
