#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	struct DATBone
	{
		Bone* bone;
		CollisionShape* shape;

		int parent_index;

		bool selected;

		DATBone(Bone* bone, CollisionShape* shape, int parent_index);
	};
}
