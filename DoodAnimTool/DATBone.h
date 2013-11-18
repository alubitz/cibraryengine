#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	struct DATBone
	{
		Bone* bone;
		CollisionShape* shape;

		unsigned int bone_index;			// index into a skeleton
		int parent_index;

		bool selected;

		Vec3 center;

		DATBone(Bone* bone, CollisionShape* shape, unsigned int bone_index, int parent_index);
	};
}
