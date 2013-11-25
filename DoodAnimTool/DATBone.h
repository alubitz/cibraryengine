#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	struct DATBone
	{
		unsigned int bone_index;			// index into a skeleton
		unsigned int name;

		CollisionShape* shape;
		Vec3 center;

		bool selected;

		DATBone(unsigned int bone_index, unsigned int name, CollisionShape* shape);
	};
}
