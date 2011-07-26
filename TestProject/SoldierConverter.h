#pragma once

#include "../CibraryEngine/CibraryEngine.h"
#include "../CibraryEngine/UnclampedVertexBoneWeightInfo.h"

namespace Test
{
	using namespace CibraryEngine;

	void ConvertSoldier(ContentMan* content);
	void FixSoldierBones(Skeleton* skeleton);
}
