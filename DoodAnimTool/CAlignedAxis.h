#pragma once

#include "StdAfx.h"

#include "Constraint.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	// constrains that a unit vector in one bone's orientation must match another unit vector in another bone's orientation
	class CAlignedAxis : public Constraint
	{
		public:

			unsigned int bone_a, bone_b;
			Vec3 axis_a, axis_b;

			CAlignedAxis(unsigned int bone_a, unsigned int bone_b, const Vec3& axis_a, const Vec3& axis_b);

			float GetErrorAmount(const DATKeyframe& pose);
	};
}
