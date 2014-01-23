#pragma once

#include "StdAfx.h"

#include "Constraint.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	// similar to CFixedJoint, except that obja does all the moving
	class CFlatFoot : public Constraint
	{
		public:

			unsigned int bone_a, bone_b;

			Vec3 socket_a, socket_b;
			Quaternion relative_ori;

			CFlatFoot(unsigned int bone_a, unsigned int bone_b, const Vec3& socket_a, const Vec3& socket_b, const Quaternion& relative_ori);

			float GetErrorAmount(const DATKeyframe& pose);
			bool  SetLockedBones(DATKeyframe& pose, bool* locked_bones);
	};
}
