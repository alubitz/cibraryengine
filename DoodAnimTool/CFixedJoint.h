#pragma once

#include "StdAfx.h"

#include "Constraint.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	class CFixedJoint : public Constraint
	{
		public:

			unsigned int bone_a, bone_b;

			Vec3 socket_a, socket_b;
			Quaternion relative_ori;

			CFixedJoint(unsigned int bone_a, unsigned int bone_b, const Vec3& socket_a, const Vec3& socket_b, const Quaternion& relative_ori);

			float GetErrorAmount(const DATKeyframe& pose);
	};
}
