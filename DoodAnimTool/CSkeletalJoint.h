#pragma once

#include "StdAfx.h"

#include "Constraint.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	struct DATBone;

	class CSkeletalJoint : public Constraint
	{
		public:

			const ModelPhysics::JointPhysics* joint;

		private:

			unsigned int joint_index;
			int bone_a, bone_b;

			Vec3 joint_pos;

		public:

			bool enforce_rotation_limits;

			CSkeletalJoint(const ModelPhysics* mphys, unsigned int joint_index, const vector<DATBone>& bones);

			float GetErrorAmount(const DATKeyframe& pose);
	};
}
