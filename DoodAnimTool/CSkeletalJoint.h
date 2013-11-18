#pragma once

#include "StdAfx.h"

#include "Constraint.h"
#include "DATKeyframe.h"

namespace DoodAnimTool
{
	using namespace std;
	using namespace CibraryEngine;

	struct DATBone;
	struct DATJoint;

	class CSkeletalJoint : public Constraint
	{
		private:

			int bone_a, bone_b;

			DATKeyframe::KBone* initial_a;
			DATKeyframe::KBone* initial_b;

			DATKeyframe::KBone* obja;
			DATKeyframe::KBone* objb;

			DATKeyframe::KBone* nexta;
			DATKeyframe::KBone* nextb;

			Vec3 joint_pos;
			Vec3 lcenter_a, lcenter_b;

		public:

			ModelPhysics::JointPhysics* joint;
			bool enforce_rotation_limits;

			CSkeletalJoint(const DATJoint& joint, const vector<DATBone>& bones);
			~CSkeletalJoint();

			void InitCachedStuff(PoseSolverState& pose);
			bool ApplyConstraint(PoseSolverState& pose);
			void OnAnyChanges(PoseSolverState& pose);
	};
}
