#pragma once

#include "StdAfx.h"

#include "Constraint.h"
#include "DATKeyframe.h"

namespace DoodAnimTool
{
	using namespace std;
	using namespace CibraryEngine;

	struct DATJoint;

	class CSkeletalJoint : public Constraint
	{
		private:

			DATKeyframe::KBone* initial_a;
			DATKeyframe::KBone* initial_b;

			DATKeyframe::KBone* obja;
			DATKeyframe::KBone* objb;

			DATKeyframe::KBone* nexta;
			DATKeyframe::KBone* nextb;

			Vec3 joint_pos;

		public:

			ModelPhysics::JointPhysics* joint;
			bool enforce_rotation_limits;

			CSkeletalJoint(const DATJoint& joint);
			~CSkeletalJoint();

			void InitCachedStuff(PoseSolverState& pose);
			bool ApplyConstraint(PoseSolverState& pose);
			void OnAnyChanges(PoseSolverState& pose);
	};
}
