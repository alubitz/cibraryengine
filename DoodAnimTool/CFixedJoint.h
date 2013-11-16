#pragma once

#include "StdAfx.h"

#include "Constraint.h"
#include "DATKeyframe.h"

namespace DoodAnimTool
{
	class CFixedJoint : public Constraint
	{
		private:

			DATKeyframe::KBone* initial_a;
			DATKeyframe::KBone* initial_b;

			DATKeyframe::KBone* obja;
			DATKeyframe::KBone* objb;

			DATKeyframe::KBone* nexta;
			DATKeyframe::KBone* nextb;

		public:

			unsigned int bone_a, bone_b;

			Vec3 relative_pos;
			Quaternion relative_ori;

			CFixedJoint(unsigned int bone_a, unsigned int bone_b, const Vec3& relative_pos, const Quaternion& relative_ori);
			~CFixedJoint();

			void InitCachedStuff(PoseSolverState& pose);
			bool ApplyConstraint(PoseSolverState& pose);
			void OnAnyChanges(PoseSolverState& pose);
	};
}
