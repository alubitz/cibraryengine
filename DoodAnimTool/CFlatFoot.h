#pragma once

#include "StdAfx.h"

#include "Constraint.h"
#include "DATKeyframe.h"

namespace DoodAnimTool
{
	// similar to CFixedJoint, except that obja does all the moving
	class CFlatFoot : public Constraint
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

			Vec3 socket_a, socket_b;
			Quaternion relative_ori;

			CFlatFoot(unsigned int bone_a, unsigned int bone_b, const Vec3& socket_a, const Vec3& socket_b, const Quaternion& relative_ori);
			~CFlatFoot();

			void InitCachedStuff(PoseSolverState& pose);
			bool ApplyConstraint(PoseSolverState& pose);
			void OnAnyChanges   (PoseSolverState& pose);

			float GetErrorAmount(const DATKeyframe& pose);
	};
}
