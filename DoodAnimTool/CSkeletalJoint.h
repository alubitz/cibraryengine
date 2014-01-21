#pragma once

#include "StdAfx.h"

#include "Constraint.h"
#include "DATKeyframe.h"

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

			DATKeyframe::KBone* initial_a;
			DATKeyframe::KBone* initial_b;

			DATKeyframe::KBone* obja;
			DATKeyframe::KBone* objb;

			DATKeyframe::KBone* nexta;
			DATKeyframe::KBone* nextb;

			Vec3 joint_pos;
			Vec3 lpivot_a, lpivot_b;

			Quaternion relative_ori;

			float ComputeDx(const Vec3& apos, const Vec3& bpos, const Quaternion& aori, const Quaternion& bori, Vec3& aend, Vec3& bend, Vec3& dx) const;

		public:

			bool enforce_rotation_limits;

			CSkeletalJoint(const ModelPhysics* mphys, unsigned int joint_index, const vector<DATBone>& bones);
			~CSkeletalJoint();

			void InitCachedStuff(PoseSolverState& pose);
			bool ApplyConstraint(PoseSolverState& pose);
			void OnAnyChanges   (PoseSolverState& pose);

			float GetErrorAmount(const DATKeyframe& pose);
	};
}
