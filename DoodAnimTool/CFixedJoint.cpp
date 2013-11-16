#include "StdAfx.h"
#include "CFixedJoint.h"

#include "PoseSolverState.h"

namespace DoodAnimTool
{
	/*
	 * CFixedJoint methods
	 */
	CFixedJoint::CFixedJoint(unsigned int bone_a, unsigned int bone_b, const Vec3& relative_pos, const Quaternion& relative_ori) :
		bone_a(bone_a),
		bone_b(bone_b),
		relative_pos(relative_pos),
		relative_ori(relative_ori)
	{
	}

	CFixedJoint::~CFixedJoint() { }

	void CFixedJoint::InitCachedStuff(PoseSolverState& pose)
	{
		initial_a = &pose.initial.data[bone_a];
		initial_b = &pose.initial.data[bone_b];

		obja = &pose.current.data[bone_a];
		objb = &pose.current.data[bone_b];

		nexta = &pose.next.data[bone_a];
		nextb = &pose.next.data[bone_b];
	}

	bool CFixedJoint::ApplyConstraint(PoseSolverState& pose)
	{
		// TODO: implement this
		return false;
	}

	void CFixedJoint::OnAnyChanges(PoseSolverState& pose)
	{
		obja = &pose.current.data[bone_a];
		objb = &pose.current.data[bone_b];
	}
}
