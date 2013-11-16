#include "StdAfx.h"
#include "CSkeletalJoint.h"

#include "DATJoint.h"
#include "PoseSolverState.h"

namespace DoodAnimTool
{
	/*
	 * CSkeletalJoint methods
	 */
	CSkeletalJoint::CSkeletalJoint(const DATJoint& joint) : joint(joint.joint), enforce_rotation_limits(true) { }

	CSkeletalJoint::~CSkeletalJoint() { }



	void CSkeletalJoint::InitCachedStuff(PoseSolverState& pose)
	{
		int bone_a = joint->bone_a - 1, bone_b = joint->bone_b - 1;
		
		initial_a = &pose.initial.data[bone_a];
		initial_b = &pose.initial.data[bone_b];

		obja = &pose.current.data[bone_a];
		objb = &pose.current.data[bone_b];

		nexta = &pose.next.data[bone_a];
		nextb = &pose.next.data[bone_b];

		joint_pos = joint->pos;

		// TODO: initialize other stuff here?
	}

	bool CSkeletalJoint::ApplyConstraint(PoseSolverState& pose)
	{
		bool did_stuff = false;

		Quaternion a_ori = obja->ori, b_ori = objb->ori;

		// bones staying in their sockets
		Vec3 apos = Mat4::FromPositionAndOrientation(obja->pos, Quaternion::Reverse(a_ori)).TransformVec3_1(joint_pos);
		Vec3 bpos = Mat4::FromPositionAndOrientation(objb->pos, Quaternion::Reverse(b_ori)).TransformVec3_1(joint_pos);

		Vec3 dx = bpos - apos;
		if(dx.ComputeMagnitudeSquared() > 0)
		{
			dx *= 0.1f;

			nexta->pos += dx;
			nextb->pos -= dx;

			// TODO: make this also affect bone orientations somehow?

			did_stuff = true;
		}

		// joint staying within its rotation limits
		if(enforce_rotation_limits)
		{
			Quaternion a_to_b = Quaternion::Reverse(a_ori) * b_ori;
			Mat3 oriented_axes = joint->axes * a_ori.ToMat3();

			Vec3 proposed_pyr = oriented_axes * -a_to_b.ToPYR();
			Vec3 nupyr = proposed_pyr;
			joint->ClampAngles(nupyr);

			if((proposed_pyr - nupyr).ComputeMagnitudeSquared() > 0)
			{
				Quaternion actual_ori = Quaternion::FromPYR(oriented_axes.Transpose() * -nupyr);
				Vec3 av = (Quaternion::Reverse(a_to_b) * actual_ori).ToPYR();
				av *= 0.1f;

				Quaternion delta_quat = Quaternion::FromPYR(av);
				nexta->ori = Quaternion::Reverse(delta_quat) * nexta->ori;
				nextb->ori = delta_quat * nextb->ori;

				did_stuff = true;
			}
		}
		
		return did_stuff;
	}

	void CSkeletalJoint::OnAnyChanges(PoseSolverState& pose)
	{
		int bone_a = joint->bone_a - 1, bone_b = joint->bone_b - 1;

		obja = &pose.current.data[bone_a];
		objb = &pose.current.data[bone_b];
	}
}
