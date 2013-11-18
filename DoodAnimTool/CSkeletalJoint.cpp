#include "StdAfx.h"
#include "CSkeletalJoint.h"

#include "DATJoint.h"
#include "DATBone.h"
#include "PoseSolverState.h"

namespace DoodAnimTool
{
	/*
	 * CSkeletalJoint methods
	 */
	CSkeletalJoint::CSkeletalJoint(const DATJoint& joint, const vector<DATBone>& bones) :
		bone_a(joint.joint->bone_a - 1),
		bone_b(joint.joint->bone_b - 1),
		joint_pos(joint.joint->pos),
		lcenter_a(bones[bone_a].center),
		lcenter_b(bones[bone_b].center),
		joint(joint.joint),
		enforce_rotation_limits(true)
	{ }

	CSkeletalJoint::~CSkeletalJoint() { }



	void CSkeletalJoint::InitCachedStuff(PoseSolverState& pose)
	{		
		initial_a = &pose.initial.data[bone_a];
		initial_b = &pose.initial.data[bone_b];

		obja = &pose.current.data[bone_a];
		objb = &pose.current.data[bone_b];

		nexta = &pose.next.data[bone_a];
		nextb = &pose.next.data[bone_b];
	}

	bool CSkeletalJoint::ApplyConstraint(PoseSolverState& pose)
	{
		bool did_stuff = false;

		Quaternion a_ori = obja->ori, b_ori = objb->ori;
		Quaternion aori_inv = Quaternion::Reverse(a_ori), bori_inv = Quaternion::Reverse(b_ori);

		// bones staying in their sockets
		Mat4 amat = Mat4::FromPositionAndOrientation(obja->pos, aori_inv);
		Mat4 bmat = Mat4::FromPositionAndOrientation(objb->pos, bori_inv);
		Vec3 apos = amat.TransformVec3_1(joint_pos);
		Vec3 bpos = bmat.TransformVec3_1(joint_pos);

		Vec3 dx = bpos - apos;
		if(dx.ComputeMagnitudeSquared() > 0)
		{
			dx *= 0.25f;

			nexta->pos += dx;
			nextb->pos -= dx;

			// also affect bone orientations
			static const float rotation_coeff = -0.25f;

			Vec3 ax = Vec3::Cross(dx, amat.TransformVec3_1(lcenter_a) - apos);
			Vec3 bx = Vec3::Cross(dx, bmat.TransformVec3_1(lcenter_b) - bpos);
			nexta->ori = Quaternion::FromPYR(ax * rotation_coeff) * nexta->ori;
			nextb->ori = Quaternion::FromPYR(bx * rotation_coeff) * nextb->ori;

			did_stuff = true;
		}

		// joint staying within its rotation limits
		if(enforce_rotation_limits)
		{
			Quaternion a_to_b = aori_inv * b_ori;
			Mat3 oriented_axes = joint->axes * a_ori.ToMat3();

			Vec3 proposed_pyr = oriented_axes * -a_to_b.ToPYR();
			Vec3 nupyr = proposed_pyr;
			joint->ClampAngles(nupyr);

			if((proposed_pyr - nupyr).ComputeMagnitudeSquared() > 0)
			{
				Quaternion actual_ori = Quaternion::FromPYR(oriented_axes.Transpose() * -nupyr);
				Vec3 av = (Quaternion::Reverse(a_to_b) * actual_ori).ToPYR();
				av *= 0.25f;

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
