#include "StdAfx.h"
#include "CFixedJoint.h"

#include "PoseSolverState.h"

namespace DoodAnimTool
{
	/*
	 * CFixedJoint methods
	 */
	CFixedJoint::CFixedJoint(unsigned int bone_a, unsigned int bone_b, const Vec3& point_in_a, const Vec3& point_in_b, const Quaternion& relative_ori) :
		bone_a(bone_a),
		bone_b(bone_b),
		point_in_a(point_in_a),
		point_in_b(point_in_b),
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
		bool did_stuff = false;

		Quaternion a_ori = obja->ori, b_ori = objb->ori;
		Quaternion aori_inv = Quaternion::Reverse(a_ori), bori_inv = Quaternion::Reverse(b_ori);

		// keep the corresponding points in each bone in the same position
		Vec3 apos = Mat4::FromPositionAndOrientation(obja->pos, aori_inv).TransformVec3_1(point_in_a);
		Vec3 bpos = Mat4::FromPositionAndOrientation(objb->pos, bori_inv).TransformVec3_1(point_in_b);

		Vec3 dx = bpos - apos;
		if(dx.ComputeMagnitudeSquared() > 0)
		{
			dx *= 0.25f;

			nexta->pos += dx;
			nextb->pos -= dx;

			did_stuff = true;
		}

		// keep the relative orientations of the two bones constant
		Quaternion a_to_b = aori_inv * b_ori;

		Vec3 av = (Quaternion::Reverse(a_to_b) * relative_ori).ToPYR();
		if(av.ComputeMagnitudeSquared() > 0)
		{
			av *= 0.25f;

			Quaternion delta_quat = Quaternion::FromPYR(av);
			nexta->ori = Quaternion::Reverse(delta_quat) * nexta->ori;
			nextb->ori = delta_quat * nextb->ori;

			did_stuff = true;
		}

		return did_stuff;
	}

	void CFixedJoint::OnAnyChanges(PoseSolverState& pose)
	{
		obja = &pose.current.data[bone_a];
		objb = &pose.current.data[bone_b];
	}
}
