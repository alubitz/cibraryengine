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
		static const float rotation_threshold            = 0.0f;
		static const float translation_threshold         = 0.0f;

		static const float linear_offset_linear_coeff    = 1.0f;

		bool did_stuff = false;

		Quaternion a_ori = obja->ori, b_ori = objb->ori;

		Vec3 nextpos_a = obja->pos, nextpos_b = objb->pos;
		Quaternion nextori_a = a_ori, nextori_b = b_ori;

		// keep the relative orientations of the two bones constant
		Vec3 av = (Quaternion::Reverse(b_ori) * a_ori * relative_ori).ToPYR();
		if(float err = av.ComputeMagnitudeSquared())
		{
			pose.errors[3] += err;

			if(err > rotation_threshold)
			{				
				Quaternion bprime = nextori_a * relative_ori;
				Quaternion aprime = nextori_b * Quaternion::Reverse(relative_ori);

				nextori_a = aprime;
				nextori_b = bprime;

				did_stuff = true;
			}
		}

		// keep the corresponding points in each bone in the same position
		Vec3 apos = Mat4::FromPositionAndOrientation(nextpos_a, Quaternion::Reverse(nextori_a)).TransformVec3_1(point_in_a);
		Vec3 bpos = Mat4::FromPositionAndOrientation(nextpos_b, Quaternion::Reverse(nextori_b)).TransformVec3_1(point_in_b);

		Vec3 dx = bpos - apos;
		if(float err = dx.ComputeMagnitudeSquared())
		{
			pose.errors[4] += err;

			if(err > translation_threshold)
			{
				dx *= 0.5f * linear_offset_linear_coeff;

				nextpos_a += dx;
				nextpos_b -= dx;

				did_stuff = true;
			}
		}

		// if we made any changes, let the solver know about it
		if(did_stuff)
		{
			nexta->pos += nextpos_a;
			nexta->ori += nextori_a;
			nextb->pos += nextpos_b;
			nextb->ori += nextori_b;

			++pose.contrib_count[bone_a];
			++pose.contrib_count[bone_b];

			return true;
		}
		else
			return false;
	}

	void CFixedJoint::OnAnyChanges(PoseSolverState& pose)
	{
		obja = &pose.current.data[bone_a];
		objb = &pose.current.data[bone_b];
	}
}
