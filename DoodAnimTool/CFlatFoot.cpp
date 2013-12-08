#include "StdAfx.h"
#include "CFlatFoot.h"

#include "PoseSolverState.h"

namespace DoodAnimTool
{
	/*
	 * CFlatFoot methods
	 */
	CFlatFoot::CFlatFoot(unsigned int bone_a, unsigned int bone_b, const Vec3& point_in_a, const Vec3& point_in_b, const Quaternion& relative_ori) :
		bone_a(bone_a),
		bone_b(bone_b),
		point_in_a(point_in_a),
		point_in_b(point_in_b),
		relative_ori(relative_ori)
	{
	}

	CFlatFoot::~CFlatFoot() { }

	void CFlatFoot::InitCachedStuff(PoseSolverState& pose)
	{
		initial_a = &pose.initial.data[bone_a];
		initial_b = &pose.initial.data[bone_b];

		obja = &pose.current.data[bone_a];
		objb = &pose.current.data[bone_b];

		nexta = &pose.next.data[bone_a];
		nextb = &pose.next.data[bone_b];
	}

	bool CFlatFoot::ApplyConstraint(PoseSolverState& pose)
	{
		static const float rotation_threshold            = 0.0f;
		static const float translation_threshold         = 0.0f;

		static const float linear_offset_linear_coeff    = 1.0f;
		static const float angular_offset_rotation_coeff = 1.0f;

		bool did_stuff = false;

		Quaternion a_ori = obja->ori, b_ori = objb->ori;
		Quaternion aori_inv = Quaternion::Reverse(a_ori), bori_inv = Quaternion::Reverse(b_ori);

		Vec3 nextpos_a = obja->pos;
		Quaternion nextori_a = a_ori;

		// keep the relative orientations of the two bones constant
		Vec3 av = (Quaternion::Reverse(aori_inv * b_ori) * relative_ori).ToPYR();
		if(float err = av.ComputeMagnitudeSquared())
		{
			pose.errors[5] += err;

			if(err > rotation_threshold)
			{
				av *= linear_offset_linear_coeff;

				Quaternion delta_quat = Quaternion::FromPYR(av);
				nextori_a = Quaternion::Reverse(delta_quat) * nextori_a;

				did_stuff = true;
			}
		}

		// keep the corresponding points in each bone in the same position
		Vec3 apos = Mat4::FromPositionAndOrientation(nextpos_a, Quaternion::Reverse(nextori_a)).TransformVec3_1(point_in_a);
		Vec3 bpos = Mat4::FromPositionAndOrientation(objb->pos, bori_inv).TransformVec3_1(point_in_b);

		Vec3 dx = bpos - apos;
		if(float err = dx.ComputeMagnitudeSquared())
		{
			pose.errors[6] += err;

			if(err > translation_threshold)
			{
				dx *= angular_offset_rotation_coeff;
				nextpos_a += dx;

				did_stuff = true;
			}
		}

		// if we made any changes, let the solver know about it
		if(did_stuff)
		{
			nexta->pos += nextpos_a;
			nexta->ori += nextori_a;

			++pose.contrib_count[bone_a];

			return true;
		}
		else
			return false;
	}

	void CFlatFoot::OnAnyChanges(PoseSolverState& pose)
	{
		obja = &pose.current.data[bone_a];
		objb = &pose.current.data[bone_b];
	}
}
