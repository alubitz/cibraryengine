#include "StdAfx.h"
#include "CFlatFoot.h"

#include "PoseSolverState.h"

namespace DoodAnimTool
{
	/*
	 * CFlatFoot methods
	 */
	CFlatFoot::CFlatFoot(unsigned int bone_a, unsigned int bone_b, const Vec3& socket_a, const Vec3& socket_b, const Quaternion& relative_ori) :
		bone_a(bone_a),
		bone_b(bone_b),
		socket_a(socket_a),
		socket_b(socket_b),
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

		bool did_stuff = false;

		Quaternion aori = obja->ori;
		Quaternion bori = objb->ori;
		Vec3       apos = obja->pos;
		Vec3       bpos = objb->pos;

		// keep the relative orientations of the two bones constant
		Vec3 av = (Quaternion::Reverse(bori) * aori * relative_ori).ToRVec();
		if(float err = av.ComputeMagnitudeSquared())
		{
			pose.errors[5] += err;

			if(err > rotation_threshold)
			{
				aori = bori * relative_ori;

				did_stuff = true;
			}
		}

		// keep the corresponding points in each bone in the same position
		Vec3 aend = aori * socket_a + apos;
		Vec3 bend = bori * socket_b + bpos;

		Vec3 dx = bend - aend;
		if(float err = dx.ComputeMagnitudeSquared())
		{
			pose.errors[6] += err;

			if(err > translation_threshold)
			{
				dx *= linear_offset_linear_coeff;
				apos += dx;

				did_stuff = true;
			}
		}

		// if we made any changes, let the solver know about it
		if(did_stuff)
		{
			nexta->pos += apos;
			nexta->ori += aori;

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

	float CFlatFoot::GetErrorAmount(const DATKeyframe& pose)
	{
		const Vec3&       apos = pose.data[bone_a].pos;
		const Vec3&       bpos = pose.data[bone_b].pos;
		const Quaternion& aori = pose.data[bone_a].ori;
		const Quaternion& bori = pose.data[bone_b].ori;

		float err = (Quaternion::Reverse(bori) * aori * relative_ori).ToRVec().ComputeMagnitudeSquared();

		Vec3 aend = aori * socket_a + apos;
		Vec3 bend = bori * socket_b + bpos;

		err += (bend - aend).ComputeMagnitudeSquared();

		return err;
	}
}
