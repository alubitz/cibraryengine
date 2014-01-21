#include "StdAfx.h"
#include "CSkeletalJoint.h"

#include "DATBone.h"
#include "PoseSolverState.h"

namespace DoodAnimTool
{
	/*
	 * CSkeletalJoint methods
	 */
	CSkeletalJoint::CSkeletalJoint(const ModelPhysics::JointPhysics* joint, const vector<DATBone>& bones) :
		joint(joint),
		bone_a(joint->bone_a - 1),
		bone_b(joint->bone_b - 1),
		joint_pos(joint->pos),
		lcenter_a(bones[bone_a].center),
		lcenter_b(bones[bone_b].center),
		enforce_rotation_limits(true)
	{
	}

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

	float CSkeletalJoint::ComputeDx(const Vec3& apos, const Vec3& bpos, const Quaternion& aori, const Quaternion& bori, Vec3& aend, Vec3& bend, Vec3& dx) const
	{
		aend = aori * joint_pos + apos;
		bend = bori * joint_pos + bpos;
		dx = bend - aend;

		return dx.ComputeMagnitudeSquared();
	}

	void CSkeletalJoint::DoHalfRot(Vec3& rot, const Vec3& endpoint, const Vec3& bone_cen, const Vec3& meet) const
	{
		Vec3 cur = bone_cen - endpoint;
		Vec3 goal = bone_cen - meet;
		Vec3 axis = Vec3::Cross(cur, goal);
		if(float axissq = axis.ComputeMagnitudeSquared())
		{
			float axismag = sqrtf(axissq);
			float bonesq = cur.ComputeMagnitudeSquared();						// would "compute once and save" but can't because aend/bend issue (see TODO below)
			float goalsq = goal.ComputeMagnitudeSquared();
			float actual_angle = asinf(axismag / sqrtf(bonesq * goalsq));
			float length_factor = min(1.0f, max(0.1f, bonesq));					// the formula used here is somewhat arbitrary, but it seems to work most of the time
			float use_angle = length_factor * actual_angle;
			float coeff = use_angle / axismag;

			rot += axis * coeff;
		}
	}

	bool CSkeletalJoint::ApplyConstraint(PoseSolverState& pose)
	{
		static const float rotation_threshold           = 0.0f;
		static const float translation_threshold        = 0.0f;

		static const float linear_offset_rotation_coeff = 1.0f * 0.5f;
		static const float linear_offset_linear_coeff   = 1.0f * 0.5f;

		bool did_stuff = false;

		Vec3       apos = obja->pos;
		Vec3       bpos = objb->pos;
		Quaternion aori = obja->ori;
		Quaternion bori = objb->ori;

		Vec3 aend, bend, dx;

		// bones rotating to stay in their sockets
		if(float err = ComputeDx(apos, bpos, aori, bori, aend, bend, dx))
		{
			pose.errors[0] += err;

			if(err > rotation_threshold)
			{
				Vec3 acen = aori * lcenter_a + apos, bcen = bori * lcenter_b + bpos;
				Vec3 midpoint = (aend + bend) * 0.5f;
				
				Vec3 rot;

				DoHalfRot(rot, aend, acen, midpoint);
				DoHalfRot(rot, aend, bcen, midpoint);		// TODO: investigate why this works with aend but not bend (even though bend would make more sense)

				if(rot.ComputeMagnitudeSquared() > 0)
				{
					Quaternion delta_quat = Quaternion::FromRVec(rot * (linear_offset_rotation_coeff));

					aori = delta_quat * aori;
					bori = Quaternion::Reverse(delta_quat) * bori;

					did_stuff = true;
				}
			}
		}


		// joint staying within its rotation limits
		if(enforce_rotation_limits)
		{
			Quaternion a_to_b = Quaternion::Reverse(aori) * bori;

			Vec3 proposed_rvec = joint->axes * -a_to_b.ToRVec();
			Vec3 nu_rvec = joint->GetClampedAngles(proposed_rvec);

			if(float err = (proposed_rvec - nu_rvec).ComputeMagnitudeSquared())
			{
				pose.errors[1] += err;

				Quaternion actual_ori = Quaternion::FromRVec(joint->axes.Transpose() * -nu_rvec);

				Quaternion newb = aori * actual_ori;
				aori = bori * Quaternion::Reverse(actual_ori);
				bori = newb;

				did_stuff = true;
			}
		}
		

		// bones translating to stay in their sockets
		if(float err = ComputeDx(apos, bpos, aori, bori, aend, bend, dx))
		{
			pose.errors[2] += err;

			if(err > translation_threshold)
			{
				dx *= linear_offset_linear_coeff;

				apos += dx;
				bpos -= dx;

				did_stuff = true;
			}
		}


		// if we made any changes, let the solver know about it
		if(did_stuff)
		{
			nexta->pos += apos;
			nexta->ori += aori;
			nextb->pos += bpos;
			nextb->ori += bori;

			++pose.contrib_count[bone_a];
			++pose.contrib_count[bone_b];

			return true;
		}
		else
			return false;
	}

	void CSkeletalJoint::OnAnyChanges(PoseSolverState& pose)
	{
		obja = &pose.current.data[bone_a];
		objb = &pose.current.data[bone_b];
	}

	float CSkeletalJoint::GetErrorAmount(const DATKeyframe& pose)
	{
		const Vec3&       apos = pose.data[bone_a].pos;
		const Vec3&       bpos = pose.data[bone_b].pos;
		const Quaternion& aori = pose.data[bone_a].ori;
		const Quaternion& bori = pose.data[bone_b].ori;

		Vec3 aend, bend, dx;
		float err = ComputeDx(apos, bpos, aori, bori, aend, bend, dx);

		Quaternion a_to_b  = Quaternion::Reverse(aori) * bori;
		Vec3 proposed_rvec = joint->axes * -a_to_b.ToRVec();
		Vec3 nu_rvec       = joint->GetClampedAngles(proposed_rvec);

		err += (proposed_rvec - nu_rvec).ComputeMagnitudeSquared();

		return err;
	}
}
