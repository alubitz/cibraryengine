#include "StdAfx.h"
#include "CSkeletalJoint.h"

#include "DATBone.h"
#include "PoseSolverState.h"

namespace DoodAnimTool
{
	/*
	 * CSkeletalJoint methods
	 */
	CSkeletalJoint::CSkeletalJoint(const ModelPhysics* mphys, unsigned int joint_index, const vector<DATBone>& bones) :
		joint(&mphys->joints[joint_index]),
		joint_index(joint_index),
		bone_a(joint->bone_a - 1),
		bone_b(joint->bone_b - 1),
		lpivot_a(),
		lpivot_b(),
		joint_pos(joint->pos),
		enforce_rotation_limits(false)
	{
		// find other joints with this bone and use the average of their positions as this bone's pivot point
		unsigned int ajoints = 0, bjoints = 0;

		unsigned int mya = joint->bone_a, myb = joint->bone_b;
		for(unsigned int i = 0; i < mphys->joints.size(); ++i)
			if(i != joint_index)
			{
				const ModelPhysics::JointPhysics& ijoint = mphys->joints[i];
				unsigned int ija = ijoint.bone_a, ijb = ijoint.bone_b;

				if(ija == mya || ijb == mya)
				{
					lpivot_a += mphys->joints[i].pos;
					++ajoints;
				}
				else if(ija == myb || ijb == myb)
				{
					lpivot_b += mphys->joints[i].pos;
					++bjoints;
				}
			}

		if(ajoints == 0) { lpivot_a = bones[bone_a].center; } else { lpivot_a /= float(ajoints); }
		if(bjoints == 0) { lpivot_b = bones[bone_b].center; } else { lpivot_b /= float(bjoints); }
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

		relative_ori = Quaternion::FromRVec(joint->axes.Transpose() * -pose.joints.data[joint_index]);
	}

	float CSkeletalJoint::ComputeDx(const Vec3& apos, const Vec3& bpos, const Quaternion& aori, const Quaternion& bori, Vec3& aend, Vec3& bend, Vec3& dx) const
	{
		aend = aori * joint_pos + apos;
		bend = bori * joint_pos + bpos;
		dx = bend - aend;

		return dx.ComputeMagnitudeSquared();
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

		
		// keep the relative orientations of the two bones constant
		Vec3 av = (Quaternion::Reverse(bori) * aori * relative_ori).ToRVec();
		if(float err = av.ComputeMagnitudeSquared())
		{
			pose.errors[0] += err;

			if(err > rotation_threshold)
			{				
				Quaternion bprime = aori * relative_ori;
				Quaternion aprime = bori * Quaternion::Reverse(relative_ori);

				aori = aprime;
				bori = bprime;

				did_stuff = true;
			}
		}

		// joint staying within its rotation limits
		if(enforce_rotation_limits)
		{
			Quaternion a_to_b = Quaternion::Reverse(aori) * bori;

			Vec3 proposed_rvec = joint->axes * -a_to_b.ToRVec();
			Vec3 nu_rvec       = joint->GetClampedAngles(proposed_rvec);

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
