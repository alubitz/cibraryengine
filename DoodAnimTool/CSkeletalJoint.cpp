#include "StdAfx.h"
#include "CSkeletalJoint.h"

#include "DATBone.h"
#include "DATKeyframe.h"

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
		joint_pos(joint->pos),
		enforce_rotation_limits(true)
	{
	}

	float CSkeletalJoint::GetErrorAmount(const DATKeyframe& pose)
	{
		const Vec3&       apos = pose.data[bone_a].pos;
		const Vec3&       bpos = pose.data[bone_b].pos;
		const Quaternion& aori = pose.data[bone_a].ori;
		const Quaternion& bori = pose.data[bone_b].ori;

		Vec3 aend = aori * joint_pos + apos;
		Vec3 bend = bori * joint_pos + bpos;
		float err = (bend - aend).ComputeMagnitudeSquared();

		if(enforce_rotation_limits)
		{
			Quaternion a_to_b  = Quaternion::Reverse(aori) * bori;
			Vec3 proposed_rvec = joint->axes * -a_to_b.ToRVec();
			Vec3 nu_rvec       = joint->GetClampedAngles(proposed_rvec);

			err += (proposed_rvec - nu_rvec).ComputeMagnitudeSquared();
		}

		return err;
	}
}
