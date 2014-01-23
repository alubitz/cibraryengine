#include "StdAfx.h"
#include "CFixedJoint.h"

#include "DATKeyframe.h"

namespace DoodAnimTool
{
	/*
	 * CFixedJoint methods
	 */
	CFixedJoint::CFixedJoint(unsigned int bone_a, unsigned int bone_b, const Vec3& socket_a, const Vec3& socket_b, const Quaternion& relative_ori) :
		bone_a(bone_a),
		bone_b(bone_b),
		socket_a(socket_a),
		socket_b(socket_b),
		relative_ori(relative_ori)
	{
	}

	float CFixedJoint::GetErrorAmount(const DATKeyframe& pose)
	{
		const Vec3&       apos = pose.data[bone_a].pos;
		const Vec3&       bpos = pose.data[bone_b].pos;
		const Quaternion& aori = pose.data[bone_a].ori;
		const Quaternion& bori = pose.data[bone_b].ori;

		float err = (Quaternion::Reverse(bori) * aori * relative_ori).GetRotationAngle();
		err *= err;

		Vec3 aend = aori * socket_a + apos;
		Vec3 bend = bori * socket_b + bpos;

		err += (bend - aend).ComputeMagnitudeSquared();
		
		return err;
	}
}
