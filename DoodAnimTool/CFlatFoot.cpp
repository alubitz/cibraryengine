#include "StdAfx.h"
#include "CFlatFoot.h"

#include "DATKeyframe.h"

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

	float CFlatFoot::GetErrorAmount(const DATKeyframe& pose)
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

	bool CFlatFoot::SetLockedBones(DATKeyframe& pose, bool* locked_bones)
	{
		if(!locked_bones[bone_a])
		{
			DATKeyframe::KBone&       a = pose.data[bone_a];
			const DATKeyframe::KBone& b = pose.data[bone_b];

			Vec3&             apos = a.pos;
			Quaternion&       aori = a.ori;

			const Vec3&       bpos = b.pos;
			const Quaternion& bori = b.ori;

			aori = bori * relative_ori;

			Vec3 aend = aori * socket_a + apos;
			Vec3 bend = bori * socket_b + bpos;

			apos += bend - aend;

			locked_bones[bone_a] = true;
		}

		return true;
	}
}
