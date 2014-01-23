#include "StdAfx.h"
#include "CAlignedAxis.h"

#include "DATKeyframe.h"

namespace DoodAnimTool
{
	/*
	 * CAlignedAxis methods
	 */
	CAlignedAxis::CAlignedAxis(unsigned int bone_a, unsigned int bone_b, const Vec3& axis_a, const Vec3& axis_b) :
		bone_a(bone_a),
		bone_b(bone_b),
		axis_a(axis_a),
		axis_b(axis_b)
	{
	}

	float CAlignedAxis::GetErrorAmount(const DATKeyframe& pose)
	{
		const Quaternion& aori = pose.data[bone_a].ori;
		const Quaternion& bori = pose.data[bone_b].ori;

		Vec3 avec = aori * axis_a;
		Vec3 bvec = bori * axis_b;

		return (avec - bvec).ComputeMagnitudeSquared();
	}
}
