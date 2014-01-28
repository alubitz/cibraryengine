#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	struct FixedJointPoseOp
	{
		unsigned int from, to;
		Mat4 xform;

		FixedJointPoseOp() { }
		FixedJointPoseOp(unsigned int from, unsigned int to, const Mat4& xform) : from(from), to(to), xform(xform) { }
	};
}
