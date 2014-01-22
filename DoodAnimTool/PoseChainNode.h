#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	struct PoseChainNode
	{
		unsigned int index;
		unsigned int from, to;
		Vec3 pos;
		Mat3 mat;

		PoseChainNode() { }
		PoseChainNode(unsigned int index, const Vec3& pos, unsigned int from, unsigned int to, const Mat3& mat) : index(index), from(from), to(to), pos(pos), mat(mat) { }
	};
}
