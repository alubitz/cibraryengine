#include "StdAfx.h"
#include "DATKeyframe.h"

namespace DoodAnimTool
{
	/*
	 * DATKeyframe methods
	 */
	DATKeyframe::DATKeyframe(unsigned int num_joints) : num_joints(num_joints), ori_data(new Vec3[num_joints]) { }

	DATKeyframe::DATKeyframe(const DATKeyframe& other) : num_joints(other.num_joints), ori_data(new Vec3[num_joints])
	{
		memcpy(ori_data, other.ori_data, num_joints * sizeof(Vec3));
	}

	void DATKeyframe::operator =(const DATKeyframe& other)
	{
		if(ori_data) { delete[] ori_data; }

		num_joints = other.num_joints;
		ori_data = new Vec3[num_joints];
	}

	DATKeyframe::~DATKeyframe() { delete[] ori_data; ori_data = NULL; }
}
