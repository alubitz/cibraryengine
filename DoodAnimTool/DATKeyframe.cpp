#include "StdAfx.h"
#include "DATKeyframe.h"

namespace DoodAnimTool
{
	/*
	 * DATKeyframe methods
	 */
	DATKeyframe::DATKeyframe(unsigned int num_joints) : num_joints(num_joints), joint_ori_data(new Vec3[num_joints]), root_ori(), root_pos() { }

	DATKeyframe::DATKeyframe(const DATKeyframe& other) : num_joints(other.num_joints), joint_ori_data(new Vec3[num_joints]), root_ori(other.root_ori), root_pos(other.root_pos)
	{
		memcpy(joint_ori_data, other.joint_ori_data, num_joints * sizeof(Vec3));
	}

	void DATKeyframe::operator =(const DATKeyframe& other)
	{
		if(joint_ori_data) { delete[] joint_ori_data; }

		num_joints = other.num_joints;
		joint_ori_data = new Vec3[num_joints];
		root_ori = other.root_ori;
		root_pos = other.root_pos;

		memcpy(joint_ori_data, other.joint_ori_data, num_joints * sizeof(Vec3));
	}

	DATKeyframe::~DATKeyframe() { delete[] joint_ori_data; joint_ori_data = NULL; }
}
