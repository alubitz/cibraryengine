#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	class DATKeyframe
	{
		public:

			unsigned int num_joints;
			Vec3* joint_ori_data;

			Vec3 root_ori;
			Vec3 root_pos;

			DATKeyframe(unsigned int num_joints);
			DATKeyframe(const DATKeyframe& other);

			void operator =(const DATKeyframe& other);

			~DATKeyframe();
	};
}
