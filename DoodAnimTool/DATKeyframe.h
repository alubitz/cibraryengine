#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	class DATKeyframe
	{
		public:

			unsigned int num_joints;
			Vec3* ori_data;

			DATKeyframe(unsigned int num_joints);
			DATKeyframe(const DATKeyframe& other);

			void operator =(const DATKeyframe& other);

			~DATKeyframe();
	};
}
