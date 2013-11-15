#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	class DATKeyframe
	{
		public:

			struct KBone
			{
				Vec3 pos;
				Quaternion ori;

				KBone() : pos(), ori(Quaternion::Identity()) { }
			};

			unsigned int num_bones;
			KBone* data;

			unsigned int num_constraints;
			bool* enabled_constraints;

			DATKeyframe(unsigned int num_bones, unsigned int num_constraints);
			DATKeyframe(const DATKeyframe& other);

			void operator =(const DATKeyframe& other);

			~DATKeyframe();
	};
}
