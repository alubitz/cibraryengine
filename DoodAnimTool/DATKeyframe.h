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
				KBone(const Vec3& pos, const Quaternion& ori) : pos(pos), ori(ori) { }
			};

			unsigned int num_bones;
			KBone* data;

			unsigned int num_constraints;
			bool* enabled_constraints;

			DATKeyframe(unsigned int num_bones, unsigned int num_constraints);
			DATKeyframe(const DATKeyframe& other);

			void operator =(const DATKeyframe& other);

			~DATKeyframe();




			unsigned int Read(istream& stream);
			void Write(ostream& stream) const;
	};
}
