#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	class DATKeyframe;

	class JointOrientations
	{
		private:

			void CopyDataDirect(const JointOrientations& other);

		public:

			unsigned int num_joints;
			Vec3* data;

			JointOrientations(unsigned int num_joints);
			JointOrientations(const JointOrientations& other);

			~JointOrientations();

			void operator =(const JointOrientations& other);

			void PoseBones(DATKeyframe& pose, const ModelPhysics* mphys, unsigned int start_where = 0) const;
	};
}
