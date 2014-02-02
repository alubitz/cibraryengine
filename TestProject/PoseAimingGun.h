#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class PoseAimingGun : public Pose
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			Quaternion pelvis_ori;
			Vec3 aim_dir;

			PoseAimingGun();
			~PoseAimingGun();

			void UpdatePose(const TimingInfo& time);
	};
}
