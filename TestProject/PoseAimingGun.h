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

			Quaternion torso2_ori;
			float yaw, pitch;

			PoseAimingGun();
			~PoseAimingGun();

			void UpdatePose(const TimingInfo& time);
	};
}
