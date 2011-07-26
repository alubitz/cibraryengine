#pragma once

#include "../CibraryEngine/CibraryEngine.h"

namespace Test
{
	using namespace CibraryEngine;

	class PoseAimingGun : public Pose
	{

	public:

		float yaw, pitch;

		PoseAimingGun() : Pose(), yaw(), pitch() { }

		void UpdatePose(TimingInfo time)
		{
			SetBonePose("torso 2", Vec3(pitch * 0.4, 0, 0), Vec3(), 1.0);
			SetBonePose("torso 3", Vec3(pitch * 0.4, 0, 0), Vec3(), 1.0);

			SetBonePose("head", Vec3(pitch * 0.2, 0, 0), Vec3(), 1.0);

			SetBonePose("r shoulder", Vec3(-0.75, 0.7, 0.0), Vec3(), 1.0);
			SetBonePose("r arm 1", Vec3(-0.25, 0.0, 0.0), Vec3(), 1.0);
			SetBonePose("r arm 2", Vec3(-0.25, 0.25, 0.0), Vec3(), 1.0);
			SetBonePose("r hand", Vec3(-0.35, -0.56, 0.0), Vec3(), 1.0);

			/*
			SetBonePose("l shoulder", Vec3(-0.75, -0.7, 0.0), Vec3(), 1.0);
			SetBonePose("l arm 1", Vec3(-0.25, 0.0, -0.0), Vec3(), 1.0);
			SetBonePose("l arm 2", Vec3(-0.25, -0.25, 0.0), Vec3(), 1.0);
			SetBonePose("l hand", Vec3(-0.35, 0.56, 0.0), Vec3(), 1.0);
			*/
		}
	};
}
