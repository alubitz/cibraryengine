#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class PoseAimingGun : public Pose
	{

	public:

		float yaw, pitch;
		Vec3 pos;

		PoseAimingGun() : Pose(), pos(), yaw(), pitch() { }

		void UpdatePose(TimingInfo time)
		{
			// whichever it has, we use...
			SetBonePose("pelvis",		Vec3(0, -yaw, 0),			Vec3(),	1.0f);
			SetBonePose("carapace",		Vec3(0, -yaw, 0),			Vec3(),	1.0f);

			SetBonePose("torso 2",		Vec3(pitch * 0.4f, 0, 0),	Vec3(),	1.0f);
			SetBonePose("torso 3",		Vec3(pitch * 0.4f, 0, 0),	Vec3(),	1.0f);

			SetBonePose("head",			Vec3(pitch * 0.2f, 0, 0),	Vec3(), 1.0f);

			SetBonePose("r shoulder",	Vec3(	-0.75f,	0.7f,	0.0f),	Vec3(), 1.0f);
			SetBonePose("r arm 1",		Vec3(	-0.25f,	0.0f,	0.0f),	Vec3(), 1.0f);
			SetBonePose("r arm 2",		Vec3(	-0.25f,	0.25f,	0.0f),	Vec3(), 1.0f);
			SetBonePose("r hand",		Vec3(	-0.35f,	-0.56f,	0.0f),	Vec3(), 1.0f);
		}
	};
}
