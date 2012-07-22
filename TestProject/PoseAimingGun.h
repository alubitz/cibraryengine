#pragma once

#include "StdAfx.h"

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
			SetBonePose(Bone::string_table["torso 1"],		Vec3(0,	-yaw * 0.5f, 0),	Vec3());
			SetBonePose(Bone::string_table["torso 2"],		Vec3(0,	-yaw * 0.5f, 0),	Vec3());

			return;

			// whichever it has, we use...
			SetBonePose(Bone::string_table["pelvis"],		Vec3(0, -yaw, 0),			Vec3());

			SetBonePose(Bone::string_table["torso 1"],		Vec3(pitch * 0.4f,	0, 0),	Vec3());
			SetBonePose(Bone::string_table["torso 2"],		Vec3(pitch * 0.4f,	0, 0),	Vec3());
			SetBonePose(Bone::string_table["head"],			Vec3(pitch * 0.2f,	0, 0),	Vec3());

			SetBonePose(Bone::string_table["r shoulder"],	Vec3(	-0.75f,	0.7f,	0.0f),	Vec3());
			SetBonePose(Bone::string_table["r arm 1"],		Vec3(	-0.25f,	0.0f,	0.0f),	Vec3());
			SetBonePose(Bone::string_table["r arm 2"],		Vec3(	-0.25f,	0.25f,	0.0f),	Vec3());
			SetBonePose(Bone::string_table["r hand"],		Vec3(	-0.35f,	-0.56f,	0.0f),	Vec3());
		}
	};
}
