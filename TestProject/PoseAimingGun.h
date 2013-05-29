#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class PoseAimingGun : public Pose
	{

	public:

		float yaw, pitch;

		unsigned int torso1, torso2, head, rarm1, rarm2, rhand;

		PoseAimingGun() :
			Pose(),
			yaw(),
			pitch(),
			torso1(	Bone::string_table["torso 1"]	),
			torso2(	Bone::string_table["torso 2"]	),
			head(	Bone::string_table["head"]		),
			rarm1(	Bone::string_table["r arm 1"]	),
			rarm2(	Bone::string_table["r arm 2"]	),
			rhand(	Bone::string_table["r hand"]	)
		{
		}

		void UpdatePose(TimingInfo time)
		{
			if(time.total < 0.1f)
				return;

			SetBonePose(torso1,	Vec3(	pitch * 0.4f,			-yaw * 0.5f,	0), Vec3());
			SetBonePose(torso2,	Vec3(	pitch * 0.4f,			-yaw * 0.5f,	0), Vec3());
			SetBonePose(head,	Vec3(	pitch * 0.2f,			0,				0), Vec3());

			SetBonePose(rarm1,	Vec3(	pitch * 0.2f - 1.0f,	0.7f,			0), Vec3());
			SetBonePose(rarm2,	Vec3(	-0.25f,					0.25f,			0), Vec3());
			SetBonePose(rhand,	Vec3(	-0.35f,					-0.56f,			0), Vec3());
		}
	};
}
