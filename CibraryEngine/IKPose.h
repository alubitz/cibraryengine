#pragma once

#include "StdAfx.h"
#include "SkeletalAnimation.h"

namespace CibraryEngine
{
	class GameState;

	class IKPose : public Pose
	{

	public:
		// these are outputs
		Vec3 pos;
		float pitch, yaw;

		GameState* game_state;
		Skeleton* ik_skeleton;

		IKPose(GameState* game_state, Skeleton* skeleton, Vec3 pos, float pitch, float yaw);
		~IKPose();

		void UpdatePose(TimingInfo time);
		void SetDesiredState(Vec3 pos, float pitch, float yaw);
	};
}