#pragma once

#include "StdAfx.h"
#include "SkeletalAnimation.h"

namespace CibraryEngine
{
	class GameState;
	class KeyframeAnimation;

	class IKPose : public Pose
	{
		protected:

			struct EndEffector
			{
				string bone_name;
				Vec3 lcs_pos;
				bool set;

				bool grounded;

				EndEffector(string bone_name, Vec3 lcs_pos, bool set);
			};

			vector<EndEffector> end_effectors;

		public:

			// these are outputs
			Vec3 pos;
			float pitch, yaw;

			GameState* game_state;
			Skeleton* ik_skeleton;
			KeyframeAnimation* keyframe_animation;

			IKPose(GameState* game_state, Skeleton* skeleton, Vec3 pos, float pitch, float yaw);
			~IKPose();

			void UpdatePose(TimingInfo time);
			void SetDesiredState(Vec3 pos, float pitch, float yaw);

			void AddEndEffector(string bone_name, Vec3 lcs_pos, bool set);
	};
}