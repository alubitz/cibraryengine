#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class Dood;

	class WalkPose : public Pose
	{
		public:

			Dood* dood;

			float anim_timer;
			float forward_v, leftward_v, yaw_v;

			KeyframeAnimation* rest_anim;
			KeyframeAnimation* forward_anim;
			KeyframeAnimation* backward_anim;
			KeyframeAnimation* left_anim;
			KeyframeAnimation* right_anim;
			KeyframeAnimation* l_turn_anim;
			KeyframeAnimation* r_turn_anim;

			float fwd_anim_rate, side_anim_rate, turn_anim_rate;

			unsigned int yaw_bone;			// id of bone to rotate to achieve yaw
			float yaw;
			float target_yaw;

			WalkPose(Dood* dood, const KeyframeAnimation* rest_anim, const KeyframeAnimation* forward_anim, const KeyframeAnimation* backward_anim, const KeyframeAnimation* left_anim, const KeyframeAnimation* right_anim, const KeyframeAnimation* l_turn_anim, const KeyframeAnimation* r_turn_anim);
			~WalkPose();

			void UpdatePose(const TimingInfo& time);
	};
}