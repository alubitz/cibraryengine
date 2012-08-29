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

			float anim_timer;
			Dood* dood;
			KeyframeAnimation* forward_anim;
			KeyframeAnimation* backward_anim;
			KeyframeAnimation* left_anim;
			KeyframeAnimation* right_anim;
			KeyframeAnimation* up_anim;
			KeyframeAnimation* down_anim;
			KeyframeAnimation* rest_anim;

			WalkPose(Dood* dood, const KeyframeAnimation* forward_anim, const KeyframeAnimation* backward_anim, const KeyframeAnimation* left_anim, const KeyframeAnimation* right_anim, const KeyframeAnimation* up_anim, const KeyframeAnimation* down_anim, const KeyframeAnimation* rest_anim);
			~WalkPose();

			void UpdatePose(TimingInfo time);
	};
}