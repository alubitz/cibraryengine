#pragma once
#include "StdAfx.h"

#include "../CibraryEngine/StepPose.h"
#include "../CibraryEngine/IKAnimation.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class Dood;

	class IKAnimationPose : public Pose
	{
		private:

			const IKAnimation* anim;				// animation prototype

			vector<StepPose*> chains;				// values are parallel to those of anim->chains
			vector<Bone*> bones;					// values are parallel to those of anim->bones

			int frame_index, next_frame;
			float frame_time;						// time since [most recent] arrival at this keyframe

			Dood* dood;								// owner of this pose

		public:

			bool dead;

			IKAnimationPose(const IKAnimation* anim, Dood* dood);
			~IKAnimationPose();

			void UpdatePose(TimingInfo time);

			void DoKeyframePose(const IKAnimation::Keyframe& keyframe, float finish);
	};
}
