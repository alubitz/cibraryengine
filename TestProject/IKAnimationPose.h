#pragma once
#include "StdAfx.h"

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
			vector<Bone*> bones;					// values are parallel to those of anim->bones

			int frame_index;
			float frame_time;						// time since [most recent] arrival at this keyframe

			Dood* dood;								// owner of this pose

		public:

			IKAnimationPose(const IKAnimation* anim, Dood* dood);

			void UpdatePose(TimingInfo time);

			void DoKeyframePose(const IKAnimation::Keyframe& a, const IKAnimation::Keyframe& b, float lerp_factor);
	};
}
