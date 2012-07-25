#include "StdAfx.h"
#include "IKAnimationPose.h"

#include "Dood.h"

namespace Test
{
	/*
	 * IKAnimationPose methods
	 */
	IKAnimationPose::IKAnimationPose(const IKAnimation* anim, Dood* dood) :
		Pose(),
		anim(anim),
		frame_index(0),
		frame_time(0.0f),
		bones(),
		dood(dood)
	{
		// find bones in the dood to match the names specified in the animation
		for(vector<string>::const_iterator iter = anim->bones.begin(); iter != anim->bones.end(); ++iter)
			bones.push_back(dood->posey->skeleton->GetNamedBone(*iter));
	}

	void IKAnimationPose::UpdatePose(TimingInfo time)
	{
		float nu_time = frame_time + time.elapsed;

		const IKAnimation::Keyframe* current_frame = &anim->frames[frame_index];

		// maybe advance to the next frame
		if(nu_time >= current_frame->duration)
		{
			frame_time -= current_frame->duration;

			frame_index = current_frame->next_frame;
			current_frame = &anim->frames[frame_index];				// current_frame refers to something else now!

			if(frame_index == -1)
			{
				SetActive(false);
				return;
			}

			// no skipping IK keyframes!
			if(frame_time > current_frame->duration)
				frame_time = current_frame->duration;
		}

		int next_index = current_frame->next_frame;
		if(next_index == -1 || next_index == frame_index || frame_time == 0.0f)
			DoKeyframePose(*current_frame, *current_frame, 0.0f);
		else
			DoKeyframePose(*current_frame, anim->frames[next_index], frame_time / current_frame->duration);
	}

	void IKAnimationPose::DoKeyframePose(const IKAnimation::Keyframe& a, const IKAnimation::Keyframe& b, float lerp_factor)
	{
		// TODO: implement this
	}
}
