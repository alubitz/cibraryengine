#include "StdAfx.h"
#include "IKAnimationPose.h"

#include "../CibraryEngine/IKChain.h"

#include "Dood.h"

namespace Test
{
	/*
	 * IKAnimationPose methods
	 */
	IKAnimationPose::IKAnimationPose(const IKAnimation* anim, Dood* dood) :
		Pose(),
		anim(anim),
		frame_index(-1),
		next_frame(0),
		frame_time(0.0f),
		bones(),
		dood(dood),
		dead(false)
	{
		// find bones in the dood to match the names specified in the animation
		for(vector<string>::const_iterator iter = anim->bones.begin(); iter != anim->bones.end(); ++iter)
			bones.push_back(dood->posey->skeleton->GetNamedBone(*iter));

		// create ik chains from those
		for(vector<IKAnimation::Chain>::const_iterator iter = anim->chains.begin(); iter != anim->chains.end(); ++iter)
			chains.push_back(new StepPose(bones[iter->from], bones[iter->to], dood->mphys));
	}

	IKAnimationPose::~IKAnimationPose()
	{
		for(vector<StepPose*>::iterator iter = chains.begin(); iter != chains.end(); ++iter)
			delete *iter;
		chains.clear();
	}

	void IKAnimationPose::UpdatePose(TimingInfo time)
	{
		if(anim->frames.empty())
			return;

		// do keyframes stuff
		if(frame_index == -1)
		{
			if(next_frame != -1)
			{
				DoKeyframePose(anim->frames[next_frame], time.total + anim->frames[next_frame].duration);
				frame_index = next_frame;
			}
			else
				dead = true;
		}
		else
		{
			frame_time += time.elapsed;

			const IKAnimation::Keyframe& current_frame = anim->frames[frame_index];

			// maybe advance to the next frame
			if(frame_time >= current_frame.duration)
			{
				frame_time = 0.0f;

				frame_index = next_frame;
				next_frame = current_frame.next_frame;

				if(frame_index == -1)
					return;
				else if(next_frame != -1)
					DoKeyframePose(anim->frames[next_frame], time.total + anim->frames[next_frame].duration);
			}
		}

		// update bone poses
		for(vector<StepPose*>::iterator iter = chains.begin(); iter != chains.end(); ++iter)
		{
			StepPose* pose = *iter;

			dood->UpdateIKChain(pose->chain);
			pose->UpdatePose(time);

			for(unordered_map<unsigned int, BoneInfluence>::iterator iter = pose->bones.begin(); iter != pose->bones.end(); ++iter)
				SetBonePose(iter->first, iter->second.ori, iter->second.pos);
		}
	}

	void IKAnimationPose::DoKeyframePose(const IKAnimation::Keyframe& keyframe, float finish)
	{
		for(vector<IKAnimation::Keyframe::ChainState>::const_iterator iter = keyframe.chain_states.begin(); iter != keyframe.chain_states.end(); ++iter)
		{
			chains[iter->chain_index]->Slide(iter->pos, iter->ori, finish);

			// TODO: use iter->weight somehow?
		}
	}
}
