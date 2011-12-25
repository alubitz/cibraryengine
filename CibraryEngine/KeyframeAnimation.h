#pragma once

#include "StdAfx.h"
#include "SkeletalAnimation.h"

namespace CibraryEngine
{
	using namespace std;
	using boost::unordered_map;

	/** Class storing the Pose information for a single frame of a KeyframeAnimation */
	struct Keyframe
	{
		/** For these bones, the Pose's influence is specified */
		unordered_map<unsigned int, BoneInfluence> values;

		/** The keyframe to display after this one ends */
		int next;

		/** The duration of this keyframe */
		float duration;

		/** Initializes a Keyframe with a default duration (1 second) */
		Keyframe();
		/** Initializes a Keyframe with the specified duration */
		Keyframe(float duration);
	};

	/** Pose class representing a predefined animation, specified as a series of keyframes */
	class KeyframeAnimation : public Pose
	{
		public:

			/** The name of the KeyframeAnimation */
			string name;

			/** The keyframes of this animation */
			vector<Keyframe> frames;

			/** Index of the current frame in the playback of this animation */
			int current_index;
			/** Time spent on the current frame of the animation */
			float current_time;

			KeyframeAnimation();
			/** Initializes a keyframe animation with the specified name */
			KeyframeAnimation(string name);

			/** Moves through the animation over time */
			bool Advance(TimingInfo time);
			/** Advances the animation and poses bones */
			virtual void UpdatePose(TimingInfo time);
	};
}
