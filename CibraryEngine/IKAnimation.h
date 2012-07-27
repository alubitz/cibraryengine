#pragma once
#include "StdAfx.h"

#include "SkeletalAnimation.h"

namespace CibraryEngine
{
	using namespace std;

	struct IKAnimation
	{
		struct Chain
		{
			unsigned int from, to;				// indices into bones vector of IKAnimation

			Chain(unsigned int from, unsigned int to) : from(from), to(to) { }
		};

		struct Keyframe
		{
			// state of one IK chain
			struct ChainState
			{
				unsigned int chain_index;		// indices into chains vector of IKAnimation

				Vec3 pos;						// desired position of "to" bone (end effector) relative to "from" bone
				Quaternion ori;

				float weight;					// fraction of total weight this end effector should be supporting

				ChainState(unsigned int index, const Vec3& pos, const Quaternion& ori, float weight) : chain_index(index), pos(pos), ori(ori), weight(weight) { }
			};
			vector<ChainState> chain_states;

			float duration;						// how long until the next keyframe
			int next_frame;						// index of the next frame (-1 means this is the last frame)

			Keyframe(float duration, int next_frame) : chain_states(), duration(duration), next_frame(next_frame) { }
		};

		vector<Keyframe> frames;

		vector<Chain> chains;
		vector<string> bones;

		void Read(istream& stream);
		void Write(ostream& stream) const;
	};
}