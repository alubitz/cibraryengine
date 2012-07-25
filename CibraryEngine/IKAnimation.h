#pragma once
#include "StdAfx.h"

#include "SkeletalAnimation.h"

namespace CibraryEngine
{
	using namespace std;

	struct IKAnimation
	{
		struct Keyframe
		{
			// state of one IK chain
			struct ChainState
			{
				unsigned int from, to;			// indices into bones vector of IKAnimation

				Vec3 pos;						// desired position of "to" bone (end effector) relative to "from" bone
				Quaternion ori;

				float weight;					// fraction of total weight this end effector should be supporting
			};
			vector<ChainState> chain_states;

			float duration;						// how long this frame should last
			int next_frame;						// index of the next frame (-1 means this is the last frame)
		};

		vector<Keyframe> frames;

		vector<string> bones;

		void Read(istream& stream);
		void Write(ostream& stream) const;
	};
}