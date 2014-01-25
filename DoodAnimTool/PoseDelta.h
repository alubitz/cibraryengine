#pragma once

#include "StdAfx.h"

#include "DATKeyframe.h"

namespace DoodAnimTool
{
	// class for representing one part of a change to a pose (e.g. the user trying to rotate a particular bone)
	struct PoseDelta
	{
		public:

			unsigned int bone;					// index into the DATBone array

			DATKeyframe::KBone old_state;
			DATKeyframe::KBone new_state;

			bool score_ori;
			bool score_pos;
			
			PoseDelta() : bone(-1), score_ori(false), score_pos(false) { }

			PoseDelta(unsigned int bone, const DATKeyframe::KBone& old_state, const DATKeyframe::KBone& new_state, bool score_pos, bool score_ori) :
				bone(bone),
				old_state(old_state),
				new_state(new_state),
				score_pos(score_pos),
				score_ori(score_ori)
			{
			}
	};
}
