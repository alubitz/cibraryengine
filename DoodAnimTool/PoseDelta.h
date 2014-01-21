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

			PoseDelta(unsigned int bone, const DATKeyframe::KBone& old_state, const DATKeyframe::KBone& new_state) : bone(bone), old_state(old_state), new_state(new_state) { }
	};
}
