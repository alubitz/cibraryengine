#include "StdAfx.h"
#include "LAStep.h"

#include "Limb.h"

namespace Test
{
	/*
	 * LAStep methods
	 */
	LAStep::LAStep(Limb* limb) : LimbAction(limb), timer(0.0f) { }

	void LAStep::Update(float timestep)
	{
		timer += timestep;

		Limb::JointEntry& shoulder	= limb->joints[0];
		Limb::JointEntry& elbow		= limb->joints[1];
		Limb::JointEntry& wrist		= limb->joints[2];

		wrist.desired_pyr = Vec3(sinf(timer), 0, 0);

		// TODO: implement this for real
	}
}
