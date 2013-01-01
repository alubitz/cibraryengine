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

		Limb::JointEntry& joint = limb->joints[limb->joints.size() - 1];
		joint.desired_pyr = Vec3(0, 0, 0);

		// TODO: implement this for real
	}
}
