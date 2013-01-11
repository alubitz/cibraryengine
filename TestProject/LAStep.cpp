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
		timer += timestep * 2.0f;

		Limb::JointEntry& shoulder	= limb->joints[0];
		Limb::JointEntry& elbow		= limb->joints[1];
		Limb::JointEntry& wrist		= limb->joints[2];

		shoulder.desired_pyr	= Vec3(	0.5f * cosf(timer) + 0.5f,	sinf(timer),	0	);
		elbow.desired_pyr		= Vec3(	0.5f * sinf(timer),			0,				0	);
		wrist.desired_pyr		= Vec3(	-0.5f * sinf(timer),		0,				0	);

		// TODO: implement this for real
	}
}
