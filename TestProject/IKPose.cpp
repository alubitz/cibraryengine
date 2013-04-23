#include "StdAfx.h"
#include "IKPose.h"

namespace Test
{
	/*
	 * IKPose::Limb methods
	 */
	IKPose::Limb::Limb(Dood* dood, Bone* attach, Dood::FootState* foot) : attach(attach), foot(foot)
	{
		// TODO: do additional initialization stuff here
	}




	/*
	 * IKPose methods
	 */
	IKPose::IKPose(Dood* dood) : limbs(), dood(dood) { }

	IKPose::~IKPose()
	{
		for(vector<Limb*>::iterator iter = limbs.begin(); iter != limbs.end(); ++iter)
			delete *iter;
		limbs.clear();
	}


	void IKPose::UpdatePose(TimingInfo time)
	{
		/*
		 *	TODO: something like the following?
		 *
		 *	1. figure out imbalance, desired acceleration, and stuff like that
		 *	2. position and orient the torso so it balances, faces forward, and moves in the right direction
		 *	3. figure out how to pose the limbs connected to the torso
		 *
		 *	pelvis stuff:
		 *		vertical position is about 0.9 meters above the lowest of the two feet
		 *		horizontal position is about the average of the two feet's horizontal positions
		 *		height decreases if feet are far apart
		 *
		 *	upper body stuff:
		 *		"head" faces the direction the dood is looking
		 *		"torso 2" faces the correct direction to aim the gun at a target (approx)
		 *		"torso 1" and "torso 2" may need to pivot along with the head to allow it the full range of motion
		 *		com for upper body is approximately above the pelvis
		 */
	}


	void IKPose::AddLimb(Bone* attach, Dood::FootState* foot)
	{
		// TODO: implement this for real
		limbs.push_back(new Limb(dood, attach, foot));
	}
}
