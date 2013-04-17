#include "StdAfx.h"
#include "IKPose.h"

namespace Test
{
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
		// TODO: implement this
	}


	void IKPose::AddLimb(Bone* attachment, Dood::FootState* foot)
	{
		// TODO: implement this for real
		limbs.push_back(new Limb(foot));
	}
}
