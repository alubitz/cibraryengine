#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	using namespace CibraryEngine;

	class IKPose : public Pose
	{
		public:

			struct Limb
			{
				Dood::FootState* foot;
				// TODO: add members to this

				Limb(Dood::FootState* foot) : foot(foot) { }
			};
			vector<Limb*> limbs;

			Dood* dood;

			// TODO: add support for balancing a torso


			IKPose(Dood* dood);
			~IKPose();


			void UpdatePose(TimingInfo time);

			void AddLimb(Bone* attachment, Dood::FootState* foot);			// TODO: add other params?
	};
}
