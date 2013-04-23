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
				Bone* attach;						// the bone this limb attaches to, e.g. the pelvis or the carapace bone
				Dood::FootState* foot;

				// TODO: add members to this

				Limb(Dood* dood, Bone* attach, Dood::FootState* foot);
			};
			vector<Limb*> limbs;

			Dood* dood;

			// TODO: add support for balancing a torso


			IKPose(Dood* dood);
			~IKPose();


			void UpdatePose(TimingInfo time);

			void AddLimb(Bone* attach, Dood::FootState* foot);			// TODO: add other params?
	};
}
