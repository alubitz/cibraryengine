#pragma once
#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class Dood;

	class DoodOrientationConstraint : public PhysicsConstraint
	{
		private:

			Dood* dood;

		public:

			DoodOrientationConstraint(Dood* dood);

			void DoConstraintAction(vector<RigidBody*>& wakeup_list);

			Quaternion desired_ori;
	};
}