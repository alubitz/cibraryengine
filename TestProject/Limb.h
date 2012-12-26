#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class LimbAction;

	class Limb
	{
		public:

			struct JointEntry
			{
				JointConstraint* constraint;
				Quaternion desired_ori;

				JointEntry(JointConstraint* constraint, const Quaternion& desired_ori = Quaternion::Identity()) : constraint(constraint), desired_ori(desired_ori) { }
			};

			vector<JointEntry> joints;
			vector<RigidBody*> rigid_bodies;				// there is one more rigid body than there are joints

			vector<Vec3> bone_torques;						// temporary values used within Update; cache with object to avoid multiple allocs/de-allocs

			Vec3 applied_torque;							// total of torques applied within the chain of bones/joints, as measured at the base joint (e.g. shoulder)

			LimbAction* action;

			Limb();
			~Limb();

			void Update(float timestep);
	};
}
