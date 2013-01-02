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
				Vec3 desired_pyr;

				// pid stuff
				Vec3 error_integral;
				Vec3 old_error;
				Vec3 pid_coeffs;

				// temporary values
				Mat3 oriented_axes;
				Vec3 bone_torque;

				JointEntry(JointConstraint* constraint, const Vec3& pid_coeffs = Vec3(1.0f, 1.0f, 1.0f)) : constraint(constraint), desired_pyr(), error_integral(), old_error(), pid_coeffs(pid_coeffs) { }
			};

			vector<JointEntry> joints;
			vector<RigidBody*> rigid_bodies;				// there is one more rigid body than there are joints

			Vec3 applied_torque;							// total of torques applied within the chain of bones/joints, as measured at the base joint (e.g. shoulder)

			LimbAction* action;

			Limb(JointConstraint** joints, RigidBody** rigid_bodies, unsigned int count);		// count = number of joints; rigid bodies should have 1 more than this
			~Limb();

			void Update(float timestep);
	};
}
