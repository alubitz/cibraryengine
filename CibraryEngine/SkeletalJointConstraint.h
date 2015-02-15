#pragma once

#include "StdAfx.h"

#include "Physics.h"

#include "Matrix.h"

namespace CibraryEngine
{
	using namespace std;

	class RigidBody;

	class SkeletalJointConstraint : public PhysicsConstraint
	{
		protected:

			// some values that are set by DoUpdateAction and which should only be used for the duration of a single physics tick
			Vec3 desired_dv;
			Vec3 desired_av;

			Mat3 oriented_axes;
			Quaternion a_to_b, b_to_a;

			Vec3 r1, r2;
			Mat3 rlv_to_impulse;
			Mat3 impulse_to_arot, impulse_to_brot;
			Mat3 alpha_to_obja, alpha_to_objb;
			Mat3 net_moi;

			float half_timestep, inv_timestep;

		public:

			/** Position of the joint in the coordinate system of the first bone */
			Vec3 pos;
			/** Axes of this joint relative to the first bone; x and y axes are swing axes; z axis is twist axis */
			Mat3 axes;
			/** Minimum angle the joint can turn in each direction, in the coordinate system specified by axes */
			Vec3 min_extents;
			/** Maximum angle the joint can turn in each direction, in the coordinate system specified by axes */
			Vec3 max_extents;

			Quaternion desired_ori;
			bool enable_motor;

			// applied torque stuff, specified relative to the joint axes
			Vec3 min_torque, max_torque;
			Vec3 apply_torque;	

			SkeletalJointConstraint(RigidBody* ibody, RigidBody* jbody, const Vec3&, const Mat3& axes, const Vec3& min_extents, const Vec3& max_extents);

			bool DoConstraintAction();
			void DoUpdateAction(float timestep);

			/** Convenience method to compute the position of this joint, averaging the positions the two bones say it should be */
			Vec3 ComputeAveragePosition() const;
	};
}
