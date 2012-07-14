#pragma once
#include "StdAfx.h"

#include "Physics.h"

#include "Matrix.h"

namespace CibraryEngine
{
	using namespace std;

	class RigidBody;

	class JointConstraint : public PhysicsConstraint
	{
		protected:

			Quaternion desired_ori;
			Quaternion inv_desired;

		public:

			/** Position of the joint in the coordinate system of the first bone */
			Vec3 pos;
			/** Axes of this joint relative to the first bone; x and y axes are swing axes; z axis is twist axis */
			Mat3 axes;
			/** Minimum angle the joint can turn in each direction, in the coordinate system specified by axes */
			Vec3 min_extents;
			/** Maximum angle the joint can turn in each direction, in the coordinate system specified by axes */
			Vec3 max_extents;

			Vec3 angular_damp;



			bool enable_motor;

			JointConstraint(RigidBody* ibody, RigidBody* jbody, const Vec3&, const Mat3& axes, const Vec3& min_extents, const Vec3& max_extents, const Vec3& angular_damp);

			void DoConstraintAction(unordered_set<RigidBody*>& wakeup);



			void SetDesiredOrientation(const Quaternion& ori);
			Quaternion GetDesiredOrientation() const;
	};
}
