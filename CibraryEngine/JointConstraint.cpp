#include "StdAfx.h"
#include "JointConstraint.h"

#include "RigidBody.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * JointConstraint methods
	 */
	JointConstraint::JointConstraint(RigidBody* ibody, RigidBody* jbody, const Vec3& pos, const Mat3& axes, const Vec3& max_extents, const Vec3& angular_damp) :
		pos(pos),
		axes(axes),
		max_extents(max_extents),
		angular_damp(angular_damp)
	{
		inv_a_xform = Mat4::Invert(Mat4::FromPositionAndOrientation(pos, axes));

		obj_a = ibody;
		obj_b = jbody;
	}

	void JointConstraint::DoConstraintAction(unordered_set<RigidBody*>& wakeup_list)
	{
		/*
		if(dist > 0)
		{
			float A, B;
			PhysicsWorld::GetUseMass(obj_a, obj_b, apply_pos, dir, A, B);

			float bounciness = 0.0f;
			float dv_mag = -(1.0f + bounciness) * B;
			if(fabs(dv_mag) > 0.0f)
			{
				Vec3 impulse = dir * (dv_mag / A);

				obj_a->ApplyImpulse(impulse, i_poi);
				obj_b->ApplyImpulse(-impulse, j_poi);

				wakeup_list.insert(obj_a);
				wakeup_list.insert(obj_b);
			}
		}
		*/
	}

	void JointConstraint::DoUpdateAction(float timestep)
	{
		Vec3 offset = obj_b->GetTransformationMatrix().TransformVec3_1(pos) - obj_a->GetTransformationMatrix().TransformVec3_1(pos);

		float dist_sq = offset.ComputeMagnitudeSquared();
		if(dist_sq > 0.00001f)
		{
			dist = sqrtf(dist_sq);
			dir = offset / dist;

			apply_pos = obj_a->GetTransformationMatrix().TransformVec3_1(pos);

			i_poi = obj_a->GetInvTransform().TransformVec3_1(apply_pos);
			j_poi = obj_b->GetInvTransform().TransformVec3_1(apply_pos);

			float A, B;
			PhysicsWorld::GetUseMass(obj_a, obj_b, apply_pos, dir, A, B);

			Vec3 impulse = offset * ((1000.0f * timestep - B / offset.ComputeMagnitude()) / A);

			obj_a->ApplyImpulse(impulse, i_poi);
			obj_b->ApplyImpulse(-impulse, j_poi);
		}
		else
			dist = 0.0f;
	}
}
