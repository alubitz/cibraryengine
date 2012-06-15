#include "StdAfx.h"
#include "JointConstraint.h"

#include "RigidBody.h"

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

	void JointConstraint::DoConstraintAction(set<RigidBody*>& wakeup_list)
	{
		Mat4 relative_xform = obj_a->GetInvTransform() * obj_b->GetTransformationMatrix();

		relative_xform *= inv_a_xform;

		Vec3 translation, scale;
		Quaternion ori;
		relative_xform.Decompose(translation, ori, scale);

		/*
		Mat3 ori_rm = ori.ToMat3();
		Mat3 inv_ori = ori_rm.Transpose();

		Vec3 vel = inv_ori * (obj_b->GetLinearVelocity() - obj_a->GetLinearVelocity());
		Vec3 avel = inv_ori * (obj_b->GetAngularVelocity() - obj_a->GetAngularVelocity());

		Vec3 ori_vector = ori.ToPYR();

		float mag = (ori_vector / max_extents).ComputeMagnitudeSquared();
		*/

		// TODO: make this awesomer

		float dist_sq = translation.ComputeMagnitudeSquared();
		if(dist_sq > 0.00001f)
		{
			translation /= sqrtf(dist_sq);

			Vec3 apply_pos = obj_a->GetTransformationMatrix().TransformVec3_1(pos);

			float A, B;
			PhysicsWorld::GetUseMass(obj_a, obj_b, apply_pos, translation, A, B);

			float bounciness = 0.2f;
			float impulse_mag = -(1.0f + bounciness) * B / A;
			if(fabs(impulse_mag) > 0.01f)
			{
				Vec3 impulse = translation * impulse_mag;

				Vec3 i_poi = obj_a->GetInvTransform().TransformVec3_1(apply_pos);
				Vec3 j_poi = obj_b->GetInvTransform().TransformVec3_1(apply_pos);

				obj_a->ApplyImpulse(impulse, i_poi);
				obj_b->ApplyImpulse(-impulse, j_poi);

				wakeup_list.insert(obj_a);
				wakeup_list.insert(obj_b);
			}
		}
	}
}
