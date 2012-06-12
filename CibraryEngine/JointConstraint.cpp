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

		Mat3 ori_rm = ori.ToMat3();
		Mat3 inv_ori = ori_rm.Transpose();

		Vec3 vel = inv_ori * (obj_b->GetLinearVelocity() - obj_a->GetLinearVelocity());
		Vec3 avel = inv_ori * (obj_b->GetAngularVelocity() - obj_a->GetAngularVelocity());

		Vec3 ori_vector = ori.ToPYR();

		float mag = (ori_vector / max_extents).ComputeMagnitudeSquared();

		// TODO: apply impulses or angular impulses as appropriate
	}
}
