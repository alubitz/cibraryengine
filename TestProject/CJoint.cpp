#include "StdAfx.h"
#include "CJoint.h"

#include "Dood.h"
#include "CBone.h"

namespace Test
{
	/*
	 * CJoint methods
	 */
	CJoint::CJoint(const Dood* dood, CBone& bone_a, CBone& bone_b, float max_torque)
	{
		RigidBody *arb = bone_a.rb, *brb = bone_b.rb;
		for(unsigned int i = 0; i < dood->constraints.size(); ++i)
		{
			SkeletalJointConstraint* j = (SkeletalJointConstraint*)dood->constraints[i];
			if(j->obj_a == arb && j->obj_b == brb)
			{
				a   = &bone_a;
				b   = &bone_b;
				sjc = j;

				sjc->min_torque = Vec3(-max_torque, -max_torque, -max_torque);
				sjc->max_torque = Vec3( max_torque,  max_torque,  max_torque);

				r1 = sjc->pos - a->local_com;
				r2 = sjc->pos - b->local_com;

				return;
			}
		}

		// joint not found?
		a = b = NULL;
		sjc = NULL;
	}

	void CJoint::Reset()
	{
		last = sjc->apply_torque;

		//sjc->apply_torque = actual = Vec3();
		oriented_axes = sjc->axes * Quaternion::Reverse(sjc->obj_a->GetOrientation()).ToMat3();
	}

	Vec3 CJoint::GetRVec() const
	{
		Quaternion a_to_b = sjc->obj_a->GetOrientation() * Quaternion::Reverse(sjc->obj_b->GetOrientation());
		return oriented_axes * a_to_b.ToRVec();
	}

	// returns true if UNABLE to match the requested value
	bool CJoint::SetWorldTorque(const Vec3& torque)
	{
		Vec3 local_torque = oriented_axes * torque;

		const Vec3 &mint = sjc->min_torque, &maxt = sjc->max_torque;

		Vec3 old = oriented_axes.TransposedMultiply(sjc->apply_torque);

		sjc->apply_torque.x = max(mint.x, min(maxt.x, local_torque.x));
		sjc->apply_torque.y = max(mint.y, min(maxt.y, local_torque.y));
		sjc->apply_torque.z = max(mint.z, min(maxt.z, local_torque.z));

		Vec3 dif = sjc->apply_torque - local_torque;
		bool result = (dif.x != 0 || dif.y != 0 || dif.z != 0);

		if(result)
			actual = oriented_axes.TransposedMultiply(sjc->apply_torque);
		else
			actual = torque;

		Vec3 delta = actual - old;

		b->applied_torque -= delta;
		a->applied_torque += delta;

		return result;
	}

	bool CJoint::SetTorqueToSatisfyA() { return SetWorldTorque(a->desired_torque - (a->applied_torque - actual)); }
	bool CJoint::SetTorqueToSatisfyB() { return SetWorldTorque((b->applied_torque + actual) - b->desired_torque); }

	bool CJoint::SetOrientedTorque(const Vec3& local_torque)
	{
		Vec3 old = oriented_axes.TransposedMultiply(sjc->apply_torque);

		const Vec3 &mint = sjc->min_torque, &maxt = sjc->max_torque;

		sjc->apply_torque.x = max(mint.x, min(maxt.x, local_torque.x));
		sjc->apply_torque.y = max(mint.y, min(maxt.y, local_torque.y));
		sjc->apply_torque.z = max(mint.z, min(maxt.z, local_torque.z));

		Vec3 dif = sjc->apply_torque - local_torque;
		bool result = (dif.x != 0 || dif.y != 0 || dif.z != 0);

		actual = oriented_axes.TransposedMultiply(sjc->apply_torque);

		Vec3 delta = actual - old;

		b->applied_torque -= delta;
		a->applied_torque += delta;

		return result;
	}
}
