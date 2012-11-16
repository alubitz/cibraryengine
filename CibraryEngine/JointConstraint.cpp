#include "StdAfx.h"
#include "JointConstraint.h"

#include "RigidBody.h"

namespace CibraryEngine
{
	/*
	 * JointConstraint methods
	 */
	JointConstraint::JointConstraint(RigidBody* ibody, RigidBody* jbody, const Vec3& pos, const Mat3& axes, const Vec3& min_extents, const Vec3& max_extents) :
		PhysicsConstraint(ibody, jbody),
		pos(pos),
		axes(axes),
		min_extents(min_extents),
		max_extents(max_extents)
	{
	}

	void JointConstraint::DoConstraintAction()
	{
		static const float dv_coeff =			1.0f;

		static const float dv_sq_threshold	=	0.01f;
		static const float alpha_sq_threshold =	0.01f;


		// linear stuff
		Vec3 current_dv = obj_b->vel - obj_a->vel + Vec3::Cross(r2, obj_b->rot) - Vec3::Cross(r1, obj_a->rot);

		Vec3 dv = desired_dv - current_dv;
		float magsq = dv.ComputeMagnitudeSquared();
		if(magsq > dv_sq_threshold)
		{
			Vec3 impulse = dv * (-dv_coeff * PhysicsWorld::GetUseMass(obj_a, obj_b, apply_pos, dv / sqrtf(magsq)));

			// apply impulse
			if(obj_a->active)
			{
				obj_a->vel += impulse * obj_a->inv_mass;
				if(obj_a->can_rotate)
					obj_a->rot += obj_a->inv_moi * Vec3::Cross(impulse, r1);
			}
			if(obj_b->can_move && obj_b->active)
			{
				obj_b->vel -= impulse * obj_b->inv_mass;
				if(obj_b->can_rotate)
					obj_b->rot -= obj_b->inv_moi * Vec3::Cross(impulse, r2);
			}
		}


		// angular stuff
		Vec3 current_av = obj_b->rot - obj_a->rot;
		Vec3 alpha;												// delta-angular-velocity

#if 1
		// enforce joint rotation limits
		Vec3 proposed_av = current_av - alpha;
		Quaternion proposed_ori = a_to_b * Quaternion::FromPYR(proposed_av.x * timestep, proposed_av.y * timestep, proposed_av.z * timestep);
		Vec3 proposed_pyr = oriented_axes * -proposed_ori.ToPYR();

		bool any_changes = false;
		if      (proposed_pyr.x < min_extents.x) { proposed_pyr.x = min_extents.x; any_changes = true; }
		else if (proposed_pyr.x > max_extents.x) { proposed_pyr.x = max_extents.x; any_changes = true; }
		if      (proposed_pyr.y < min_extents.y) { proposed_pyr.y = min_extents.y; any_changes = true; }
		else if (proposed_pyr.y > max_extents.y) { proposed_pyr.y = max_extents.y; any_changes = true; }
		if      (proposed_pyr.z < min_extents.z) { proposed_pyr.z = min_extents.z; any_changes = true; }
		else if (proposed_pyr.z > max_extents.z) { proposed_pyr.z = max_extents.z; any_changes = true; }

		if(any_changes)
		{
			// at least one rotation limit was violated, so we must recompute alpha
			Quaternion actual_ori = Quaternion::FromPYR(reverse_oriented_axes * -proposed_pyr);
			Vec3 actual_av = (b_to_a * actual_ori).ToPYR() * inv_timestep;

			alpha = current_av - actual_av;
		}
#endif

		// apply angular velocity changes
		magsq = alpha.ComputeMagnitudeSquared();
		if(magsq > alpha_sq_threshold)
		{
			obj_a->rot += alpha_to_obja * alpha;
			obj_b->rot -= alpha_to_objb * alpha;
		}
	}

	void JointConstraint::DoUpdateAction(float timestep_)
	{
		timestep = timestep_;
		inv_timestep = 1.0f / timestep;

		const float spring_coeff =				1.0f;

		Quaternion a_ori = obj_a->GetOrientation();
		Quaternion b_ori = obj_b->GetOrientation();

		a_to_b = Quaternion::Reverse(a_ori) * b_ori;
		b_to_a = Quaternion::Reverse(a_to_b);

		Mat3 net_moi = Mat3::Invert(obj_a->GetInvMoI() + obj_b->GetInvMoI());
		alpha_to_obja = obj_a->inv_moi * net_moi;
		alpha_to_objb = obj_b->inv_moi * net_moi;

		oriented_axes = axes.Transpose() * a_ori.ToMat3();
		reverse_oriented_axes = oriented_axes.Transpose();


		// force to keep the two halves of the joint together
		Vec3 a_pos = obj_a->GetTransformationMatrix().TransformVec3_1(pos);
		Vec3 b_pos = obj_b->GetTransformationMatrix().TransformVec3_1(pos);

		apply_pos = (a_pos + b_pos) * 0.5f;
		desired_dv = (b_pos - a_pos) * -(spring_coeff * inv_timestep);

		r1 = apply_pos - obj_a->cached_com;
		r2 = apply_pos - obj_b->cached_com;
	}

	void JointConstraint::WriteDataToBuffer(float* ptr)
	{
		Mat3 net_moi = Mat3::Invert(obj_a->GetInvMoI() + obj_b->GetInvMoI());

		ptr[0] =	-1.0f;
		ptr[1] =	desired_dv.x;
		ptr[2] =	desired_dv.y;
		ptr[3] =	desired_dv.z;

		ptr[4] =	apply_pos.x;
		ptr[5] =	apply_pos.y;
		ptr[6] =	apply_pos.z;
		ptr[7] =	oriented_axes[0];

		ptr[8] =	oriented_axes[1];
		ptr[9] =	oriented_axes[2];
		ptr[10] =	oriented_axes[3];
		ptr[11] =	oriented_axes[4];

		ptr[12] =	oriented_axes[5];
		ptr[13] =	oriented_axes[6];
		ptr[14] =	oriented_axes[7];
		ptr[15] =	oriented_axes[8];

		ptr[16] =	a_to_b.x;
		ptr[17] =	a_to_b.y;
		ptr[18] =	a_to_b.z;
		ptr[19] =	a_to_b.w;

		ptr[20] =	min_extents.x;
		ptr[21] =	min_extents.y;
		ptr[22] =	min_extents.z;
		ptr[23] =	net_moi[0];

		ptr[24] =	net_moi[1];
		ptr[25] =	net_moi[2];
		ptr[26] =	net_moi[3];
		ptr[27] =	net_moi[4];

		ptr[28] =	net_moi[5];
		ptr[29] =	net_moi[6];
		ptr[30] =	net_moi[7];
		ptr[31] =	net_moi[8];

		ptr[32] =	max_extents.x;
		ptr[33] =	max_extents.y;
		ptr[34] =	max_extents.z;
		// ptr[35] is unused
	}
}
