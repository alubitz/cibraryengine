#include "StdAfx.h"
#include "SkeletalJointConstraint.h"

#include "RigidBody.h"

#define ENFORCE_JOINT_ROTATION_LIMITS   1

#define DEFAULT_TORQUE_LIMIT            150

namespace CibraryEngine
{
	/*
	 *	Reference formulae:
	 *	B = A * M_inv * R * M
	 *	A = B * M_inv * R_inv * M
	 *	R = M * A_inv * B * M_inv
	 *	where A and B are the bones' Mat3 oris, M is the axes matrix (not oriented), and R is Mat3::FromRVec of the clamped rvec
	 */




	/*
	 * SkeletalJointConstraint methods
	 */
	SkeletalJointConstraint::SkeletalJointConstraint(RigidBody* ibody, RigidBody* jbody, const Vec3& pos, const Mat3& axes, const Vec3& min_extents, const Vec3& max_extents) :
		PhysicsConstraint(ibody, jbody),
		pos(pos),
		axes(axes),
		min_extents(min_extents),
		max_extents(max_extents),
		desired_ori(Quaternion::Identity()),
		enable_motor(false),
		min_torque(-DEFAULT_TORQUE_LIMIT, -DEFAULT_TORQUE_LIMIT, -DEFAULT_TORQUE_LIMIT),
		max_torque( DEFAULT_TORQUE_LIMIT,  DEFAULT_TORQUE_LIMIT,  DEFAULT_TORQUE_LIMIT),
		apply_torque()
	{
	}

	// function to trim a few ops from the computation of Quaternion::FromRVec(p * t, y * t, r * t)
	static Quaternion QuatFromRVhT(float p, float y, float r, float half_t)
	{
		// assumes t != 0
		if(float magsq = Vec3::MagnitudeSquared(p, y, r))
		{
			float mag = sqrtf(magsq), half = mag * half_t, coeff = sinf(half) / mag;
			return Quaternion(cosf(half), p * coeff, y * coeff, r * coeff);
		}
		else
			return Quaternion::Identity();
	}

	bool SkeletalJointConstraint::DoConstraintAction()
	{
		static const float dv_sq_threshold    = 0.0f;
		static const float alpha_sq_threshold = 0.0f;

		bool wakeup = false;

		// linear stuff
		Vec3 current_dv = obj_b->vel - obj_a->vel + Vec3::Cross(r2, obj_b->rot) - Vec3::Cross(r1, obj_a->rot);

		Vec3 dv = desired_dv - current_dv;
		float magsq = dv.ComputeMagnitudeSquared();
		if(magsq > dv_sq_threshold)
		{
			Vec3 impulse = rlv_to_impulse * dv;

			Vec3 arot = impulse_to_arot * impulse;
			Vec3 brot = impulse_to_brot * impulse;
			obj_a->vel += impulse * obj_a->inv_mass;
			obj_a->rot += arot;
			obj_b->vel -= impulse * obj_b->inv_mass;
			obj_b->rot -= brot;

			net_impulse_linear  += impulse;
			net_impulse_angular += arot + brot;

			wakeup = true;
		}


		// angular stuff
		Vec3 current_av = obj_b->rot - obj_a->rot;
		Vec3 alpha;												// delta-angular-velocity

		if(enable_motor)
			alpha = current_av - desired_av;

#if ENFORCE_JOINT_ROTATION_LIMITS
		// enforce joint rotation limits
		Vec3 proposed_av = current_av - alpha;

		Quaternion proposed_ori = a_to_b * QuatFromRVhT(proposed_av.x, proposed_av.y, proposed_av.z, half_timestep); 
		Vec3 proposed_rvec = oriented_axes * -proposed_ori.ToRVec();

		bool any_changes = false;
		if      (proposed_rvec.x < min_extents.x) { proposed_rvec.x = min_extents.x; any_changes = true; }
		else if (proposed_rvec.x > max_extents.x) { proposed_rvec.x = max_extents.x; any_changes = true; }
		if      (proposed_rvec.y < min_extents.y) { proposed_rvec.y = min_extents.y; any_changes = true; }
		else if (proposed_rvec.y > max_extents.y) { proposed_rvec.y = max_extents.y; any_changes = true; }
		if      (proposed_rvec.z < min_extents.z) { proposed_rvec.z = min_extents.z; any_changes = true; }
		else if (proposed_rvec.z > max_extents.z) { proposed_rvec.z = max_extents.z; any_changes = true; }

		if(any_changes)
		{
			// at least one rotation limit was violated, so we must recompute alpha
			Quaternion actual_ori = Quaternion::FromRVec(oriented_axes.TransposedMultiply(-proposed_rvec));
			Vec3 actual_av = (b_to_a * actual_ori).ToRVec() * inv_timestep;

			alpha = current_av - actual_av;
		}
#endif

		// apply angular velocity changes
		magsq = alpha.ComputeMagnitudeSquared();
		if(magsq > alpha_sq_threshold)
		{
			obj_a->rot += alpha_to_obja * alpha;
			obj_b->rot -= alpha_to_objb * alpha;

			net_impulse_angular += net_moi * alpha;

			wakeup = true;
		}

		return wakeup;
	}

	void SkeletalJointConstraint::DoUpdateAction(float timestep)
	{
		PhysicsConstraint::DoUpdateAction(timestep);

		half_timestep = timestep * 0.5f;
		inv_timestep = 1.0f / timestep;

		Quaternion a_ori = obj_a->GetOrientation();
		Quaternion b_ori = obj_b->GetOrientation();

		a_to_b = a_ori * Quaternion::Reverse(b_ori);
		b_to_a = Quaternion::Reverse(a_to_b);

		net_moi = Mat3::Invert(obj_a->GetInvMoI() + obj_b->GetInvMoI());
		alpha_to_obja = obj_a->inv_moi * net_moi;
		alpha_to_objb = obj_b->inv_moi * net_moi;

		oriented_axes = axes * Quaternion::Reverse(a_ori).ToMat3();


		// force to keep the two halves of the joint together
		Vec3 a_pos = obj_a->GetTransformationMatrix().TransformVec3_1(pos);
		Vec3 b_pos = obj_b->GetTransformationMatrix().TransformVec3_1(pos);

		Vec3 apply_pos = (a_pos + b_pos) * 0.5f;
		desired_dv = (b_pos - a_pos) * -inv_timestep;

		r1 = apply_pos - obj_a->cached_com;
		r2 = apply_pos - obj_b->cached_com;

		// computing rlv-to-impulse matrix
		Mat3 xr1(     0,   r1.z,  -r1.y,
				  -r1.z,      0,   r1.x,
				   r1.y,  -r1.x,      0     );
		Mat3 xr2(     0,   r2.z,  -r2.y,
				  -r2.z,      0,   r2.x,
				   r2.y,  -r2.x,      0     );

		float invmasses = -(obj_a->inv_mass + obj_b->inv_mass);
		Mat3 impulse_to_rlv = Mat3(invmasses, 0, 0, 0, invmasses, 0, 0, 0, invmasses)
			+ xr1 * obj_a->inv_moi * xr1
			+ xr2 * obj_b->inv_moi * xr2;

		rlv_to_impulse = Mat3::Invert(impulse_to_rlv);

		impulse_to_arot = obj_a->inv_moi * xr1;
		impulse_to_brot = obj_b->inv_moi * xr2;

		// constrained orientation stuff
		if(enable_motor)
			desired_av = (Quaternion::Reverse(desired_ori) * a_to_b).ToRVec() * -inv_timestep;

		// applied torque stuff
		Vec3 use_torque(max(min_torque.x, min(max_torque.x, apply_torque.x)),
						max(min_torque.y, min(max_torque.y, apply_torque.y)),
						max(min_torque.z, min(max_torque.z, apply_torque.z)));
		Vec3 world_torque = oriented_axes.TransposedMultiply(use_torque) * timestep;
		obj_a->rot += obj_a->inv_moi * world_torque;
		obj_b->rot -= obj_b->inv_moi * world_torque;

		net_impulse_linear  = Vec3();
		net_impulse_angular = world_torque;
	}

	Vec3 SkeletalJointConstraint::ComputeAveragePosition() const
	{
		Vec3 a_pos = obj_a->GetTransformationMatrix().TransformVec3_1(pos);
		Vec3 b_pos = obj_b->GetTransformationMatrix().TransformVec3_1(pos);
		return (a_pos + b_pos) * 0.5f;
	}
}
