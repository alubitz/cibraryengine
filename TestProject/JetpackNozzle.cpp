#include "StdAfx.h"
#include "JetpackNozzle.h"

#include "CBone.h"

namespace Test
{
	/*
	 * JetpackNozzle methods
	 */
	JetpackNozzle::JetpackNozzle(CBone& bone, const Vec3& pos, const Vec3& cone_center, float cone_angle, float max_force) : bone(&bone), pos(pos), cone_center(cone_center), cone_cossq(cosf(cone_angle)), max_force(max_force), max_forcesq(max_force * max_force) { cone_cossq *= cone_cossq; }

	void JetpackNozzle::Reset() { world_force = Vec3(); }

	void JetpackNozzle::SolverInit(const Vec3& dood_com, float prop_frac)
	{
		const RigidBody& rb = *bone->rb;
		Mat3 rm = rb.GetOrientation().ToMat3();
		world_center = rm * cone_center;
		apply_pos    = rm * pos + rb.GetPosition();

		// compute force-to-torque Mat3
		Vec3 bone_com = rb.GetCenterOfMass();
		Vec3 r1 = apply_pos - bone_com;
		Mat3 xr1 = Mat3(        0,   r1.z,  -r1.y,
			-r1.z,      0,   r1.x,
			r1.y,  -r1.x,      0	);
		Vec3 r2 = bone_com - dood_com;
		Mat3 xr2 = Mat3(        0,   r2.z,  -r2.y,
			-r2.z,      0,   r2.x,
			r2.y,  -r2.x,      0	);
		force_to_torque = xr1 + xr2;			// is this right?


		world_force  = world_center * max_force * prop_frac;
		world_torque = force_to_torque * world_force;

		try_force  = world_force;
		try_torque = world_torque;
	}

	void JetpackNozzle::GetNudgeEffects(const Vec3& nudge, Vec3& nu_force, Vec3& nu_torque)
	{
		nu_force = world_force + nudge;


		float dot = Vec3::Dot(nu_force, world_center);
		if(dot <= 0.0f)
			nu_force = nu_torque = Vec3();
		else
		{
			Vec3 axial = world_center * dot;
			Vec3 ortho = nu_force - axial;
			float axialsq = axial.ComputeMagnitudeSquared();
			float orthosq = ortho.ComputeMagnitudeSquared();
			if(orthosq > axialsq * cone_cossq)
			{
				ortho *= sqrtf(axialsq * cone_cossq / orthosq);
				nu_force = axial + ortho;
			}

			float magsq = nu_force.ComputeMagnitudeSquared();
			if(magsq > max_forcesq)
				nu_force *= sqrtf(max_forcesq / magsq);

			nu_torque = force_to_torque * nu_force;
		}
	}

	void JetpackNozzle::ApplySelectedForce(float timestep)
	{
		GetNudgeEffects(Vec3(), world_force, world_torque);

		//bone->rb->ApplyWorldForce(world_force, apply_pos);				// TODO: make this work?
		bone->rb->ApplyWorldImpulse(world_force * timestep, apply_pos);
	}
}
