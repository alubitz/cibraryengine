#include "StdAfx.h"
#include "CBone.h"

#include "Dood.h"

namespace Test
{
	/*
	 * CBone methods
	 */
	void CBone::Reset(float inv_timestep)
	{
		Vec3 vel = rb->GetLinearVelocity();
		Vec3 rot = rb->GetAngularVelocity();
		net_impulse_linear = (vel - last_vel) * (inv_timestep * rb->GetMass());
		net_impulse_angular = Mat3(rb->GetTransformedMassInfo().moi) * (rot - last_rot) * inv_timestep;
		last_vel = vel;
		last_rot = rot;
		desired_torque = applied_torque = Vec3();
	}

	void CBone::ComputeDesiredTorque(const Quaternion& desired_ori, const Mat3& use_moi, float inv_timestep)
	{
		Quaternion ori      = rb->GetOrientation();
		Vec3 rot            = rb->GetAngularVelocity();

		Vec3 desired_rot    = (desired_ori * Quaternion::Reverse(ori)).ToRVec() * -inv_timestep;
		Vec3 desired_aaccel = (desired_rot - rot) * inv_timestep;

#if 1
		desired_torque = use_moi * desired_aaccel;
#else
		Vec3 full_torque = use_moi * desired_aaccel;
		float torque_approx = 1400.0f;							// TODO: make this less of a hack

		float frac = full_torque.ComputeMagnitude() / torque_approx;
		if(frac <= 1.0f)
			desired_torque = full_torque;
		else
			desired_torque = full_torque * (0.5f / frac);		// expect to spend half the time accelerating to the desired speed, and the remaining half decelerating
#endif
	}
}
