#include "StdAfx.h"
#include "CBone.h"

#include "Dood.h"

namespace Test
{
	/*
	 * CBone methods
	 */
	CBone::CBone(const Dood* dood, const string& name) : name(name), rb(dood->RigidBodyForNamedBone(name)), posey(dood->posey->skeleton->GetNamedBone(name)), initial_pos(rb->GetPosition()), initial_ori(rb->GetOrientation()), last_vel(rb->GetLinearVelocity()), last_rot(rb->GetAngularVelocity()), net_impulse_linear(), net_impulse_angular() { }

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

		desired_torque = use_moi * desired_aaccel;
	}
}
