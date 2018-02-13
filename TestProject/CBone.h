#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class Dood;

	struct CBone
	{
		string name;
		RigidBody* rb;
		Bone* posey;

		Vec3 local_com;

		Vec3 desired_torque;
		Vec3 applied_torque;

		Vec3 desired_force;

		Vec3 initial_pos;
		Quaternion initial_ori;

		Vec3 last_vel, last_rot;
		Vec3 net_impulse_linear, net_impulse_angular;

		CBone() { }
		CBone(const Dood* dood, const string& name);

		void Reset(float inv_timestep);

		void ComputeDesiredTorque(const Quaternion& desired_ori, const Mat3& use_moi, float inv_timestep);

		void ComputeDesiredTorqueWithDefaultMoI(const Quaternion& desired_ori, float inv_timestep) { ComputeDesiredTorque(desired_ori, Mat3(rb->GetTransformedMassInfo().moi), inv_timestep); }
		void ComputeDesiredTorqueWithPosey(const Mat3& use_moi, float inv_timestep)                { ComputeDesiredTorque(posey->GetTransformationMatrix().ExtractOrientation(), use_moi, inv_timestep); }
		void ComputeDesiredTorqueWithDefaultMoIAndPosey(float inv_timestep)                        { ComputeDesiredTorque(posey->GetTransformationMatrix().ExtractOrientation(), Mat3(rb->GetTransformedMassInfo().moi), inv_timestep); }
	};
}
