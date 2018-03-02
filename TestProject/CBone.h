#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class Dood;

	// wraps a RigidBody
	struct CBone
	{
		string name;
		RigidBody* rb;						// reference to the raw RigidBody object used by the physical simulation
		Bone* posey;						// posey --> physics --> character

		Vec3 desired_torque;				// if we know what the net torque we want to apply to this bone is, we can put that here
		Vec3 applied_torque;				// running total of the effects of adjacent joints' applied torques on this bone

		Vec3 initial_pos;					// records the initial position (not the same as CoM) of this bone (for use in resetting the dood without killing/respawning it)
		Quaternion initial_ori;				// the same but for orientation

		Vec3 last_vel, last_rot;			// records the most recent vel/rot of this bone, so that when it updates we can compute the force/torque
		Vec3 net_impulse_linear, net_impulse_angular;	// measurement of the net force/torque this bone experienced during the most recent simulation step

		CBone() { }
		CBone(const Dood* dood, const string& name);

		void Reset(float inv_timestep);		// roll over last_vel and last_rot; compute net impulses; reset desired and applied torques

		/**
		 * Computes the net torque required to get this bone to the specified orientation by the next simulation step, and stores it in desired_torque
		 * This is done using the "Get Me There Immediately" formula
		 *
		 * Sometimes it may be useful to explicitly specify the MoI to use, instead of using that of the RigidBody directly
		 */
		void ComputeDesiredTorque(const Quaternion& desired_ori, const Mat3& use_moi, float inv_timestep);

		// computes desired_torque using the RigidBody's MoI and an explicitly specified desired orientation
		void ComputeDesiredTorqueWithDefaultMoI(const Quaternion& desired_ori, float inv_timestep) { ComputeDesiredTorque(desired_ori, Mat3(rb->GetTransformedMassInfo().moi), inv_timestep); }

		// computes desired_torque using the desired orientation specified by the posey bone, and using the specified MoI
		void ComputeDesiredTorqueWithPosey(const Mat3& use_moi, float inv_timestep)                { ComputeDesiredTorque(posey->GetTransformationMatrix().ExtractOrientation(), use_moi, inv_timestep); }

		// computes desired_torque using the desired orientation specified by the posey bone, and using the RigidBody's MoI
		void ComputeDesiredTorqueWithDefaultMoIAndPosey(float inv_timestep)                        { ComputeDesiredTorque(posey->GetTransformationMatrix().ExtractOrientation(), Mat3(rb->GetTransformedMassInfo().moi), inv_timestep); }
	};
}
