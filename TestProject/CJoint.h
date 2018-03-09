#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	struct CBone;
	class Dood;

	// wraps a SkeletalJointConstraint
	struct CJoint
	{
		SkeletalJointConstraint* sjc;	// reference to the raw constraint object used by the physical simulation
		CBone *a, *b;					// reference to the parent and child bones

		Vec3 actual;					// world-coords torque to be applied by this joint

		Mat3 oriented_axes;				// cache the custom coordinate system of this joint (e.g. knee joint rotates on only one axis, which is not quite the X axis); gets recomputed every time Reset() is called

		CJoint() : sjc(NULL), a(NULL), b(NULL) { }
		CJoint(const Dood* dood, CBone& bone_a, CBone& bone_b, float max_torque) : CJoint(dood, bone_a, bone_b, max_torque, max_torque, max_torque) { }
		CJoint(const Dood* dood, CBone& bone_a, CBone& bone_b, float x, float y, float z);

		void Reset();					// called in preparation for the physics step callback

		Vec3 GetRVec() const;			// get the relative orienation of the child bone wrt the parent bone, expressed as a rotation vector in the joint's custom coordinate system

		/**
		 * Attempts to set the world-coords torque for this joint to apply (parent gets a +impulse, child gets a -impulse). Updates the applied_torque field of both bones.
		 *
		 * @return false if the requested value was matched exactly, or true if it had to be truncated on one or more axes
		 */
		bool SetWorldTorque(const Vec3& torque);

		/**
		 * Sets the torque for this joint to apply to a value that will satisfy the desired torque of the parent bone. Updates the applied_torque field of both bones.
		 *
		 * @return false if the requested value was matched exactly, or true if it had to be truncated on one or more axes
		 */
		bool SetTorqueToSatisfyA();

		/**
		 * Sets the torque for this joint to apply to a value that will satisfy the desired torque of the child bone. Updates the applied_torque field of both bones.
		 *
		 * @return false if the requested value was matched exactly, or true if it had to be truncated on one or more axes
		 */
		bool SetTorqueToSatisfyB();

		/**
		 * Sets the custom-coords torque for this joint to apply. Updates the applied_torque field of both bones.
		 *
		 * @return false if the requested value was matched exactly, or true if it had to be truncated on one or more axes
		 */
		bool SetOrientedTorque(const Vec3& local_torque);
	};
}
