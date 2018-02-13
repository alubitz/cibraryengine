#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	struct CBone;
	class Dood;

	struct CJoint
	{
		SkeletalJointConstraint* sjc;
		CBone *a, *b;

		Vec3 actual;				// world-coords torque to be applied by this joint
		Vec3 last;

		Mat3 oriented_axes;			// gets recomputed every time Reset() is called

		Mat3 last_desired;

		Vec3 r1, r2;

		Vec3 initial_rvec;
		Vec3 goal_rvec;
		Vec3 prev_goal;
		Vec3 integral;

		int mode;
		Vec3 pd_result;
		float chain_coeff;

		CJoint() : sjc(NULL), a(NULL), b(NULL), mode(0) { }
		CJoint(const Dood* dood, CBone& bone_a, CBone& bone_b, float max_torque, int mode = 0);

		void Reset();

		Vec3 GetRVec() const;

		// returns true if UNABLE to match the requested value
		bool SetWorldTorque(const Vec3& torque);

		bool SetTorqueToSatisfyA();
		bool SetTorqueToSatisfyB();

		bool SetOrientedTorque(const Vec3& local_torque);
	};
}
