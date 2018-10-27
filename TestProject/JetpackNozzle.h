#pragma once

#include "StdAfx.h"

namespace Test
{
	struct CBone;

	using namespace CibraryEngine;

	struct JetpackNozzle
	{
		CBone* bone;

		Vec3 pos;
		Vec3 cone_center;
		float cone_cossq;
		float max_force, max_forcesq;

		Vec3 world_force, world_torque;
		Vec3 try_force, try_torque;

		Vec3 world_center;
		Vec3 apply_pos;
		Mat3 force_to_torque;

		JetpackNozzle(CBone* bone, const Vec3& pos, const Vec3& cone_center, float cone_angle, float max_force);

		void Reset();

		void SolverInit(const Vec3& dood_com, float prop_frac);

		void GetNudgeEffects(const Vec3& nudge, Vec3& nu_force, Vec3& nu_torque);

		void ApplySelectedForce(float timestep);



		void Vis(SceneRenderer* renderer, const Vec3& forward, BillboardMaterial* jetpack_trail) const;
	};
}